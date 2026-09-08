// Execute the actual channel implementation against a clock/GPIO register model.
#include "channels.h"
#include "timebase.h"
#include <cassert>
#include <cmath>
#include <limits>
namespace timebase {
    double nominalHz() { return 75000000.0; }
    uint64_t now() { set_tick(fake_tick + now_step); return fake_tick; }
}
static constexpr uint64_t hz = 75000000;
static void restart(uint64_t tick) {
    channels::setEnabled(false);
    channels::resetCounts();
    set_tick(tick);
    now_step = 25;
    pin_edges.clear();
}
static void run_until(uint64_t end) {
    unsigned calls = 0;
    while (GPT1_IR & SS_GPT_IR_OF1IE) {
        const uint32_t delta = GPT1_OCR1 - GPT1_CNT;
        assert(delta < 0x80000000u); // every compare must be in the future
        const uint64_t next = fake_tick + delta;
        if (next > end) break;
        set_tick(next);
        channels::gptIsr(SS_GPT_SR_OF1);
        assert(++calls < 10000);
    }
}
int main() {
    modes[0] = 99; modes[1] = 99; // Serial1 already configured before channels::begin
    channels::begin();
    assert(modes[0] == 99 && modes[1] == 99);
    assert(STROBE_CFG[2].pin == 14 && STROBE_CFG[3].pin == 15);
    assert(channels::freqHz(0) == 50 && channels::freqHz(1) == 0);
    assert(channels::setFreqHz(1, 25));
    assert(channels::setFreqHz(2, 5));
    assert(channels::setFreqHz(3, 7));
    assert(!channels::setFreqHz(0, std::numeric_limits<double>::quiet_NaN()));
    assert(!channels::setFreqHz(0, std::numeric_limits<double>::infinity()));
    assert(!channels::setFreqHz(0, -1));
    assert(channels::freqHz(0) == 50);
    // Four independent rates, including simultaneous starts, on both sides of wrap.
    for (const uint64_t begin : {uint64_t(100000), uint64_t(0x100000000ULL - 10000)}) {
        restart(begin);
        channels::setEnabled(true);
        run_until(begin + hz);
        const unsigned counts[] = {100, 50, 10, 14};
        uint64_t starts[TRIG_MAX] = {}, last_tick = 0;
        unsigned seen[TRIG_MAX] = {};
        uint8_t ch, level; uint64_t tick; uint32_t seq;
        size_t physical = 0;
        while (channels::popTrig(ch, level, tick, seq)) {
            assert(seq == ++seen[ch]);
            assert(tick >= last_tick); last_tick = tick;
            const auto &edge = pin_edges.at(physical++);
            assert(edge.pin == TRIG_CFG[ch].pin && edge.level == level);
            assert(tick >= edge.tick && tick - edge.tick <= now_step);
            if (level) starts[ch] = tick;
            else assert(tick - starts[ch] >= 3750);
        }
        assert(physical == pin_edges.size());
        for (unsigned i = 0; i < TRIG_MAX; ++i) {
            assert(seen[i] == counts[i]);
            assert(channels::skippedCount(i) == 0 && channels::stalledCount(i) == 0);
        }
    }
    // An ISR after the whole pulse window skips the pulse, never emits a runt.
    restart(100000);
    channels::setEnabled(true);
    set_tick(fake_tick + uint32_t(GPT1_OCR1 - GPT1_CNT) + 10000);
    channels::gptIsr(SS_GPT_SR_OF1);
    assert(pin_edges.empty());
    for (unsigned i = 0; i < TRIG_MAX; ++i) assert(channels::skippedCount(i) == 1);
    // A late falling-edge ISR deasserts and logs its actual software sample time.
    restart(100000);
    channels::setEnabled(true);
    run_until(102000);
    for (unsigned i = 0; i < TRIG_MAX; ++i) assert(levels[TRIG_CFG[i].pin] == HIGH);
    set_tick(fake_tick + 10000);
    channels::gptIsr(SS_GPT_SR_OF1);
    for (unsigned i = 0; i < TRIG_MAX; ++i) assert(levels[TRIG_CFG[i].pin] == LOW);
    // Existing strobe callbacks still map each physical connector to its channel.
    for (unsigned i = 0; i < STROBE_MAX; ++i) {
        levels[STROBE_CFG[i].pin] = HIGH;
        callbacks[STROBE_CFG[i].pin]();
        uint8_t ch, level; uint64_t tick; uint32_t seq;
        assert(channels::popStrobe(ch, level, tick, seq));
        assert(ch == i && level == HIGH && seq == 1);
    }
    // Total overload is bounded and leaves every output idle.
    restart(100000);
    now_step = hz;
    channels::setEnabled(true);
    assert(!(GPT1_IR & SS_GPT_IR_OF1IE));
    for (unsigned i = 0; i < TRIG_MAX; ++i) {
        assert(channels::stalledCount(i) == 1);
        assert(levels[TRIG_CFG[i].pin] == LOW);
    }
    restart(100000);
    for (unsigned i = 0; i < TRIG_MAX; ++i) assert(channels::setFreqHz(i, 0));
    channels::setEnabled(true);
    assert(!(GPT1_IR & SS_GPT_IR_OF1IE));
    puts("channel scheduler regressions passed");
}
