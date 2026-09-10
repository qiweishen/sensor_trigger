// Execute the actual LED implementation against counters and a clock/GPIO model.
#include "channels.h"
#include "leds.h"
#include "timesync.h"
#include <cassert>

static uint32_t pps = 0, rx_bytes = 0;
static uint32_t trig[TRIG_MAX] = {}, strobe[STROBE_MAX] = {};
namespace channels {
    uint32_t trigCount(uint8_t ch) { return ch < TRIG_MAX ? trig[ch] : 0; }
    uint32_t strobeCount(uint8_t ch) { return ch < STROBE_MAX ? strobe[ch] : 0; }
}
namespace timesync {
    uint32_t ppsRawCount() { return pps; }
    uint32_t todBytes() { return rx_bytes; }
}
static void all_off() {
    for (uint8_t pin : LED_PINS) assert(levels[pin] == LOW);
}
static void restart(uint32_t ms = 0) {
    pps = rx_bytes = 0;
    for (auto &n : trig) n = 0;
    for (auto &n : strobe) n = 0;
    fake_ms = ms;
    leds::reset();
    leds::poll();
    all_off();
}
int main() {
    // Initialize every pin high to catch missed outputs and unrelated GPIO writes.
    for (unsigned pin = 0; pin < CORE_NUM_DIGITAL; ++pin) {
        levels[pin] = HIGH;
        modes[pin] = 99;
    }
    leds::begin();
    const uint8_t schematic_pins[] = {2, 3, 6, 5, 8, 7}; // D211..D216
    for (unsigned pin = 0; pin < CORE_NUM_DIGITAL; ++pin) {
        bool is_led = false;
        for (uint8_t led : schematic_pins) is_led |= pin == led;
        assert(modes[pin] == (is_led ? OUTPUT : 99));
        assert(levels[pin] == (is_led ? LOW : HIGH));
    }
    // Static HIGH PPS/exposure inputs, no captured edges: no false activity.
    leds::poll();
    all_off();

    // Each source, including the second camera in each group, reaches only its LED.
    struct Source { uint32_t *count; uint8_t pin; };
    const Source sources[] = {
        {&pps, 2}, {&rx_bytes, 3},
        {&trig[0], 6}, {&trig[1], 6}, {&strobe[0], 5}, {&strobe[1], 5},
        {&trig[2], 8}, {&trig[3], 8}, {&strobe[2], 7}, {&strobe[3], 7}
    };
    for (const auto &source : sources) {
        restart();
        ++*source.count;
        leds::poll(); // at millis == 0; zero must not act as an unset sentinel
        for (uint8_t pin : LED_PINS) assert(levels[pin] == (pin == source.pin ? HIGH : LOW));
        fake_ms = LED_HOLD_MS - 1;
        leds::poll();
        assert(levels[source.pin] == HIGH);
        fake_ms = LED_HOLD_MS;
        leds::poll();
        all_off();
    }

    // A complete, microsecond-scale pulse between polls still counts as activity.
    restart();
    trig[0] += 2;
    strobe[0] += 2;
    leds::poll();
    assert(levels[6] == HIGH && levels[5] == HIGH);
    fake_ms = LED_HOLD_MS - 1;
    ++trig[1]; // second FX camera extends the shared trigger LED only
    leds::poll();
    fake_ms = LED_HOLD_MS;
    leds::poll();
    assert(levels[6] == HIGH && levels[5] == LOW);
    fake_ms = 2 * LED_HOLD_MS - 1;
    leds::poll();
    all_off();

    // Concurrent activity and STOP: extinguish immediately, then start from zero.
    for (const auto &source : sources) ++*source.count;
    leds::poll();
    for (uint8_t pin : LED_PINS) assert(levels[pin] == HIGH);
    leds::reset();
    all_off();
    restart();
    ++pps;
    leds::poll();
    assert(levels[2] == HIGH);
    for (uint8_t pin : LED_PINS) if (pin != 2) assert(levels[pin] == LOW);

    // Activity remains visible when a 32-bit counter (including a group sum) wraps.
    restart(1000);
    pps = UINT32_MAX;
    trig[0] = UINT32_MAX;
    leds::poll();
    fake_ms += LED_HOLD_MS;
    leds::poll();
    all_off();
    ++pps;
    ++trig[1];
    leds::poll();
    assert(levels[2] == HIGH && levels[6] == HIGH);

    // millis wrap neither shortens nor sticks a pulse on.
    restart(UINT32_MAX - LED_HOLD_MS / 2);
    const uint32_t start_ms = fake_ms;
    ++rx_bytes;
    leds::poll();
    fake_ms = start_ms + LED_HOLD_MS - 1;
    leds::poll();
    assert(levels[3] == HIGH);
    fake_ms = start_ms + LED_HOLD_MS;
    leds::poll();
    all_off();
    puts("LED activity regressions passed");
}
