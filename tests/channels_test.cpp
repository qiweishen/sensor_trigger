// Run the production driver against an event-driven model of clocked PWM
// compares. Hardware edges continue when interrupts are withheld.
#include "channels.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <tuple>

#include "timebase.h"

namespace timebase {
	double nominalHz() {
		return 75000000.0;
	}
	uint64_t now() {
		set_tick(fake_tick + now_step);
		return fake_tick;
	}
}  // namespace timebase
constexpr uint64_t hz = 75000000;
static uint64_t next_event(unsigned g) {
	const unsigned sm = g ? 3 : 0;
	const auto &r = IMXRT_FLEXPWM2.SM[sm];
	const uint16_t run = IMXRT_FLEXPWM2.MCTRL;
	if (!(run & (0x100u << sm)))
		return UINT64_MAX;
	uint64_t divider = 1u << ((r.CTRL >> 4) & 7);
	const unsigned source = r.CTRL2 & 3;
	if (source == 2) {
		if (!(run & 0x100))
			return UINT64_MAX;
		divider *= 1u << ((IMXRT_FLEXPWM2.SM[0].CTRL >> 4) & 7);
	} else
		assert(source == 0);
	// All configurations used here have an integer GPT ticks / PWM tick ratio.
	assert(divider % 2 == 0);
	const uint64_t count = (r.next / 2) * (uint64_t(r.VAL1) + 1) + (r.next % 2 ? r.VAL3 : r.VAL2);
	return r.origin + count * (divider / 2);
}
static void dispatch() {
	if (fake_primask)
		return;
	for (unsigned g = 0; g < 2; ++g) {
		const auto &r = IMXRT_FLEXPWM2.SM[g ? 3 : 0];
		if (irq_enabled[g + 1] && (r.STS & r.INTEN))
			irq_vectors[g + 1]();
	}
}
static void advance(uint64_t end, bool interrupts = true) {
	for (;;) {
		uint64_t next[2] = { next_event(0), next_event(1) };
		const uint64_t tick = std::min(next[0], next[1]);
		if (tick > end || tick == UINT64_MAX)
			break;
		set_tick(std::max(tick, fake_tick));
		for (unsigned g = 0; g < 2; ++g)
			if (next[g] == tick) {
				const unsigned sm = g ? 3 : 0;
				auto &r = IMXRT_FLEXPWM2.SM[sm];
				const bool high = (r.next++ % 2 == 0);
				r.CNT = high ? r.VAL2 : r.VAL3;
				assert(r.VAL2 == r.VAL4 && r.VAL3 == r.VAL5);
				assert((r.CTRL2 & ((3u << 8) | (1u << 2))) == 0);  // local init/reload
				r.STS.raise(high ? (1u << 2) | (1u << 4) : (1u << 3) | (1u << 5));
				for (unsigned c = 0; c < 2; ++c) {
					const unsigned pin = TRIG_CFG[2 * g + c].pin;
					const unsigned mask = 1u << (sm + (c == 0 ? 4 : 8));
					if (muxes[pin] != (g ? 6u : 1u) || !(IMXRT_FLEXPWM2.OUTEN & mask))
						continue;
					const bool output = high && !(IMXRT_FLEXPWM2.MASK & mask);
					if (levels[pin] != output)
						pin_edges.push_back({ uint8_t(pin), uint8_t(output), tick });
					levels[pin] = output;
				}
			}
		if (interrupts)
			dispatch();
	}
	set_tick(std::max(end, fake_tick));
}
static void restart(trigger::Rates rates, uint64_t start = 100000) {
	channels::setEnabled(false);
	channels::resetCounts();
	set_tick(start);
	now_step = 25;
	pin_edges.clear();
	assert(channels::applyRates(rates));
	assert(channels::setEnabled(true));
}
static void drain() {
	uint8_t ch, level;
	uint64_t tick;
	uint32_t seq;
	while (channels::popTrig(ch, level, tick, seq)) {
	}
}
static void all_low() {
	for (const auto &c: TRIG_CFG)
		assert(levels[c.pin] == LOW && muxes[c.pin] == 5);
}
int main() {
	modes[0] = modes[1] = 99;
	IMXRT_FLEXPWM2.SM[1].CTRL = 1234;
	IMXRT_FLEXPWM2.MCTRL = 0x600;  // unrelated SM1/2 already running
	channels::begin();
	assert(modes[0] == 99 && modes[1] == 99);
	assert(IMXRT_FLEXPWM2.SM[1].CTRL == 1234 && (IMXRT_FLEXPWM2.MCTRL & 0x600) == 0x600);
	all_low();
	assert(channels::freqHz(0) == channels::freqHz(1));
	assert(channels::freqHz(2) == 0 && channels::freqHz(3) == 0);
	assert(irq_priority[1] == 48 && irq_priority[2] == 48);

	trigger::Rates rates = { { 25, 5 } };
	for (const char *bad: { "", "0:25", "FX:25,JAI:11", "JAI:0.5", "FX:19", "FX:nan", "JAI:inf", "FX:25,", ",JAI:1", "FX:25,,JAI:1",
							"FX:25,FX:50", "JAI:1,FX:-1" }) {
		assert(!trigger::parseRates(bad, rates));
		assert(rates.hz[0] == 25 && rates.hz[1] == 5);
	}
	assert(trigger::parseRates("JAI:1", rates) && rates.hz[0] == 25 && rates.hz[1] == 1);
	assert(trigger::parseRates(nullptr, rates));
	assert(!channels::applyRates({ { 25, 0.1 } }));
	assert(!channels::applyRates({ { std::numeric_limits<double>::infinity(), 1 } }));
	assert(!channels::applyRates({ { 1e300, 1 } }));
	assert(!channels::applyRates({ { 25, 11 } }));
	assert(channels::requestedRates().hz[0] == 50 && channels::requestedRates().hz[1] == 0);

	for (double fx: { 20., 25., 50. })
		for (double jai: { 1., 2., 5., 10. }) {
			restart({ { fx, jai } }, 0x100000000ULL - 10000);
			assert(!channels::applyRates({ { 100, 5 } }));	// running update is rejected
			const auto &t = channels::timing(1);
			const unsigned d = jai == 1 ? 32 : jai == 2 ? 16 : jai == 5 ? 4 : 2;
			assert((1u << t.prescale) == d);
			assert(t.period == (jai <= 2 ? 36622 : 58594));
			assert(t.pulse_us >= 50 && t.pulse_us < 55);
			const auto &sm0 = IMXRT_FLEXPWM2.SM[0];
			assert(((sm0.CTRL >> 4) & 7) == 7);
			const uint64_t end = fake_tick + 2 * hz;
			advance(end);
			assert(!channels::faulted());
			uint8_t ch, level;
			uint64_t tick;
			uint32_t seq;
			uint32_t seen[4] = {};
			uint64_t pair_tick = 0, prev = 0;
			while (channels::popTrig(ch, level, tick, seq)) {
				assert(seq == ++seen[ch] && tick >= prev);
				prev = tick;
				if (ch % 2 == 0)
					pair_tick = tick;
				else
					assert(tick == pair_tick);
				assert(level == (seq % 2));
			}
			for (unsigned c = 0; c < 4; ++c)
				assert(seen[c] == unsigned(4 * (c < 2 ? fx : jai)));
			// Independent physical measurement: common edges and fixed pulse width.
			for (unsigned c = 0; c < 4; ++c) {
				uint64_t rise = 0, prev_rise = 0;
				const auto &tm = channels::timing(c / 2);
				const uint64_t quantum = c < 2 ? 64 : 64 * d;
				for (const auto &edge: pin_edges)
					if (edge.pin == TRIG_CFG[c].pin) {
						if (edge.level) {
							if (prev_rise)
								assert(edge.tick - prev_rise == uint64_t(tm.period) * quantum);
							rise = prev_rise = edge.tick;
						} else
							assert(edge.tick - rise == uint64_t(tm.width) * quantum);
					}
			}
			channels::setEnabled(false);
			all_low();
		}
	// Test both sides of every JAI divider transition, not just integer rates.
	for (double jai: { 1.1, 1.2, 2.2, 2.3, 4.4, 4.5, 8.9, 9.0 }) {
		restart({ { 25, jai } });
		const auto &t = channels::timing(1);
		assert(t.actual_hz <= jai && t.pulse_us >= 50);
		assert(t.prescale == 0 || std::ceil(t.clock_hz * 2 / jai) > 65535);
		advance(fake_tick + 2 * hz);
		assert(!channels::faulted());
		drain();
	}
	// JAI alone still gets SM0 AUX_CLK; no FX physical outputs or IRQs.
	restart({ { 0, 5 } });
	assert((IMXRT_FLEXPWM2.MCTRL & 0x900) == 0x900 && IMXRT_FLEXPWM2.SM[0].INTEN == 0);
	advance(fake_tick + hz);
	assert(channels::trigCount(0) == 0 && channels::trigCount(2) == 10);
	for (const auto &edge: pin_edges)
		assert(edge.pin == 36 || edge.pin == 37);
	drain();
	restart({ { 25, 0 } });
	advance(fake_tick + hz);
	assert(channels::trigCount(0) == 50 && channels::trigCount(2) == 0);
	drain();

	// Short ISR latency changes log times, never the physical pulse duration.
	restart({ { 25, 5 } });
	const uint64_t first = next_event(0);
	advance(first + 750, false);  // 10 us late, still before 50 us fall
	dispatch();
	assert(!channels::faulted());
	advance(first + 10000);
	assert(pin_edges[0].tick == first && pin_edges[0].level == HIGH);
	drain();

	// Both edges happened while masked: PWM completes its pulse in hardware,
	// logger rejects merged flags and parks ALL outputs without invented T edges.
	restart({ { 25, 5 } });
	advance(next_event(0) + 20000, false);
	assert(!pin_edges.empty());
	dispatch();
	assert(channels::faulted() && !channels::enabled() && channels::pwmErrors(0) == 1);
	assert(channels::trigCount(0) == 0);
	all_low();

	// Stop during HIGH explicitly marks the last rising sequence as truncated.
	restart({ { 25, 0 } });
	advance(next_event(0) + 100);
	channels::setEnabled(false);
	uint32_t seq;
	uint64_t tick;
	assert(channels::truncation(0, seq, tick) && seq == 1);
	assert(!channels::truncation(1, seq, tick));
	all_low();
	assert(channels::trigCount(0) == 1);  // no synthetic falling timestamp
	restart({ { 25, 0 } });
	assert(!channels::truncation(0, seq, tick));
	assert(pin_edges.empty());			  // restart exposes no stale HIGH latch

	// A lost timer clock is visible even if there is no interrupt to report it.
	IMXRT_FLEXPWM2.MCTRL &= ~uint16_t(0x900);
	set_tick(fake_tick + hz);
	channels::pollHealth();
	assert(channels::faulted());
	all_low();
	restart({ { 0, 0 } });
	advance(fake_tick + 2 * hz);
	channels::pollHealth();
	assert(!channels::faulted());
	for (unsigned c = 0; c < STROBE_MAX; ++c) {
		levels[STROBE_CFG[c].pin] = HIGH;
		callbacks[STROBE_CFG[c].pin]();
		uint8_t ch, level;
		uint64_t t;
		uint32_t n;
		assert(channels::popStrobe(ch, level, t, n) && ch == c && level == HIGH && n == 1);
	}
	// A full Trigger queue is an acquisition failure, not silent continuation.
	restart({ { 200, 0 } });
	advance(fake_tick + 12 * hz);  // >4095 queued edges per pair without draining
	assert(channels::trigDrops() > 0 && channels::faulted());
	all_low();
	// Clock changes invalidate the approved conversion and leave old config intact.
	channels::setEnabled(false);  // close retained exposure capture before reset
	channels::resetCounts();
	const auto old_rates = channels::requestedRates();
	F_BUS_ACTUAL = 132000000;
	assert(!channels::applyRates({ { 50, 1 } }));
	assert(channels::requestedRates().hz[0] == old_rates.hz[0]);
	F_BUS_ACTUAL = 150000000;
	// Critical sections restore their caller's mask, including already-masked STOP.
	__disable_irq();
	channels::setEnabled(false);
	assert(fake_primask == 1);
	__enable_irq();
	assert(IMXRT_FLEXPWM2.SM[1].CTRL == 1234 && (IMXRT_FLEXPWM2.MCTRL & 0x600) == 0x600);
	puts("hardware PWM group regressions passed");
}
