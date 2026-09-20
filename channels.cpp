#include <stdint.h>
// Teensy supplies its own startup; only use the CMSIS CPU intrinsics here.
#define __PROGRAM_START
#include "cmsis_gcc.h"
#undef __PROGRAM_START
#include <cmath>

#include "channels.h"
#include "timebase.h"

namespace channels {
	namespace {
		constexpr uint16_t kModules = (1u << 0) | (1u << 3);
		constexpr uint16_t kOutputs = FLEXPWM_OUTEN_PWMA_EN(kModules) | FLEXPWM_OUTEN_PWMB_EN(kModules);
		constexpr uint16_t kRise = FLEXPWM_SMSTS_CMPF(1u << 2);
		constexpr uint16_t kFall = FLEXPWM_SMSTS_CMPF(1u << 3);
		constexpr uint16_t kEdges = kRise | kFall;
		constexpr uint8_t kSubmodule[] = { 0, 3 };
		static_assert(TRIG_MAX == 4 && STROBE_MAX == 4, "four wired cameras required");
		static_assert(TRIG_CFG[0].pin == 33 && TRIG_CFG[1].pin == 4 && TRIG_CFG[2].pin == 37 && TRIG_CFG[3].pin == 36,
					  "Teensy 4.1 FlexPWM2 pin mapping");
		static_assert(TRIG_CFG[0].active_high && TRIG_CFG[1].active_high && TRIG_CFG[2].active_high && TRIG_CFG[3].active_high,
					  "PCB uses active-high triggers");
		static_assert(
				[] {
					if (PPS_PIN >= CORE_NUM_DIGITAL || PPS_PIN == 0 || PPS_PIN == 1)
						return false;
					for (const auto &c: TRIG_CFG)
						if (c.pin == PPS_PIN)
							return false;
					for (uint8_t i = 0; i < STROBE_MAX; ++i) {
						const auto pin = STROBE_CFG[i].pin;
						if (pin >= CORE_NUM_DIGITAL || pin == 0 || pin == 1 || pin == PPS_PIN)
							return false;
						for (const auto &c: TRIG_CFG)
							if (c.pin == pin)
								return false;
						for (uint8_t j = 0; j < i; ++j)
							if (STROBE_CFG[j].pin == pin)
								return false;
					}
					return true;
				}(),
				"PPS and strobes must not overlap triggers, each other, or Serial1");
		static_assert(PWM_IRQ_PRIORITY > TB_IRQ_PRIORITY && PWM_IRQ_PRIORITY < STROBE_IRQ_PRIORITY,
					  "PWM logging must not preempt GPT1 and must outrank GPIO");
		struct IrqGuard {
			uint32_t saved = __get_PRIMASK();
			IrqGuard() { __disable_irq(); }
			~IrqGuard() { __set_PRIMASK(saved); }
		};
		struct State {
			uint64_t last_sample = 0, period_gpt = 0, stop_tick = 0;
			uint32_t errors = 0, cut_seq = 0;
			bool high = false;
		};
		trigger::Rates g_rates = TRIGGER_DEFAULT_RATES;
		trigger::Timing g_timing[2];
		State g_state[2];
		volatile bool g_enabled = false, g_fault = false;
		volatile bool g_strobe_enabled = false;
		volatile uint8_t g_strobe_active = 0;
		bool g_ready = false;
	}  // namespace

	// PWM IRQs have equal priority and cannot nest; STOP invokes the same handler
	// with IRQs masked. Queue writes are therefore serialized. GPIO owns Strobe.
	struct Evt {
		uint8_t ch;
		uint8_t level;
		uint64_t tick;
		uint32_t seq;
	};

	static volatile Evt g_trigq[EVT_RING_SIZE];
	static volatile uint16_t g_trigq_head = 0, g_trigq_tail = 0;
	static volatile uint32_t g_trig_seq[TRIG_MAX] = { 0 };
	static volatile uint32_t g_trig_drops = 0;

	static volatile Evt g_strobeq[EVT_RING_SIZE];
	static volatile uint16_t g_strobeq_head = 0, g_strobeq_tail = 0;
	static volatile uint32_t g_strobe_seq[STROBE_MAX] = { 0 };
	static volatile uint32_t g_strobe_drops = 0;


	static void pushTrig(const uint8_t ch, const uint8_t level, const uint64_t tick) {
		const uint32_t s = ++g_trig_seq[ch];
		const uint16_t h = g_trigq_head;
		const auto nh = static_cast<uint16_t>((h + 1) % EVT_RING_SIZE);
		if (nh == g_trigq_tail) {
			g_trig_drops++;	 // seq s is spent but not stored -> gap in this channel's stream
			return;
		}
		g_trigq[h].ch = ch;
		g_trigq[h].level = level;
		g_trigq[h].tick = tick;
		g_trigq[h].seq = s;
		__DMB();
		g_trigq_head = nh;
	}


	static void pushStrobe(const uint8_t ch, const uint8_t level, const uint64_t tick) {
		if (!g_strobe_enabled) {
			return;	 // idle between sessions: don't count or queue strobe edges
		}
		const uint8_t bit = static_cast<uint8_t>(1u << ch);
		if (bool(level) == STROBE_CFG[ch].active_high)
			g_strobe_active |= bit;
		else
			g_strobe_active &= static_cast<uint8_t>(~bit);
		const uint32_t s = ++g_strobe_seq[ch];
		const uint16_t h = g_strobeq_head;
		const auto nh = static_cast<uint16_t>((h + 1) % EVT_RING_SIZE);
		if (nh == g_strobeq_tail) {
			g_strobe_drops++;
			return;
		}
		g_strobeq[h].ch = ch;
		g_strobeq[h].level = level;
		g_strobeq[h].tick = tick;
		g_strobeq[h].seq = s;
		__DMB();
		g_strobeq_head = nh;
	}


	bool popTrig(uint8_t &ch, uint8_t &level, uint64_t &tick, uint32_t &seq) {
		if (g_trigq_tail == g_trigq_head) {
			return false;
		}
		__DMB();
		const uint16_t t = g_trigq_tail;
		ch = g_trigq[t].ch;
		level = g_trigq[t].level;
		tick = g_trigq[t].tick;
		seq = g_trigq[t].seq;
		__DMB();
		g_trigq_tail = static_cast<uint16_t>((t + 1) % EVT_RING_SIZE);
		return true;
	}


	bool popStrobe(uint8_t &ch, uint8_t &level, uint64_t &tick, uint32_t &seq) {
		if (g_strobeq_tail == g_strobeq_head) {
			return false;
		}
		__DMB();
		const uint16_t t = g_strobeq_tail;
		ch = g_strobeq[t].ch;
		level = g_strobeq[t].level;
		tick = g_strobeq[t].tick;
		seq = g_strobeq[t].seq;
		__DMB();
		g_strobeq_tail = static_cast<uint16_t>((t + 1) % EVT_RING_SIZE);
		return true;
	}


	namespace {
		void freezeCounters() {
			for (uint8_t sm: kSubmodule)
				IMXRT_FLEXPWM2.SM[sm].INTEN = 0;
			// RUN=0 resets counters, but holds PWM outputs (RM 55.8.48).
			IMXRT_FLEXPWM2.MCTRL &= ~FLEXPWM_MCTRL_RUN(kModules);
			__DSB();
		}
		void parkPins() {
			IMXRT_FLEXPWM2.MASK = IMXRT_FLEXPWM2.MASK | kOutputs | FLEXPWM_MASK_UPDATE_MASK(kModules);
			// MASK updates on a clock edge. GPIO takeover guarantees LOW even with the
			// counters stopped or a missing AUX clock. Preload LOW before changing mux.
			for (const auto &c: TRIG_CFG) {
				digitalWriteFast(c.pin, LOW);
				pinMode(c.pin, OUTPUT);
			}
			__DSB();
		}
		void fail(uint8_t group) {
			++g_state[group].errors;
			g_fault = true;
			freezeCounters();
			parkPins();
			g_enabled = false;
		}
		void pwmEvent(uint8_t group) {
			const uint64_t tick = timebase::now();
			auto &sm = IMXRT_FLEXPWM2.SM[kSubmodule[group]];
			const uint16_t status = sm.STS;
			const uint16_t edges = status & kEdges;
			sm.STS = status;  // W1C, never clear a later flag by read/modify/write
			if (!g_enabled || g_rates.hz[group] == 0 || (!edges && !(status & FLEXPWM_SMSTS_REF)))
				return;
			State &s = g_state[group];
			// Sticky flags cannot reconstruct merged events. Never fabricate their times.
			if ((status & FLEXPWM_SMSTS_REF) || edges != (s.high ? kFall : kRise) || tick - s.last_sample >= s.period_gpt) {
				fail(group);
				return;
			}
			s.high = edges == kRise;
			s.last_sample = tick;
			for (uint8_t ch = 2 * group; ch < 2 * group + 2; ++ch)
				pushTrig(ch, s.high ? HIGH : LOW, tick);
			if (g_trig_drops)
				fail(group);
		}
		void pwm0Isr() {
			pwmEvent(0);
			__DSB();
		}
		void pwm3Isr() {
			pwmEvent(1);
			__DSB();
		}

		template<uint8_t ch>
		void strobeIsr() {
			const uint64_t t = timebase::now();
			pushStrobe(ch, static_cast<uint8_t>(digitalReadFast(STROBE_CFG[ch].pin)), t);
		}
		void program(uint8_t group) {
			const auto &t = g_timing[group];
			auto &sm = IMXRT_FLEXPWM2.SM[kSubmodule[group]];
			// Local sync/reload, independent A/B, no complementary polarity or deadtime.
			sm.CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN | FLEXPWM_SMCTRL2_FRCEN |
					   FLEXPWM_SMCTRL2_CLK_SEL(group ? 2 : 0);
			sm.CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_LDMOD | FLEXPWM_SMCTRL_PRSC(t.prescale);
			sm.INIT = 0;
			sm.VAL0 = 0;
			sm.VAL1 = t.period - 1;
			sm.VAL2 = sm.VAL4 = t.width;  // first pulse starts after a defined LOW interval
			sm.VAL3 = sm.VAL5 = 2 * t.width;
			sm.FRCTRL = 0;
			sm.FRACVAL1 = sm.FRACVAL2 = sm.FRACVAL3 = sm.FRACVAL4 = sm.FRACVAL5 = 0;
			sm.OCTRL = 0;
			sm.DTCNT0 = sm.DTCNT1 = 0;
			sm.DISMAP0 = sm.DISMAP1 = 0;  // no wired fault inputs in this board
			sm.DMAEN = sm.TCTRL = 0;
			sm.CAPTCTRLA = sm.CAPTCTRLB = sm.CAPTCTRLX = 0;
			sm.INTEN = 0;
		}
	}  // namespace

	bool validateRates(const trigger::Rates &rates) {
		// The approved clock tree and GPT conversion assume this project clock.
		if (F_BUS_ACTUAL != 150000000u)
			return false;
		trigger::Timing next;
		for (uint8_t g = 0; g < 2; ++g)
			if (!trigger::calculate(static_cast<trigger::Group>(g), rates.hz[g], GROUP_PULSE_US[g], F_BUS_ACTUAL, next))
				return false;
		return true;
	}
	bool applyRates(const trigger::Rates &rates) {
		if (g_enabled || !validateRates(rates))
			return false;
		trigger::Timing next[2];
		for (uint8_t g = 0; g < 2; ++g)
			trigger::calculate(static_cast<trigger::Group>(g), rates.hz[g], GROUP_PULSE_US[g], F_BUS_ACTUAL, next[g]);
		g_rates = rates;
		for (uint8_t g = 0; g < 2; ++g)
			g_timing[g] = next[g];
		g_ready = true;
		return true;
	}
	trigger::Rates requestedRates() {
		return g_rates;
	}
	const trigger::Timing &timing(uint8_t group) {
		return g_timing[group < 2 ? group : 0];
	}
	double freqHz(uint8_t ch) {
		return ch < TRIG_MAX ? g_timing[ch / 2].actual_hz : 0;
	}

	bool setEnabled(bool on, bool retain_strobes) {
		IrqGuard lock;
		if (!on) {
			const bool was_enabled = g_enabled;
			freezeCounters();
			// No further PWM events can arrive. Drain final pending flags once; these
			// remain handler samples, including when the handler is called by STOP.
			if (g_enabled)
				for (uint8_t g = 0; g < 2; ++g)
					pwmEvent(g);
			parkPins();
			const uint64_t tick = timebase::now();
			if (was_enabled && !g_fault)
				for (uint8_t g = 0; g < 2; ++g) {
					if (g_state[g].high) {
						g_state[g].cut_seq = g_trig_seq[2 * g];
						g_state[g].stop_tick = tick;
					}
				}
			g_enabled = false;
			if (!retain_strobes)
				g_strobe_enabled = false;
			return true;
		}
		if (g_enabled)
			return true;
		if (!g_ready || g_fault || !validateRates(g_rates))
			return false;
		freezeCounters();
		parkPins();
		IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_CLDOK(kModules);
		for (uint8_t g = 0; g < 2; ++g)
			program(g);
		// Preserve other submodules' output selectors; select generated PWM for ours.
		IMXRT_FLEXPWM2.DTSRCSEL &= ~uint16_t(0xF00F);
		IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_LDOK(kModules);
		for (uint8_t g = 0; g < 2; ++g) {
			auto &sm = IMXRT_FLEXPWM2.SM[kSubmodule[g]];
			sm.CTRL2 |= FLEXPWM_SMCTRL2_FORCE;	// initialize CNT and PWM latches LOW
			sm.STS = 0xFFFF;
			sm.INTEN = g_rates.hz[g] > 0 ? FLEXPWM_SMINTEN_CMPIE((1u << 2) | (1u << 3)) | FLEXPWM_SMINTEN_REIE : 0;
			g_state[g].high = false;
			g_state[g].period_gpt =
					static_cast<uint64_t>(std::llround(g_timing[g].period * timebase::nominalHz() / g_timing[g].clock_hz));
		}
		NVIC_CLEAR_PENDING(IRQ_FLEXPWM2_0);
		NVIC_CLEAR_PENDING(IRQ_FLEXPWM2_3);
		// Exact muxes for Teensy 4.1. Disabled pairs stay GPIO LOW even when their
		// PWM clock is stopped; never expose a held PWM latch on a disabled pin.
		if (g_rates.hz[0] > 0)
			*portConfigRegister(33) = *portConfigRegister(4) = 1;
		if (g_rates.hz[1] > 0)
			*portConfigRegister(37) = *portConfigRegister(36) = 6;
		IMXRT_FLEXPWM2.OUTEN |= kOutputs;
		uint16_t active = 0;
		for (uint8_t g = 0; g < 2; ++g)
			if (g_rates.hz[g] > 0)
				active |= 1u << kSubmodule[g];
		const uint16_t mask = FLEXPWM_MASK_MASKA(active) | FLEXPWM_MASK_MASKB(active);
		IMXRT_FLEXPWM2.MASK = (IMXRT_FLEXPWM2.MASK & ~mask) | FLEXPWM_MASK_UPDATE_MASK(kModules);
		const uint64_t start = timebase::now();
		for (auto &s: g_state)
			s.last_sample = start;
		g_enabled = true;
		g_strobe_enabled = true;
		// JAI alone still needs SM0 RUN to produce AUX_CLK. Both off may still log inputs.
		const uint16_t run = active ? (active | 1u) : 0;
		IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_RUN(run);
		__DSB();
		return true;
	}

	void begin() {
		CCM_CCGR4 |= CCM_CCGR4_PWM2(CCM_CCGR_ON);
		freezeCounters();
		parkPins();
		applyRates(TRIGGER_DEFAULT_RATES);
		attachInterruptVector(IRQ_FLEXPWM2_0, pwm0Isr);
		attachInterruptVector(IRQ_FLEXPWM2_3, pwm3Isr);
		NVIC_SET_PRIORITY(IRQ_FLEXPWM2_0, PWM_IRQ_PRIORITY);
		NVIC_SET_PRIORITY(IRQ_FLEXPWM2_3, PWM_IRQ_PRIORITY);
		NVIC_ENABLE_IRQ(IRQ_FLEXPWM2_0);
		NVIC_ENABLE_IRQ(IRQ_FLEXPWM2_3);
		void (*const callbacks[])() = { strobeIsr<0>, strobeIsr<1>, strobeIsr<2>, strobeIsr<3> };
		for (uint8_t ch = 0; ch < STROBE_MAX; ++ch) {
			pinMode(STROBE_CFG[ch].pin, STROBE_CFG[ch].active_high ? INPUT_PULLDOWN : INPUT_PULLUP);
			attachInterrupt(digitalPinToInterrupt(STROBE_CFG[ch].pin), callbacks[ch], CHANGE);
		}
		NVIC_SET_PRIORITY(IRQ_GPIO6789, STROBE_IRQ_PRIORITY);
	}
	void pollHealth() {
		IrqGuard lock;
		if (!g_enabled)
			return;
		const uint64_t tick = timebase::now();
		for (uint8_t g = 0; g < 2; ++g) {
			if (g_rates.hz[g] > 0 &&
				tick - g_state[g].last_sample > g_state[g].period_gpt + static_cast<uint64_t>(timebase::nominalHz() / 100)) {
				fail(g);
				return;
			}
		}
	}
	void resetCounts() {
		IrqGuard lock;
		if (g_enabled || g_strobe_enabled)
			return;
		for (auto &s: g_state)
			s = {};
		for (auto &s: g_trig_seq)
			s = 0;
		for (auto &s: g_strobe_seq)
			s = 0;
		g_trig_drops = g_strobe_drops = 0;
		g_trigq_head = g_trigq_tail = g_strobeq_head = g_strobeq_tail = 0;
		g_fault = false;
		g_strobe_active = 0;
	}
	bool enabled() {
		return g_enabled;
	}
	uint8_t pendingStrobes() {
		IrqGuard lock;
		return g_strobe_active;
	}
	bool faulted() {
		return g_fault;
	}
	uint32_t trigCount(uint8_t ch) {
		return ch < TRIG_MAX ? g_trig_seq[ch] : 0;
	}
	uint32_t strobeCount(uint8_t ch) {
		return ch < STROBE_MAX ? g_strobe_seq[ch] : 0;
	}
	uint32_t pwmErrors(uint8_t group) {
		IrqGuard lock;
		return group < 2 ? g_state[group].errors : 0;
	}
	uint32_t trigDrops() {
		return g_trig_drops;
	}
	uint32_t strobeDrops() {
		return g_strobe_drops;
	}
	bool truncation(uint8_t group, uint32_t &rise_seq, uint64_t &stop_tick) {
		if (group >= 2 || !g_state[group].cut_seq)
			return false;
		rise_seq = g_state[group].cut_seq;
		stop_tick = g_state[group].stop_tick;
		return true;
	}
}  // namespace channels
