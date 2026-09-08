#include "channels.h"

#include <cmath>
#include <cstring>
#include <iterator>

#include "timebase.h"

namespace channels {

	// ---------------------------------------------------------------------------
	// One compare wakes the scheduler for the earliest edge of any wired channel.
	static_assert(TRIG_MAX > 0 && TRIG_MAX <= 4, "scheduler supports 1..4 trigger channels");
	static_assert([] {
		for (uint8_t i = 0; i < TRIG_MAX; ++i) {
			if (!TRIG_CFG[i].name || TRIG_CFG[i].pin >= CORE_NUM_DIGITAL ||
			    TRIG_CFG[i].pin == 0 || TRIG_CFG[i].pin == 1 || TRIG_CFG[i].pin == PPS_PIN) return false;
			for (uint8_t j = 0; j < i; ++j)
				if (TRIG_CFG[i].pin == TRIG_CFG[j].pin) return false;
			for (uint8_t j = 0; j < STROBE_MAX; ++j)
				if (TRIG_CFG[i].pin == STROBE_CFG[j].pin) return false;
		}
		return true;
	}(), "trigger pins must be unique and must not overlap Serial1, PPS or strobes");


	// ---------------------------------------------------------------------------
	// Trigger channel state
	struct TrigState {
		uint64_t period_ticks;
		uint64_t width_ticks;
		uint64_t phase_ticks;
		uint64_t rise_tick;		 // absolute tick of the current/next rising edge
		uint64_t fall_tick;       // sampled rise + pulse width
		volatile uint8_t stage;	 // 0 = rise scheduled, 1 = fall scheduled
		bool cfg_ok;			 // config validated in begin() (else channel disabled)
	};
	static TrigState g_trig[TRIG_MAX];
	static volatile bool g_enabled = false;
	static uint64_t g_margin_ticks = 0;
	static double g_freq_eff[TRIG_MAX] = { 0 };	 // effective rate (0 = disabled)


	// ---------------------------------------------------------------------------
	// Event rings.
	// Each has a SINGLE producer context: g_trigq from the GPT1 ISR, g_strobeq from the GPIO ISR - so both are plain SPSC (no locks).
	// Per-channel sequence numbers advance even on a drop, so a gap in the log is detectable.
	struct Evt {
		uint8_t ch;
		uint8_t level;
		uint64_t tick;
		uint32_t seq;
	};

	static volatile Evt g_trigq[EVT_RING_SIZE];
	static volatile uint16_t g_trigq_head = 0, g_trigq_tail = 0;
	static volatile uint32_t g_trig_seq[TRIG_MAX] = { 0 };
	static volatile uint32_t g_skipped[TRIG_MAX] = { 0 };
	static volatile uint32_t g_stalled[TRIG_MAX] = { 0 };  // scheduler overload; outputs parked until next START
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
		g_trigq_head = nh;
	}


	static void pushStrobe(const uint8_t ch, const uint8_t level, const uint64_t tick) {
		if (!g_enabled) {
			return;	 // idle between sessions: don't count or queue strobe edges
		}
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
		g_strobeq_head = nh;
	}


	bool popTrig(uint8_t &ch, uint8_t &level, uint64_t &tick, uint32_t &seq) {
		if (g_trigq_tail == g_trigq_head) {
			return false;
		}
		const uint16_t t = g_trigq_tail;
		ch = g_trigq[t].ch;
		level = g_trigq[t].level;
		tick = g_trigq[t].tick;
		seq = g_trigq[t].seq;
		g_trigq_tail = static_cast<uint16_t>((t + 1) % EVT_RING_SIZE);
		return true;
	}


	bool popStrobe(uint8_t &ch, uint8_t &level, uint64_t &tick, uint32_t &seq) {
		if (g_strobeq_tail == g_strobeq_head) {
			return false;
		}
		const uint16_t t = g_strobeq_tail;
		ch = g_strobeq[t].ch;
		level = g_strobeq[t].level;
		tick = g_strobeq[t].tick;
		seq = g_strobeq[t].seq;
		g_strobeq_tail = static_cast<uint16_t>((t + 1) % EVT_RING_SIZE);
		return true;
	}


	// ---------------------------------------------------------------------------
	// Trigger generation
	static inline uint8_t activeLevel(uint8_t i) {
		return TRIG_CFG[i].active_high ? HIGH : LOW;
	}
	static inline uint8_t idleLevel(uint8_t i) {
		return TRIG_CFG[i].active_high ? LOW : HIGH;
	}
	static void setIdle(uint8_t i) {
		digitalWriteFast(TRIG_CFG[i].pin, idleLevel(i));
	}


	// Catch up without compressing missed trigger periods into a burst.
	static void advanceRise(const uint8_t ch, const uint64_t deadline) {
		TrigState &s = g_trig[ch];
		if (s.rise_tick <= deadline) {
			const uint64_t n = (deadline - s.rise_tick) / s.period_ticks + 1;
			s.rise_tick += n * s.period_ticks;
			g_skipped[ch] += static_cast<uint32_t>(n);
		}
	}

	static uint64_t nextEdge(const TrigState &s) {
		return s.stage ? s.fall_tick : s.rise_tick;
	}

	static void serviceEdge(const uint8_t ch, const uint64_t now) {
		TrigState &s = g_trig[ch];
		if (!s.stage) {
			if (now >= s.rise_tick + s.width_ticks) {
				// The entire intended pulse window passed. Skip it rather than emit a runt.
				advanceRise(ch, now + g_margin_ticks);
				return;
			}
			digitalWriteFast(TRIG_CFG[ch].pin, activeLevel(ch));
			const uint64_t tick = timebase::now();
			pushTrig(ch, activeLevel(ch), tick);
			s.fall_tick = tick + s.width_ticks;
			s.stage = 1;
		} else {
			setIdle(ch);
			pushTrig(ch, idleLevel(ch), timebase::now());
			s.stage = 0;
			s.rise_tick += s.period_ticks;
			advanceRise(ch, timebase::now() + g_margin_ticks);
		}
	}

	static void scheduleNext() {
		// Bounded even for overload or a compare passed during the register write.
		for (unsigned attempt = 0; attempt < 2 * TRIG_MAX + 8; ++attempt) {
			uint8_t ch = TRIG_MAX;
			uint64_t tick = UINT64_MAX;
			for (uint8_t i = 0; i < TRIG_MAX; ++i) {
				if (g_trig[i].cfg_ok && nextEdge(g_trig[i]) < tick) {
					ch = i;
					tick = nextEdge(g_trig[i]);
				}
			}
			if (ch == TRIG_MAX) {
				GPT1_IR &= ~SS_GPT_IR_OF1IE;
				return;
			}
			const uint64_t now = timebase::now();
			if (tick <= now) {
				serviceEdge(ch, now);
				continue;
			}
			GPT1_OCR1 = static_cast<uint32_t>(tick);
			GPT1_SR = SS_GPT_SR_OF1;
			GPT1_IR |= SS_GPT_IR_OF1IE;
			// GPT compares equality. Never leave a passed compare armed for a full wrap.
			if (static_cast<int32_t>(GPT1_CNT - static_cast<uint32_t>(tick)) < 0) return;
		}
		// Overload: park every output and report the failure; a new START can recover.
		GPT1_IR &= ~SS_GPT_IR_OF1IE;
		for (uint8_t i = 0; i < TRIG_MAX; ++i) {
			if (!g_trig[i].cfg_ok) continue;
			if (g_trig[i].stage) {
				setIdle(i);
				pushTrig(i, idleLevel(i), timebase::now());
			}
			g_trig[i].stage = 0;
			++g_stalled[i];
		}
	}

	void gptIsr(const uint32_t sr) {
		if (g_enabled && (sr & SS_GPT_SR_OF1) && (GPT1_IR & SS_GPT_IR_OF1IE)) scheduleNext();
	}


	// ---------------------------------------------------------------------------
	// Strobe / Exposure-Active capture (both edges, raw)
	// Thin ISR: read the time before the level, or the GPIO read folds into the measurement.
	template<uint8_t N>
	static void strobeIsr() {
		constexpr uint8_t IDX = (N < STROBE_MAX) ? N : 0;
		const uint64_t t = timebase::now();
		pushStrobe(IDX, static_cast<uint8_t>(digitalReadFast(STROBE_CFG[IDX].pin)), t);
	}

	typedef void (*IsrFn)();
	static const IsrFn kStrobeIsr[] = {
		strobeIsr<0>, strobeIsr<1>, strobeIsr<2>, strobeIsr<3>, strobeIsr<4>, strobeIsr<5>, strobeIsr<6>, strobeIsr<7>,
	};
	static_assert(STROBE_MAX <= static_cast<int>(std::size(kStrobeIsr)), "STROBE_MAX exceeds the kStrobeIsr table");


	// ---------------------------------------------------------------------------
	void setEnabled(const bool on) {
		if (on == g_enabled) {
			return;
		}
		__disable_irq();
		if (on) {
			GPT1_SR = SS_GPT_SR_OF1;
			g_enabled = true;
			const uint64_t start = timebase::now() + g_margin_ticks * 4;
			for (uint8_t i = 0; i < TRIG_MAX; ++i) {
				g_trig[i].stage = 0;
				g_trig[i].rise_tick = start + g_trig[i].phase_ticks;
			}
			scheduleNext();
		} else {
			g_enabled = false;
			GPT1_IR &= ~SS_GPT_IR_OF1IE;
			for (uint8_t i = 0; i < TRIG_MAX; ++i) setIdle(i);
		}
		__enable_irq();
	}


	// Validate + program period/width/phase for channel i at `hz`. Warns loudly and
	// returns false on reject, leaving the old timing intact. hz == 0 = silent off.
	// Rules: freq >= 1 Hz (whole-tick period), period >= 4*margin (scheduler margin),
	// phase >= 0 (stagger, folded modulo the period).
	static bool configFreq(const uint8_t i, const double hz) {
		const double nom = timebase::nominalHz();
		const TriggerCfg &c = TRIG_CFG[i];
		if (hz == 0.0) {
			return false;  // configured off, not an error
		}
		if (!std::isfinite(hz) || hz < 1.0) {
			Serial.printf("# !! WARNING: trig[%u] %s freq=%.4fHz must be finite and >= 1 Hz; channel DISABLED\n", i, c.name, hz);
			return false;
		}
		const uint64_t min_period = g_margin_ticks * 4;
		const auto period = static_cast<uint64_t>(llround(nom / hz));
		if (period < min_period) {
			Serial.printf("# !! WARNING: trig[%u] %s freq=%.4fHz too fast (period < %llu ticks); channel DISABLED\n", i, c.name, hz,
						  static_cast<unsigned long long>(min_period));
			return false;
		}
		if (!std::isfinite(c.phase_s) || c.phase_s < 0.0) {
			Serial.printf("# !! WARNING: trig[%u] %s invalid phase_s=%.4f; channel DISABLED\n", i, c.name, c.phase_s);
			return false;
		}
		g_trig[i].period_ticks = period;
		g_trig[i].width_ticks = static_cast<uint64_t>(llround(c.pulse_us * 1e-6 * nom));
		if (g_trig[i].width_ticks < g_margin_ticks) {
			// must outlast the ISR path to the fall
			Serial.printf("# !! WARNING: trig[%u] %s pulse widened %lluus -> %.1fus (ISR margin)\n", i, c.name,
						  (unsigned long long) c.pulse_us, g_margin_ticks * 1e6 / nom);
			g_trig[i].width_ticks = g_margin_ticks;
		}
		if (g_trig[i].width_ticks >= period) {
			// keep the fall before the next rise
			Serial.printf("# !! WARNING: trig[%u] %s pulse %lluus >= period; clamped to period/2\n", i, c.name,
						  (unsigned long long) c.pulse_us);
			g_trig[i].width_ticks = period / 2;
		}
		// Fold keeps targets within the signed 32-bit compare horizon (rates >= 1 Hz).
		g_trig[i].phase_ticks = static_cast<uint64_t>(llround(std::fmod(c.phase_s, period / nom) * nom)) % period;
		return true;
	}


	// Runtime rate change (host START option). Only while disabled; the caller
	// re-enables and the new period takes effect from the session start.
	bool setFreqHz(const uint8_t ch, const double hz) {
		if (ch >= TRIG_MAX || g_enabled) {
			return false;
		}
		if (!std::isfinite(hz) || hz < 0.0) return false;
		if (hz == 0.0) {  // deliberate off
			g_trig[ch].cfg_ok = false;
			g_freq_eff[ch] = 0.0;
			return true;
		}
		if (!configFreq(ch, hz)) {
			return false;  // rejected: old rate kept
		}
		g_trig[ch].cfg_ok = true;
		g_freq_eff[ch] = timebase::nominalHz() / g_trig[ch].period_ticks;
		return true;
	}

	double freqHz(const uint8_t ch) {
		return ch < TRIG_MAX ? g_freq_eff[ch] : 0.0;
	}


	void begin() {
		const double nom = timebase::nominalHz();
		g_margin_ticks = static_cast<uint64_t>(nom * 5e-6);	 // 5 us
		if (g_margin_ticks == 0) g_margin_ticks = 1;

		for (uint8_t i = 0; i < TRIG_MAX; i++) {
			memset(&g_trig[i], 0, sizeof(TrigState));
			g_trig_seq[i] = 0;
			g_skipped[i] = 0;
			g_stalled[i] = 0;
			g_trig[i].cfg_ok = configFreq(i, TRIG_CFG[i].freq_hz);
			g_freq_eff[i] = g_trig[i].cfg_ok ? nom / g_trig[i].period_ticks : 0.0;
			pinMode(TRIG_CFG[i].pin, OUTPUT);
			setIdle(i);
		}

		for (uint8_t i = 0; i < STROBE_MAX; i++) {
			g_strobe_seq[i] = 0;
			// Bias bare-board inputs. The PCB buffer overrides this pull;
			// its external camera-side pull-up makes a disconnected camera read HIGH.
			pinMode(STROBE_CFG[i].pin, STROBE_CFG[i].active_high ? INPUT_PULLDOWN : INPUT_PULLUP);
		}
		for (uint8_t i = 0; i < STROBE_MAX; i++) {
			attachInterrupt(digitalPinToInterrupt(STROBE_CFG[i].pin), kStrobeIsr[i], CHANGE);
		}
		// PPS (software mode) shares this vector; keep >= 32 from GPT1 (see config.h).
		NVIC_SET_PRIORITY(IRQ_GPIO6789, STROBE_IRQ_PRIORITY);

		// Boot idle: no triggers, no strobe capture. A START command begins a session.
	}


	// Zero every per-channel counter and empty the event rings, so the next session counts from 0.
	// Call only while disabled (no ISR is pushing).
	void resetCounts() {
		__disable_irq();
		for (uint8_t i = 0; i < TRIG_MAX; i++) {
			g_trig_seq[i] = 0;
			g_skipped[i] = 0;
			g_stalled[i] = 0;
		}
		for (uint8_t i = 0; i < STROBE_MAX; i++) {
			g_strobe_seq[i] = 0;
		}
		g_trig_drops = 0;
		g_strobe_drops = 0;
		g_trigq_head = g_trigq_tail = 0;
		g_strobeq_head = g_strobeq_tail = 0;
		__enable_irq();
	}


	// ---------------------------------------------------------------------------
	bool enabled() {
		return g_enabled;
	}
	uint32_t trigCount(const uint8_t ch) {
		return ch < TRIG_MAX ? g_trig_seq[ch] : 0;
	}
	uint32_t strobeCount(const uint8_t ch) {
		return ch < STROBE_MAX ? g_strobe_seq[ch] : 0;
	}
	uint32_t skippedCount(const uint8_t ch) {
		return ch < TRIG_MAX ? g_skipped[ch] : 0;
	}
	uint32_t stalledCount(const uint8_t ch) {
		return ch < TRIG_MAX ? g_stalled[ch] : 0;
	}
	uint32_t trigDrops() {
		return g_trig_drops;
	}
	uint32_t strobeDrops() {
		return g_strobe_drops;
	}

}  // namespace channels
