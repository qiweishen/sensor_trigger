#include "channels.h"

#include <cmath>
#include <cstring>

#include "timebase.h"

namespace channels {

	// ---------------------------------------------------------------------------
	// GPT1 output-compare helpers (software trigger generation)
	static_assert(TRIG_MAX <= 3, "GPT1 has only 3 output compare channels");

	static volatile uint32_t *ocrPtr(const uint8_t ch) {
		return (ch == 0) ? &GPT1_OCR1 : (ch == 1) ? &GPT1_OCR2 : &GPT1_OCR3;
	}
	static uint32_t ofFlag(const uint8_t ch) {
		return (ch == 0) ? SS_GPT_SR_OF1 : (ch == 1) ? SS_GPT_SR_OF2 : SS_GPT_SR_OF3;
	}
	static uint32_t ofIe(const uint8_t ch) {
		return (ch == 0) ? SS_GPT_IR_OF1IE : (ch == 1) ? SS_GPT_IR_OF2IE : SS_GPT_IR_OF3IE;
	}


	// ---------------------------------------------------------------------------
	// Trigger channel state
	struct TrigState {
		uint64_t period_ticks;
		uint64_t width_ticks;
		uint64_t phase_ticks;
		uint64_t rise_tick;		 // absolute tick of the current/next rising edge
		volatile uint8_t stage;	 // 0 = rise scheduled, 1 = fall scheduled
		bool cfg_ok;			 // freq >= 1 Hz (else channel disabled)
	};
	static TrigState g_trig[TRIG_MAX];
	static volatile bool g_enabled = false;
	static uint64_t g_margin_ticks = 0;


	// ---------------------------------------------------------------------------
	// Event rings. Each has a SINGLE producer context: g_trigq from the GPT1 ISR,
	// g_strobeq from the GPIO ISR - so both are plain SPSC (no locks). Per-channel
	// sequence numbers advance even on a drop, so a gap in the log is detectable.
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


	// Write a 32-bit compare and read back to confirm it is still in the future. With FRR=1
	// the GPT compares for EQUALITY, so a value that just passed will not match again for a
	// full 2^32 lap; the read-back makes that visible.
	static bool armCompare(const uint8_t ch, const uint64_t tick) {
		*ocrPtr(ch) = static_cast<uint32_t>(tick & 0xFFFFFFFFULL);
		GPT1_SR = ofFlag(ch);
		GPT1_IR |= ofIe(ch);
		const uint32_t cnt = GPT1_CNT;
		return static_cast<int32_t>(cnt - static_cast<uint32_t>(tick)) < 0;
	}


	// Catch rise_tick up to the future (if the ISR fell behind) and arm the rising edge.
	static void armRise(const uint8_t ch) {
		TrigState &s = g_trig[ch];
		s.stage = 0;
		for (int attempt = 0; attempt < 8; attempt++) {
			const uint64_t deadline = timebase::now() + g_margin_ticks;
			if (static_cast<int64_t>(s.rise_tick - deadline) <= 0) {
				const uint64_t behind = deadline - s.rise_tick;
				const uint64_t n = behind / s.period_ticks + 1;
				s.rise_tick += n * s.period_ticks;
				g_skipped[ch] += static_cast<uint32_t>(n);
			}
			if (armCompare(ch, s.rise_tick)) {
				return;
			}
			// Slipped past while writing the OCR: advance one period and retry.
			s.rise_tick += s.period_ticks;
			g_skipped[ch]++;
		}
		GPT1_IR &= ~ofIe(ch);  // give up (should never happen: period >> margin)
	}


	// GPT1 interrupt (forwarded by timebase). Single pass: the physical edge skew between
	// two channels that fire in the same ISR is a few microseconds, but every edge is logged
	// at its EXACT compare tick, so the recorded times are unaffected.
	void gptIsr(const uint32_t sr) {
		if (!g_enabled) {
			return;
		}
		const uint32_t ie = GPT1_IR;
		for (uint8_t i = 0; i < TRIG_MAX; i++) {
			if (!g_trig[i].cfg_ok) {
				continue;
			}
			if (!(sr & ofFlag(i)) || !(ie & ofIe(i))) {
				continue;
			}
			TrigState &s = g_trig[i];
			const TriggerCfg &c = TRIG_CFG[i];

			if (s.stage == 0) {	 // rising edge fired
				digitalWriteFast(c.pin, activeLevel(i));
				pushTrig(i, activeLevel(i), s.rise_tick);
				s.stage = 1;
				if (!armCompare(i, s.rise_tick + s.width_ticks)) {
					// Fall slipped past: a late fall only stretches the pulse, and leaving it
					// would pin the pin asserted for a full lap - so drop it in software now.
					digitalWriteFast(c.pin, idleLevel(i));
					pushTrig(i, idleLevel(i), s.rise_tick + s.width_ticks);
					s.rise_tick += s.period_ticks;
					armRise(i);
				}
			} else {  // falling edge fired
				digitalWriteFast(c.pin, idleLevel(i));
				pushTrig(i, idleLevel(i), s.rise_tick + s.width_ticks);
				s.rise_tick += s.period_ticks;
				armRise(i);
			}
		}
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
			GPT1_SR = ofFlag(0) | ofFlag(1) | ofFlag(2);  // drop stale flags
			g_enabled = true;
			const uint64_t start = timebase::now() + g_margin_ticks * 4;
			for (uint8_t i = 0; i < TRIG_MAX; i++) {
				if (!g_trig[i].cfg_ok) {
					continue;
				}
				g_trig[i].rise_tick = start + g_trig[i].phase_ticks;
				armRise(i);
			}
		} else {
			g_enabled = false;
			for (uint8_t i = 0; i < TRIG_MAX; i++) {
				GPT1_IR &= ~ofIe(i);
				setIdle(i);
			}
		}
		__enable_irq();
	}


	void begin() {
		const double nom = timebase::nominalHz();
		g_margin_ticks = static_cast<uint64_t>(nom * 5e-6);	 // 5 us

		for (uint8_t i = 0; i < TRIG_MAX; i++) {
			const TriggerCfg &c = TRIG_CFG[i];
			memset(&g_trig[i], 0, sizeof(TrigState));
			g_trig_seq[i] = 0;
			g_skipped[i] = 0;

			// Period is a whole number of ticks: freq must be >= 1 Hz (and, to be safe, well
			// below the tick rate). Reject loudly and disable rather than misbehave silently.
			bool ok = (c.freq_hz >= 1.0);
			if (c.freq_hz > 0.0 && !ok) {
				Serial.printf("# !! WARNING: trig[%u] %s freq=%.4fHz < 1 Hz unsupported; channel DISABLED\n", i, c.name, c.freq_hz);
			}
			g_trig[i].cfg_ok = ok;
			if (ok) {
				g_trig[i].period_ticks = static_cast<uint64_t>(llround(nom / c.freq_hz));
				if (g_trig[i].period_ticks < 2) {
					g_trig[i].period_ticks = 2;
				}
				g_trig[i].width_ticks = static_cast<uint64_t>(c.pulse_us * 1e-6 * nom);
				if (g_trig[i].width_ticks < g_margin_ticks) {
					g_trig[i].width_ticks = g_margin_ticks;	 // must outlast the ISR path to the fall
				}
				if (g_trig[i].width_ticks >= g_trig[i].period_ticks) {
					g_trig[i].width_ticks = g_trig[i].period_ticks / 2;	 // keep fall before the next rise
				}
				g_trig[i].phase_ticks = static_cast<uint64_t>(c.phase_s * nom);
			}
			pinMode(c.pin, OUTPUT);
			setIdle(i);
		}

		for (uint8_t i = 0; i < STROBE_MAX; i++) {
			g_strobe_seq[i] = 0;
			// Bias toward the inactive level, so a floating/unpowered line reads idle instead
			// of making the ISR free-run on noise and flooding the event ring.
			pinMode(STROBE_CFG[i].pin, STROBE_CFG[i].active_high ? INPUT_PULLDOWN : INPUT_PULLUP);
		}
		for (uint8_t i = 0; i < STROBE_MAX; i++) {
			attachInterrupt(digitalPinToInterrupt(STROBE_CFG[i].pin), kStrobeIsr[i], CHANGE);
		}
		// PPS (software mode) shares this vector; keep >= 32 from GPT1 (see config.h).
		NVIC_SET_PRIORITY(IRQ_GPIO6789, STROBE_IRQ_PRIORITY);

		setEnabled(true);  // start triggering immediately (no lock needed)
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
	uint32_t trigDrops() {
		return g_trig_drops;
	}
	uint32_t strobeDrops() {
		return g_strobe_drops;
	}

}  // namespace channels
