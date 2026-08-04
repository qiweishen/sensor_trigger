#include "timebase.h"

namespace timebase {

	static volatile uint64_t g_rollover = 0;  // low 32 bits always zero
	static double g_nominal_hz = 0.0;
	static IsrHook g_hook = nullptr;		  // Point -> Interrupt callback function


	double nominalHz() {
		return g_nominal_hz;
	}


	void setIsrHook(IsrHook h) {
		g_hook = h;
	}


	// -----------------------------------------------------------------------------
	// GPT1 ISR: rollover + dispatch to trigger/capture
	static void gpt1_isr() {
		uint32_t sr = GPT1_SR;
		// Read ICR1 before clearing SR
		uint32_t icr1 = GPT1_ICR1;
		GPT1_SR = sr;  // clear

		if (sr & SS_GPT_SR_ROV) {
			g_rollover += 0x100000000ULL;
		}
		if (g_hook) {
			g_hook(sr, icr1);
		}

		asm volatile("dsb");  // Ensure the flag clear retired before we leave
	}


	// -----------------------------------------------------------------------------
	// GPT1_CAPTURE1 (PPS hardware capture)
	static void enableCapture1(const bool rising) {
		// IM1 is CR bits 16-17
		GPT1_CR = (GPT1_CR & ~(3u << 16)) | SS_GPT_CR_IM1(rising ? SS_GPT_IM_RISING : SS_GPT_IM_FALLING);
		GPT1_SR = SS_GPT_SR_IF1;  // drop any spurious capture from the reconfigure
		GPT1_IR |= SS_GPT_IR_IF1IE;
	}


	static void configCapturePad(const uint8_t daisy, const bool rising) {
		IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_24 = 4;	// ALT4 = GPT1_CAPTURE1
		// Schmitt trigger + weak bias to the idle level, so an unpowered INS or a
		// broken wire reads quiet instead of flooding the PPS queue with noise.
		IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_24 = IOMUXC_PAD_HYS | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(rising ? 0 : 3) |
											IOMUXC_PAD_SPEED(2) | IOMUXC_PAD_DSE(6);
		// Load-bearing: two pads can drive GPT1_CAPTURE1, so one must be selected.
		IOMUXC_GPT1_IPP_IND_CAPIN1_SELECT_INPUT = daisy;
		enableCapture1(rising);
	}


	// Drive one edge into the capture pin and check the hardware latched it.
	static bool tryCapture(const uint8_t daisy, const bool rising, const uint8_t drive_pin) {
		configCapturePad(daisy, rising);

		// The self-test polls the IF1 status flag by hand. Without masking, gpt1_isr()
		// preempts during the settle windows below and write-1-clears IF1 (its normal
		// GPT1_SR = sr) before we ever read it - the ISR's ~1 us latency always beats
		// the 200 us delay, so a pad that captures perfectly still reads "no capture"
		// and the self-test fails on good hardware. Mask the GPT1 vector for the test;
		// delayMicroseconds() is a CPU busy-loop and needs no interrupt. IF1IE stays as
		// configCapturePad() set it, so normal capture interrupts resume on unmask.
		NVIC_DISABLE_IRQ(IRQ_GPT1);

		// Settle at the idle level first
		digitalWrite(drive_pin, rising ? LOW : HIGH);
		delayMicroseconds(200);
		GPT1_SR = SS_GPT_SR_IF1;

		const uint32_t before = GPT1_CNT;
		digitalWrite(drive_pin, rising ? HIGH : LOW);
		delayMicroseconds(200);
		const uint32_t after = GPT1_CNT;

		bool ok = false;
		if (GPT1_SR & SS_GPT_SR_IF1) {	// something captured (else: wrong mux / daisy)
			const uint32_t cap = GPT1_ICR1;
			// The capture must land in a narrow window after the edge we drove
			const uint32_t span = after - before;
			const uint32_t off = cap - before;
			if (off <= span) {	// inside the window (else: likely a stale capture)
				const auto tol = static_cast<uint32_t>(g_nominal_hz * static_cast<double>(HW_SELFTEST_TOL_US) * 1e-6);
				ok = (off <= tol);
			}
		}
		GPT1_SR = SS_GPT_SR_IF1;	// drop the self-test capture either way
		NVIC_ENABLE_IRQ(IRQ_GPT1);
		return ok;
	}


	bool setupCapture1(const bool rising, const uint8_t daisy, const uint8_t drive_pin, uint8_t &out_daisy) {
		if (drive_pin == 0xFF) {
			// No self-test: daisy must be concrete
			if (daisy > 1) {
				return false;
			}
			configCapturePad(daisy, rising);
			out_daisy = daisy;
			return true;
		}

		pinMode(drive_pin, OUTPUT);
		const uint8_t first = (daisy > 1) ? 0 : daisy;
		const uint8_t count = (daisy > 1) ? 2 : 1;
		bool ok = false;
		for (uint8_t i = 0; i < count; i++) {
			const auto d = static_cast<uint8_t>(first + i);
			if (tryCapture(d, rising, drive_pin)) {
				out_daisy = d;
				ok = true;
				break;
			}
		}
		// Release the test pin so it stops driving the PPS line
		pinMode(drive_pin, INPUT);
		if (!ok) {
			// Stop IF1 from raising interrupts AND revert the pad from ALT4 capture back
			// to plain biased GPIO input. Clearing IF1IE alone is not enough: the pad
			// stays a live hardware capture, so a real edge keeps latching IF1 in SR and
			// the GPT1 rollover ISR forwards it to onPpsEvent() (gptHook checks only
			// sr & IF1). Reverting the mux removes the source entirely.
			GPT1_IR &= ~SS_GPT_IR_IF1IE;
			pinMode(PPS_PIN, rising ? INPUT_PULLDOWN : INPUT_PULLUP);
		}
		return ok;
	}


	// -----------------------------------------------------------------------------
	void begin() {
		// Ungate GPT1 (CCM = Clock Control Module, CCGR1 = Clock Gating Register 1)
		CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON) | CCM_CCGR1_GPT1_SERIAL(CCM_CCGR_ON);

		// Repoint perclk from the 24 MHz crystal to ipg_clk, divided by TB_PERCLK_DIV
		// perclk also feeds the PIT, and IntervalTimer hard-codes 24 MHz in its own conversion - do not use IntervalTimer after this
		static_assert(TB_PERCLK_DIV >= 1 && TB_PERCLK_DIV <= 64, "PERCLK_PODF is 6 bits, divider range 1..64");
		// CCM_CSCMR1 = Clock Control Module – Clock Switch & Divider Register 1
		// CCM_CSCMR1 = (CCM_CSCMR1 & ~(A | B)) | (B_new); clear A and B value, and write new B value
		CCM_CSCMR1 =
				(CCM_CSCMR1 & ~(CCM_CSCMR1_PERCLK_CLK_SEL | CCM_CSCMR1_PERCLK_PODF(0x3F))) | CCM_CSCMR1_PERCLK_PODF(TB_PERCLK_DIV - 1);

		GPT1_CR = 0;
		GPT1_CR = SS_GPT_CR_SWR;  // software reset
		while (GPT1_CR & SS_GPT_CR_SWR) {
		}

		GPT1_IR = 0;
		GPT1_SR = 0x3F;	 // write-1-to-clear all status flags

		GPT1_PR = (TB_PRESCALER - 1) & 0x0FFF;
		// Must track the divider actually written above
		// Hard-coding a "/1" assumption here would silently scale every timestamp if PODF changed.
		g_nominal_hz =
				static_cast<double>(F_BUS_ACTUAL) / static_cast<double>(TB_PERCLK_DIV) / static_cast<double>(TB_PRESCALER);	 // 75 MHz

		const uint32_t cr =
				SS_GPT_CR_CLKSRC(TB_CLKSRC) | SS_GPT_CR_FRR | SS_GPT_CR_ENMOD | SS_GPT_CR_WAITEN | SS_GPT_CR_STOPEN | SS_GPT_CR_DBGEN;

		GPT1_CR = cr;
		GPT1_IR = SS_GPT_IR_ROVIE;

		// Set interrupt function
		attachInterruptVector(IRQ_GPT1, gpt1_isr);
		// Must outrank USB / UART / GPIO
		NVIC_SET_PRIORITY(IRQ_GPT1, TB_IRQ_PRIORITY);
		NVIC_ENABLE_IRQ(IRQ_GPT1);

		// Enbale it
		GPT1_CR |= SS_GPT_CR_EN;
	}


	// -----------------------------------------------------------------------------
	uint64_t now() {
		uint64_t base;
		uint32_t c, sr;

		// Get a self-consistent (base, cnt, pending)
		do {
			base = g_rollover;	// number of rollovers
			c = GPT1_CNT;		// count number
			sr = GPT1_SR;		// read statue register
		} while (base != g_rollover);

		// Overflow occurred *before* we read CNT → advance high-word by 2^32
		if ((sr & SS_GPT_SR_ROV) && c < 0x80000000u) {
			base += 0x100000000ULL;
		}

		return base + static_cast<uint64_t>(c);
	}


	// -----------------------------------------------------------------------------
	uint64_t extend(const uint32_t cnt32) {
		const uint64_t n = now();
		uint64_t guess = (n & ~0xFFFFFFFFULL) | static_cast<uint64_t>(cnt32);
		if (guess > n) {
			guess -= 0x100000000ULL;  // capture happened before the wrap
		}
		return guess;
	}

}  // namespace timebase
