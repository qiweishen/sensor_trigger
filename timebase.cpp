#include "timebase.h"

namespace timebase {

	static volatile uint64_t g_rollover = 0;  // low 32 bits always zero
	static double g_nominal_hz = 0.0;


	double nominalHz() {
		return g_nominal_hz;
	}


	// -----------------------------------------------------------------------------
	// GPT1 ISR: extend the free-running time ruler
	static void gpt1_isr() {
		uint32_t sr = GPT1_SR;
		GPT1_SR = sr;				// write-1-clear

		if (sr & SS_GPT_SR_ROV) {
			g_rollover += 0x100000000ULL;
		}

		asm volatile("dsb");  // flag clear must retire before exit (else re-entry)
	}


	// -----------------------------------------------------------------------------
	void begin() {
		static_assert(TB_CLKSRC == 1, "nominalHz requires the perclk source");
		// ungate GPT1 clocks
		CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON) | CCM_CCGR1_GPT1_SERIAL(CCM_CCGR_ON);

		// repoint perclk: 24 MHz crystal -> ipg_clk / TB_PERCLK_DIV
		// NOTE perclk also feeds the PIT; IntervalTimer assumes 24 MHz - don't use it after this
		static_assert(TB_PERCLK_DIV >= 1 && TB_PERCLK_DIV <= 64, "PERCLK_PODF is 6 bits, divider range 1..64");
		CCM_CSCMR1 =
				(CCM_CSCMR1 & ~(CCM_CSCMR1_PERCLK_CLK_SEL | CCM_CSCMR1_PERCLK_PODF(0x3F))) | CCM_CSCMR1_PERCLK_PODF(TB_PERCLK_DIV - 1);

		GPT1_CR = 0;
		GPT1_CR = SS_GPT_CR_SWR;  // software reset
		while (GPT1_CR & SS_GPT_CR_SWR) {
		}

		GPT1_IR = 0;
		GPT1_SR = 0x3F;	 // write-1-to-clear all status flags

		// out-of-range prescaler would silently mis-scale every timestamp
		static_assert(TB_PRESCALER >= 1 && TB_PRESCALER <= 4096, "GPT1_PR is 12 bits, prescaler range 1..4096");
		GPT1_PR = TB_PRESCALER - 1;
		// must track the dividers actually written (never hard-code the rate)
		g_nominal_hz = static_cast<double>(F_BUS_ACTUAL) / static_cast<double>(TB_PERCLK_DIV) / static_cast<double>(TB_PRESCALER);

		const uint32_t cr =
				SS_GPT_CR_CLKSRC(TB_CLKSRC) | SS_GPT_CR_FRR | SS_GPT_CR_ENMOD | SS_GPT_CR_WAITEN | SS_GPT_CR_STOPEN | SS_GPT_CR_DBGEN;

		GPT1_CR = cr;
		GPT1_IR = SS_GPT_IR_ROVIE;

		attachInterruptVector(IRQ_GPT1, gpt1_isr);
		NVIC_SET_PRIORITY(IRQ_GPT1, TB_IRQ_PRIORITY);  // must outrank USB / UART / GPIO
		NVIC_ENABLE_IRQ(IRQ_GPT1);

		GPT1_CR |= SS_GPT_CR_EN;
	}


	// -----------------------------------------------------------------------------
	uint64_t now() {
		uint64_t base;
		uint32_t c, sr;

		// self-consistent (base, cnt, pending-flag) snapshot
		do {
			base = g_rollover;
			c = GPT1_CNT;
			sr = GPT1_SR;
		} while (base != g_rollover);

		// rollover latched before CNT was read -> high word not yet advanced
		if ((sr & SS_GPT_SR_ROV) && c < 0x80000000u) {
			base += 0x100000000ULL;
		}

		return base + static_cast<uint64_t>(c);
	}


}  // namespace timebase
