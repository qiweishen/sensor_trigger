#include "timebase.h"

#include "config.h"


namespace timebase {
	static volatile uint64_t g_high = 0;  // rollovers << 32


	static void gpt1Isr() {
		const uint32_t sr = GPT1_SR;
		GPT1_SR = sr;		  // W1C
		if (sr & GPT_SR_ROV) {
			g_high += 1ULL << 32;
		}
		asm volatile("dsb");  // clear before exit
	}


	void begin() {
		CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON) | CCM_CCGR1_GPT1_SERIAL(CCM_CCGR_ON);
		// perclk: 24 MHz crystal -> ipg / TB_PERCLK_DIV
		CCM_CSCMR1 =
				(CCM_CSCMR1 & ~(CCM_CSCMR1_PERCLK_CLK_SEL | CCM_CSCMR1_PERCLK_PODF(0x3F))) | CCM_CSCMR1_PERCLK_PODF(TB_PERCLK_DIV - 1);
		GPT1_CR = 0;
		GPT1_CR = GPT_CR_SWR;
		while (GPT1_CR & GPT_CR_SWR) {
		}
		GPT1_PR = 0;
		GPT1_SR = 0x3F;
		GPT1_CR = GPT_CR_CLKSRC(1) | GPT_CR_FRR | GPT_CR_ENMOD | GPT_CR_WAITEN | GPT_CR_STOPEN | GPT_CR_DBGEN;
		GPT1_IR = GPT_IR_ROVIE;
		attachInterruptVector(IRQ_GPT1, gpt1Isr);
		NVIC_SET_PRIORITY(IRQ_GPT1, TB_IRQ_PRIORITY);
		NVIC_ENABLE_IRQ(IRQ_GPT1);
		GPT1_CR |= GPT_CR_EN;
	}


	double hz() {
		return F_BUS_ACTUAL / double(TB_PERCLK_DIV);
	}


	uint64_t now() {
		uint64_t high;
		uint32_t cnt, sr;
		do {
			high = g_high;
			cnt = GPT1_CNT;
			sr = GPT1_SR;
		} while (high != g_high);
		// rollover pending, ISR not yet run
		if ((sr & GPT_SR_ROV) && cnt < 0x80000000u) {
			high += 1ULL << 32;
		}
		return high | cnt;
	}
}  // namespace timebase
