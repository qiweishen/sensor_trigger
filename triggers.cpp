#include "triggers.h"

#include <cmath>

#include "config.h"
#include "events.h"
#include "timebase.h"


namespace triggers {
	static_assert(TRIG_CFG[0].pin == 33 && TRIG_CFG[1].pin == 4 && TRIG_CFG[2].pin == 37 && TRIG_CFG[3].pin == 36,
				  "pins fixed by FlexPWM2 SM0/SM3 wiring");

	static constexpr uint8_t kSubmodule[2] = { 0, 3 };
	static constexpr uint8_t kMux[2] = { 1, 6 };					// pin ALT: SM0, SM3
	static constexpr uint16_t kModules = (1u << 0) | (1u << 3);
	static constexpr uint16_t kOutputs = FLEXPWM_OUTEN_PWMA_EN(kModules) | FLEXPWM_OUTEN_PWMB_EN(kModules);
	static constexpr uint16_t kRise = FLEXPWM_SMSTS_CMPF(1u << 2);	// VAL2 compare

	static double g_hz[2] = { FX_DEFAULT_HZ, JAI_DEFAULT_HZ };
	static Timing g_timing[2];


	// SM0: bus / 128. SM3: SM0 AUX clock / 2^prescale (JAI below 18 Hz).
	static bool calculate(uint8_t group, double hz, Timing &t) {
		if (!(hz >= 0) || !std::isfinite(hz)) {
			return false;
		}
		const double rate = hz > 0 ? hz : (group == 0 ? 50 : 1);  // stopped SM0 still clocks SM3
		t.clock_hz = F_BUS_ACTUAL / 128.0;
		t.prescale = group == 0 ? 7 : 0;
		while (group == 1 && t.clock_hz / rate > 65535 && t.prescale < 7) {
			++t.prescale;
			t.clock_hz /= 2;
		}
		const double n = std::ceil(t.clock_hz / rate), w = std::ceil(t.clock_hz * PULSE_US / 1e6);
		// 16-bit period; rise at W, fall at 2W
		if (n > 65535 || n <= 2 * w + 1) {
			return false;
		}
		t.period = n;
		t.width = w;
		t.actual_hz = hz > 0 ? t.clock_hz / n : 0;
		t.pulse_us = 1e6 * w / t.clock_hz;
		return true;
	}


	template<uint8_t group>
	static void pwmIsr() {
		const uint64_t tick = timebase::now();
		IMXRT_FLEXPWM2.SM[kSubmodule[group]].STS = kRise;  // W1C
		for (uint8_t ch = 2 * group; ch < 2 * group + 2; ++ch)
			if (TRIG_CFG[ch].enabled) {
				events::push(events::TRIG, ch, HIGH, tick);
			}
		asm volatile("dsb");
	}


	static void program(uint8_t group) {
		const Timing &t = g_timing[group];
		auto &sm = IMXRT_FLEXPWM2.SM[kSubmodule[group]];
		// independent A/B, local reload; SM3 clocked from SM0 AUX
		sm.CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN | FLEXPWM_SMCTRL2_FRCEN |
				   FLEXPWM_SMCTRL2_CLK_SEL(group ? 2 : 0);
		sm.CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_LDMOD | FLEXPWM_SMCTRL_PRSC(t.prescale);
		sm.INIT = sm.VAL0 = 0;
		sm.VAL1 = t.period - 1;
		sm.VAL2 = sm.VAL4 = t.width;	  // rise, after a defined LOW start
		sm.VAL3 = sm.VAL5 = 2 * t.width;  // fall
		sm.FRACVAL1 = sm.FRACVAL2 = sm.FRACVAL3 = sm.FRACVAL4 = sm.FRACVAL5 = 0;
		sm.FRCTRL = sm.OCTRL = sm.DTCNT0 = sm.DTCNT1 = sm.DISMAP0 = sm.DISMAP1 = 0;
		sm.DMAEN = sm.TCTRL = sm.CAPTCTRLA = sm.CAPTCTRLB = sm.CAPTCTRLX = sm.INTEN = 0;
	}


	void begin() {
		CCM_CCGR4 |= CCM_CCGR4_PWM2(CCM_CCGR_ON);
		stop();
		setRates(FX_DEFAULT_HZ, JAI_DEFAULT_HZ);
		attachInterruptVector(IRQ_FLEXPWM2_0, pwmIsr<0>);
		attachInterruptVector(IRQ_FLEXPWM2_3, pwmIsr<1>);
		NVIC_SET_PRIORITY(IRQ_FLEXPWM2_0, PWM_IRQ_PRIORITY);
		NVIC_SET_PRIORITY(IRQ_FLEXPWM2_3, PWM_IRQ_PRIORITY);
		NVIC_ENABLE_IRQ(IRQ_FLEXPWM2_0);
		NVIC_ENABLE_IRQ(IRQ_FLEXPWM2_3);
	}


	bool setRates(double fx_hz, double jai_hz) {
		Timing fx, jai;
		if (!calculate(0, fx_hz, fx) || !calculate(1, jai_hz, jai)) {
			return false;
		}
		g_hz[0] = fx_hz;
		g_hz[1] = jai_hz;
		g_timing[0] = fx;
		g_timing[1] = jai;
		return true;
	}


	double requested(uint8_t group) {
		return g_hz[group];
	}


	const Timing &timing(uint8_t group) {
		return g_timing[group];
	}


	double freqHz(uint8_t ch) {
		return TRIG_CFG[ch].enabled ? g_timing[ch / 2].actual_hz : 0;
	}


	void start() {
		stop();
		IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_CLDOK(kModules);
		program(0);
		program(1);
		IMXRT_FLEXPWM2.DTSRCSEL &= ~uint16_t(0xF00F);  // SM0/SM3 outputs from PWM generator
		IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_LDOK(kModules);
		uint16_t run = 0, unmask = 0;
		for (uint8_t g = 0; g < 2; ++g) {
			auto &sm = IMXRT_FLEXPWM2.SM[kSubmodule[g]];
			sm.CTRL2 |= FLEXPWM_SMCTRL2_FORCE;	// counter + outputs start LOW
			sm.STS = 0xFFFF;
			if (g_hz[g] == 0) {
				continue;
			}
			sm.INTEN = FLEXPWM_SMINTEN_CMPIE(1u << 2);	// rising edge only
			run |= 1u << kSubmodule[g];
			for (uint8_t ch = 2 * g; ch < 2 * g + 2; ++ch) {
				if (!TRIG_CFG[ch].enabled) {
					continue;
				}
				*portConfigRegister(TRIG_CFG[ch].pin) = kMux[g];
				const uint16_t bit = 1u << kSubmodule[g];
				unmask |= ch % 2 ? FLEXPWM_MASK_MASKA(bit) : FLEXPWM_MASK_MASKB(bit);  // even ch = B
			}
		}
		NVIC_CLEAR_PENDING(IRQ_FLEXPWM2_0);
		NVIC_CLEAR_PENDING(IRQ_FLEXPWM2_3);
		IMXRT_FLEXPWM2.OUTEN |= kOutputs;
		IMXRT_FLEXPWM2.MASK = (IMXRT_FLEXPWM2.MASK & ~unmask) | FLEXPWM_MASK_UPDATE_MASK(kModules);
		if (run) {
			run |= 1u << 0;	 // JAI needs SM0 running for its clock
		}
		IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_RUN(run);
	}


	void stop() {
		for (uint8_t sm: kSubmodule) {
			IMXRT_FLEXPWM2.SM[sm].INTEN = 0;
		}
		IMXRT_FLEXPWM2.MCTRL &= ~FLEXPWM_MCTRL_RUN(kModules);
		IMXRT_FLEXPWM2.MASK |= kOutputs | FLEXPWM_MASK_UPDATE_MASK(kModules);
		for (const auto &c: TRIG_CFG) {	 // GPIO takeover: LOW even without PWM clock
			digitalWriteFast(c.pin, LOW);
			pinMode(c.pin, OUTPUT);
		}
	}
}  // namespace triggers
