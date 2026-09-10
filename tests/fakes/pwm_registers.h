#pragma once
#include <cstdint>
#define CCM_CCGR_ON				3
#define CCM_CCGR4_PWM2(n)			((uint32_t)(((n) & 0x03) << 18))
#define FLEXPWM_SMCTRL2_DBGEN			((uint16_t)(1<<15))
#define FLEXPWM_SMCTRL2_WAITEN			((uint16_t)(1<<14))
#define FLEXPWM_SMCTRL2_INDEP			((uint16_t)(1<<13))
#define FLEXPWM_SMCTRL2_FRCEN			((uint16_t)(1<<7))
#define FLEXPWM_SMCTRL2_FORCE			((uint16_t)(1<<6))
#define FLEXPWM_SMCTRL2_CLK_SEL(n)		((uint16_t)(((n) & 0x03) << 0))
#define FLEXPWM_SMCTRL_FULL			((uint16_t)(1<<10))
#define FLEXPWM_SMCTRL_PRSC(n)			((uint16_t)(((n) & 0x0F) << 4))
#define FLEXPWM_SMCTRL_LDMOD			((uint16_t)(1<<2))
#define FLEXPWM_SMSTS_REF			((uint16_t)(1<<13))
#define FLEXPWM_SMSTS_CMPF(n)			((uint16_t)(((n) & 0x3F) << 0))
#define FLEXPWM_SMINTEN_REIE			((uint16_t)(1<<13))
#define FLEXPWM_SMINTEN_CMPIE(n)		((uint16_t)(((n) & 0x3F) << 0))
#define FLEXPWM_OUTEN_PWMA_EN(n)		((uint16_t)(((n) & 0x0F) << 8))
#define FLEXPWM_OUTEN_PWMB_EN(n)		((uint16_t)(((n) & 0x0F) << 4))
#define FLEXPWM_MASK_UPDATE_MASK(n)		((uint16_t)(((n) & 0x0F) << 12))
#define FLEXPWM_MASK_MASKA(n)			((uint16_t)(((n) & 0x0F) << 8))
#define FLEXPWM_MASK_MASKB(n)			((uint16_t)(((n) & 0x0F) << 4))
#define FLEXPWM_MCTRL_RUN(n)			((uint16_t)(((n) & 0x0F) << 8))
#define FLEXPWM_MCTRL_CLDOK(n)			((uint16_t)(((n) & 0x0F) << 4))
#define FLEXPWM_MCTRL_LDOK(n)			((uint16_t)(((n) & 0x0F) << 0))
inline uint32_t CCM_CCGR4 = 0;
struct W1C {
    uint16_t bits = 0;
    operator uint16_t() const { return bits; }
    W1C &operator=(uint16_t value) { bits &= ~value; return *this; }
    void raise(uint16_t value) { bits |= value; }
};
struct PwmRegisters {
    struct Submodule {
        uint16_t CNT=0, INIT=0, CTRL2=0, CTRL=0;
        uint16_t VAL0=0, VAL1=0, VAL2=0, VAL3=0, VAL4=0, VAL5=0;
        uint16_t FRACVAL1=0, FRACVAL2=0, FRACVAL3=0, FRACVAL4=0, FRACVAL5=0;
        uint16_t FRCTRL=0, OCTRL=0, DTCNT0=0, DTCNT1=0;
        uint16_t DISMAP0=0, DISMAP1=0, DMAEN=0, TCTRL=0;
        uint16_t CAPTCTRLA=0, CAPTCTRLB=0, CAPTCTRLX=0, INTEN=0;
        W1C STS;
        uint64_t origin=0, next=0;
    } SM[4];
    struct MasterControl {
        PwmRegisters *p;
        uint16_t value=0;
        operator uint16_t() const { return value; }
        MasterControl &operator=(uint16_t v) {
            for (unsigned i=0; i<4; ++i) {
                const bool old_run = value & (0x100u << i), run = v & (0x100u << i);
                if (!run) p->SM[i].CNT = 0;
                if (run && !old_run) { p->SM[i].origin = fake_tick; p->SM[i].next = 0; }
            }
            value = v & 0xFF00; // CLDOK self-clears; immediate LDOK loads consumed
            return *this;
        }
        MasterControl &operator|=(uint16_t v) { return *this = value | v; }
        MasterControl &operator&=(uint16_t v) { return *this = value & v; }
    } MCTRL{this};
    uint16_t OUTEN=0, MASK=0, DTSRCSEL=0;
};
inline PwmRegisters IMXRT_FLEXPWM2;
