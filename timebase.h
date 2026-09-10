#pragma once
#include <Arduino.h>

#include "config.h"

// =============================================================================
//  timebase - GPT1 free-running counter, software-extended to 64 bit.
//
//  The firmware's only time ruler; every logged event value is in its
//  ticks. GPT1 is 32 bit and wraps every 2^32 ticks (57 s at 75 MHz); the ROV
//  interrupt extends it to 64 bit.
// =============================================================================

namespace timebase {
// GPT register bits actually used by this build. Prefixed to avoid clashing
// with imxrt.h.
#define SS_GPT_CR_EN (1u << 0)
#define SS_GPT_CR_ENMOD (1u << 1)
#define SS_GPT_CR_DBGEN (1u << 2)
#define SS_GPT_CR_WAITEN (1u << 3)
#define SS_GPT_CR_STOPEN (1u << 5)
#define SS_GPT_CR_CLKSRC(n) (((n) & 7u) << 6)
#define SS_GPT_CR_FRR (1u << 9)
#define SS_GPT_CR_SWR (1u << 15)
#define SS_GPT_IR_ROVIE (1u << 5)

#define SS_GPT_SR_ROV (1u << 5)

	void begin();

	// nominal tick rate, Hz; true rate is fit from PPS by the host
	double nominalHz();

	// current 64-bit tick; safe from loop() and ISRs at GPT1 priority or below;
	// never call from an ISR that preempts GPT1 (rollover window race)
	uint64_t now();


}  // namespace timebase
