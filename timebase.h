#pragma once
#include <Arduino.h>

#include "config.h"

// =============================================================================
//  timebase - GPT1 free-running counter, software-extended to 64 bit.
//
//  The firmware's only time ruler; every capture and compare value is in its
//  ticks. GPT1 is 32 bit and wraps every 2^32 ticks (57 s at 75 MHz); the ROV
//  interrupt extends it to 64 bit.
// =============================================================================

namespace timebase {
// GPT register bits. Prefixed to avoid clashing with imxrt.h.
#define SS_GPT_CR_EN (1u << 0)
#define SS_GPT_CR_ENMOD (1u << 1)
#define SS_GPT_CR_DBGEN (1u << 2)
#define SS_GPT_CR_WAITEN (1u << 3)
#define SS_GPT_CR_DOZEEN (1u << 4)
#define SS_GPT_CR_STOPEN (1u << 5)
#define SS_GPT_CR_CLKSRC(n) (((n) & 7u) << 6)
#define SS_GPT_CR_FRR (1u << 9)
#define SS_GPT_CR_EN_24M (1u << 10)
#define SS_GPT_CR_SWR (1u << 15)
#define SS_GPT_CR_IM1(n) (((n) & 3u) << 16)
#define SS_GPT_CR_IM2(n) (((n) & 3u) << 18)
#define SS_GPT_CR_OM1(n) (((n) & 7u) << 20)
#define SS_GPT_CR_OM2(n) (((n) & 7u) << 23)
#define SS_GPT_CR_OM3(n) (((n) & 7u) << 26)
#define SS_GPT_CR_FO1 (1u << 29)
#define SS_GPT_CR_FO2 (1u << 30)
#define SS_GPT_CR_FO3 (1u << 31)

// Output compare modes
#define SS_GPT_OM_DISCONNECT 0
#define SS_GPT_OM_TOGGLE 1
#define SS_GPT_OM_CLEAR 2
#define SS_GPT_OM_SET 3

// Input capture modes
#define SS_GPT_IM_DISABLE 0
#define SS_GPT_IM_RISING 1
#define SS_GPT_IM_FALLING 2
#define SS_GPT_IM_BOTH 3

#define SS_GPT_IR_OF1IE (1u << 0)
#define SS_GPT_IR_OF2IE (1u << 1)
#define SS_GPT_IR_OF3IE (1u << 2)
#define SS_GPT_IR_IF1IE (1u << 3)
#define SS_GPT_IR_IF2IE (1u << 4)
#define SS_GPT_IR_ROVIE (1u << 5)

#define SS_GPT_SR_OF1 (1u << 0)
#define SS_GPT_SR_OF2 (1u << 1)
#define SS_GPT_SR_OF3 (1u << 2)
#define SS_GPT_SR_IF1 (1u << 3)
#define SS_GPT_SR_IF2 (1u << 4)
#define SS_GPT_SR_ROV (1u << 5)

	void begin();

	// Nominal tick rate (Hz). The true rate is measured from PPS by timesync.
	double nominalHz();

	// Current 64-bit tick. Interrupt-safe.
	uint64_t now();

	// Widen a recent 32-bit capture to 64 bit. Valid only if it happened within
	// the last 2^32 ticks.
	uint64_t extend(uint32_t cnt32);

	// Trigger and capture share the GPT1 vector, so they hook in here.
	// icr1 is snapshot before SR is cleared - the reverse order lets a new
	// capture overwrite ICR1 before we read it.
	typedef void (*IsrHook)(uint32_t status, uint32_t icr1);
	void setIsrHook(IsrHook h);

	// Configure GPT1_CAPTURE1 (PPS) on pin 48 / GPIO_EMC_24 ALT4 - the only pad
	// broken out on Teensy 4.1. Writes the pad mux AND the input daisy register;
	// without the daisy write no capture ever fires and nothing reports an error.
	//     daisy     0 / 1, or 0xFF to probe both
	//     drive_pin loopback self-test output (jumper to pin 48); 0xFF skips the test, in which case daisy must be a concrete value
	//     out_daisy receives the value that took effect
	// Returns true if capture is verified working.
	bool setupCapture1(bool rising, uint8_t daisy, uint8_t drive_pin, uint8_t &out_daisy);

}  // namespace timebase
