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
#define SS_GPT_CR_IM1(n) (((n) & 3u) << 16)

// input capture modes (IM1)
#define SS_GPT_IM_RISING 1
#define SS_GPT_IM_FALLING 2

#define SS_GPT_IR_OF1IE (1u << 0)
#define SS_GPT_IR_OF2IE (1u << 1)
#define SS_GPT_IR_OF3IE (1u << 2)
#define SS_GPT_IR_IF1IE (1u << 3)
#define SS_GPT_IR_ROVIE (1u << 5)

#define SS_GPT_SR_OF1 (1u << 0)
#define SS_GPT_SR_OF2 (1u << 1)
#define SS_GPT_SR_OF3 (1u << 2)
#define SS_GPT_SR_IF1 (1u << 3)
#define SS_GPT_SR_ROV (1u << 5)

	void begin();

	// nominal tick rate, Hz; true rate is fit from PPS by the host
	double nominalHz();

	// current 64-bit tick; safe from loop() and ISRs at GPT1 priority or below;
	// never call from an ISR that preempts GPT1 (rollover window race)
	uint64_t now();

	// widen a recent 32-bit capture to 64 bit; valid within the last 2^32 ticks
	uint64_t extend(uint32_t cnt32);

	// trigger + capture share the GPT1 vector; icr1 is snapshot before SR is
	// cleared so a new capture cannot overwrite it first
	typedef void (*IsrHook)(uint32_t status, uint32_t icr1);
	void setIsrHook(IsrHook h);

	// GPT1_CAPTURE1 (PPS) on pin 48 / GPIO_EMC_24 ALT4 - the only pad broken out
	// on Teensy 4.1. Writes the pad mux AND the input daisy register (without
	// the daisy write no capture ever fires, silently).
	//   daisy     0 / 1, or 0xFF = probe both
	//   drive_pin loopback self-test output (jumper to 48); 0xFF = skip test
	//             (then daisy must be concrete)
	//   out_daisy receives the value that took effect
	// true = capture verified working
	bool setupCapture1(bool rising, uint8_t daisy, uint8_t drive_pin, uint8_t &out_daisy);

}  // namespace timebase
