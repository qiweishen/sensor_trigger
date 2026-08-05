#pragma once
#include <Arduino.h>

#include "config.h"

// =============================================================================
//  channels - trigger generation + strobe read-back, raw.
//
//  Triggers: plain periodic GPT1 output-compare per channel, free-running (NOT
//            GNSS-aligned); each edge's compare tick is queued for the host.
//  Strobes:  both edges queued raw (tick + level); host reconstructs exposures.
//  Per-channel seq numbers advance even on drop -> gaps visible in the log.
// =============================================================================

namespace channels {

	void begin();
	void gptIsr(uint32_t sr);  // forwarded from timebase's GPT1 interrupt

	// drain one queued edge in loop(); false if empty; `level` = raw pin level
	bool popTrig(uint8_t &ch, uint8_t &level, uint64_t &tick, uint32_t &seq);
	bool popStrobe(uint8_t &ch, uint8_t &level, uint64_t &tick, uint32_t &seq);

	// trigger + strobe capture on/off (boot default OFF); off = trigger pins
	// parked idle, strobe edges ignored (not counted, not queued)
	void setEnabled(bool on);
	bool enabled();

	// runtime frequency override (only while disabled). hz <= 0 disables the
	// channel deliberately; invalid hz is rejected (false) and the old rate kept
	bool setFreqHz(uint8_t ch, double hz);
	double freqHz(uint8_t ch);	// effective rate for the header (0 = disabled)

	// zero all counters + empty the rings (next session from 0); only while disabled
	void resetCounts();

	// health (monotonic within a session)
	uint32_t trigCount(uint8_t ch);		// trigger edges emitted (incl. dropped seq)
	uint32_t strobeCount(uint8_t ch);	// strobe edges seen
	uint32_t skippedCount(uint8_t ch);	// trigger periods skipped (ISR fell behind)
	uint32_t stalledCount(uint8_t ch);	// armRise gave up; channel dead until next START
	uint32_t trigDrops();				// trigger ring overflow drops
	uint32_t strobeDrops();				// strobe ring overflow drops

}  // namespace channels
