#pragma once
#include <Arduino.h>

#include "config.h"

// =============================================================================
//  channels (pure-logger role) - trigger generation + strobe read-back, RAW.
//
//  Triggers: a plain periodic GPT1 output-compare per channel (NOT GNSS-aligned).
//            Every edge's exact compare tick is queued; the host converts it to
//            GNSS time offline. No lock/holdover - it just free-runs.
//  Strobes:  both edges of each Exposure-Active / Strobe-Out line are queued raw
//            (tick + level). The host reconstructs exposure start/end and matches
//            frames.
//  Each stream carries a per-channel sequence number, so a dropped edge shows up
//  as a gap in the log.
// =============================================================================

namespace channels {

	void begin();
	void gptIsr(uint32_t sr);  // forwarded from timebase's GPT1 interrupt

	// Drain one queued edge in loop(); false if empty. `level` is the raw pin level.
	bool popTrig(uint8_t &ch, uint8_t &level, uint64_t &tick, uint32_t &seq);
	bool popStrobe(uint8_t &ch, uint8_t &level, uint64_t &tick, uint32_t &seq);

	// Trigger generation on/off (default on). Pausing parks the pins idle.
	void setEnabled(bool on);
	bool enabled();

	// Health (monotonic).
	uint32_t trigCount(uint8_t ch);		// trigger edges emitted for this channel (incl. dropped seq)
	uint32_t strobeCount(uint8_t ch);	// strobe edges seen for this channel
	uint32_t skippedCount(uint8_t ch);	// trigger periods skipped (ISR fell behind)
	uint32_t trigDrops();				// trigger event-ring overflow drops
	uint32_t strobeDrops();				// strobe event-ring overflow drops

}  // namespace channels
