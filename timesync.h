#pragma once
#include <Arduino.h>

#include "config.h"

// =============================================================================
//  timesync (pure-logger role) - PPS ticks + raw ToD, NO on-board time solution.
//
//  In this build the board computes no GNSS time. This module just:
//    - queues the tick of every PPS edge (from the capture ISR), and
//    - assembles raw NMEA ToD sentences from Serial2 with the tick they finished.
//  A host script pairs ToD<->PPS and fits tick->GNSS with a centered window.
// =============================================================================

namespace timesync {

	void begin();

	// PPS edge, queued from the capture ISR (GPIO in software mode, or the GPT1
	// vector in hardware-capture mode). Single producer context -> SPSC ring.
	void onPpsEvent(uint64_t tick);
	bool popPps(uint64_t &tick, uint32_t &seq);	 // drain one in loop(); false if empty

	// One raw NMEA sentence, assembled in loop() context.
	struct TodMsg {
		uint32_t seq;
		uint64_t tick;	// timebase tick when the terminator was read out
		uint16_t len;
		char nmea[100];	// the sentence, "$...*CS", no CR/LF, null-terminated
	};
	// Read available Serial2 bytes; returns true and fills `out` when a full
	// sentence is ready. Call repeatedly until it returns false.
	bool readTod(TodMsg &out);

	// Health / diagnostics (monotonic).
	uint32_t ppsRawCount();		   // PPS edges seen (== last PPS seq, includes dropped)
	uint32_t ppsDrops();		   // PPS ring overflow drops
	uint32_t todCount();		   // sentences completed
	uint32_t todBytes();		   // raw bytes read from Serial2
	double lastPpsIntervalSec();   // seconds between the last two PPS edges (0 if < 2 seen)

	void resetStats();

}  // namespace timesync
