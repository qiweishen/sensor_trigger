#pragma once
#include <Arduino.h>

#include "config.h"

// =============================================================================
//  timesync - PPS ticks + raw ToD, no on-board time solution.
//
//  Queues the tick of every PPS edge and assembles raw NMEA sentences from
//  Serial2 with the tick they finished. Host pairs ToD<->PPS and fits offline.
// =============================================================================

namespace timesync {

	void begin();

	// PPS edge from the capture ISR (GPIO, or GPT1 vector in hw-capture mode)
	void onPpsEvent(uint64_t tick);
	bool popPps(uint64_t &tick, uint32_t &seq);	 // drain one in loop(); false if empty

	// one raw NMEA sentence, assembled in loop() context
	struct TodMsg {
		uint32_t seq;
		uint64_t tick;	// timebase tick at the terminator read-out
		char nmea[100];	// "$...*CS", no CR/LF, null-terminated
	};
	// reads available Serial2 bytes; true when a full sentence is ready;
	// call repeatedly until false
	bool readTod(TodMsg &out);

	// health (monotonic within a session)
	uint32_t ppsRawCount();		   // PPS edges seen (== last seq, incl. dropped)
	uint32_t ppsDrops();		   // PPS ring overflow drops
	uint32_t todCount();		   // sentences completed
	uint32_t todBytes();		   // raw bytes read
	uint32_t todTruncated();	   // oversize sentences discarded
	double lastPpsIntervalSec();   // last inter-PPS gap (0 until 2 edges seen)

	void resetStats();	 // full reset (counters + ring + assembly); only while paused
	void flushInput();	 // drop buffered ToD bytes -> clean session start

}  // namespace timesync
