#pragma once
#include <Arduino.h>


// Timestamped ISR events (PPS, trigger, exposure edges) -> one queue -> loop.
// Per-stream seq from 1; a dropped event leaves a gap in its stream.
namespace events {
	enum Kind : uint8_t { PPS, TRIG, STROBE };
	struct Event {
		Kind kind;
		uint8_t ch, level;
		uint32_t seq;
		uint64_t tick;
	};

	void begin();													 // PPS + exposure GPIO interrupts
	void capture(bool on);											 // gate PPS + exposure ISRs
	void reset();													 // counts + queue; only while capture is off
	void push(Kind kind, uint8_t ch, uint8_t level, uint64_t tick);	 // ISR context
	bool pop(Event &e);												 // loop context
	uint32_t count(Kind kind, uint8_t ch = 0);						 // events seen, incl. dropped
	uint32_t drops();
}  // namespace events
