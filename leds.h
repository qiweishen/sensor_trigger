#pragma once

// =============================================================================
//  leds - six active-high activity indicators, serviced only in loop context.
//  Existing counters supply activity; no LED work is added to the capture ISRs.
// =============================================================================

namespace leds {

	void begin();
	void reset();  // all off; call on STOP and after clearing counters for START
	void poll();   // call only during a session; each counter change extends the hold

}  // namespace leds
