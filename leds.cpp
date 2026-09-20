#include "leds.h"

#include "channels.h"
#include "config.h"
#include "timesync.h"

namespace leds {

	static_assert(LED_MAX == 6, "activity mapping requires the six schematic LEDs");
	static_assert(LED_HOLD_MS > 0 && LED_HOLD_MS < 0x80000000UL, "LED hold must be a positive millis interval");
	static_assert(
			[] {
				for (uint8_t i = 0; i < LED_MAX; ++i) {
					const uint8_t pin = LED_PINS[i];
					if (pin >= CORE_NUM_DIGITAL || pin == 0 || pin == 1 || pin == PPS_PIN)
						return false;
					for (uint8_t j = 0; j < i; ++j)
						if (pin == LED_PINS[j])
							return false;
					for (uint8_t j = 0; j < TRIG_MAX; ++j)
						if (pin == TRIG_CFG[j].pin)
							return false;
					for (uint8_t j = 0; j < STROBE_MAX; ++j)
						if (pin == STROBE_CFG[j].pin)
							return false;
				}
				return true;
			}(),
			"LED pins must be unique and must not overlap Serial1, PPS, triggers or strobes");


	struct LedState {
		uint32_t seen;
		uint32_t last_ms;
		bool on;
	};
	static LedState g_led[LED_MAX];


	void reset() {
		for (uint8_t i = 0; i < LED_MAX; ++i) {
			g_led[i] = {};
			digitalWriteFast(LED_PINS[i], LOW);
		}
	}

	void begin() {
		reset();  // preload LOW before enabling output drivers
		for (uint8_t i = 0; i < LED_MAX; ++i)
			pinMode(LED_PINS[i], OUTPUT);
	}


	void poll() {
		// Counts include queue/output drops. A static pulled-up exposure input is
		// not activity. Both edge polarities count; this is not an exposure level.
		const uint32_t counts[] = { timesync::ppsRawCount(),
									timesync::todBytes(),
									channels::trigCount(0) + channels::trigCount(1),
									channels::strobeCount(0) + channels::strobeCount(1),
									channels::trigCount(2) + channels::trigCount(3),
									channels::strobeCount(2) + channels::strobeCount(3) };
		const uint32_t now = millis();
		for (uint8_t i = 0; i < LED_MAX; ++i) {
			LedState &s = g_led[i];
			const bool changed = counts[i] != s.seen;
			s.seen = counts[i];
			if (changed)
				s.last_ms = now;
			// Unsigned subtraction keeps the hold correct across millis() rollover.
			const bool on = changed || (s.on && static_cast<uint32_t>(now - s.last_ms) < LED_HOLD_MS);
			if (on != s.on) {
				digitalWriteFast(LED_PINS[i], on ? HIGH : LOW);
				s.on = on;
			}
		}
	}

}  // namespace leds
