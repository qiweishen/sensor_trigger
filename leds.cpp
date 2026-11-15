#include "leds.h"

#include "config.h"


namespace leds {
	static uint32_t g_since[6];
	static bool g_on[6];

	void begin() {
		for (uint8_t pin: LED_PINS) {
			digitalWriteFast(pin, LOW);
			pinMode(pin, OUTPUT);
		}
	}

	void blink(uint8_t i) {
		g_since[i] = millis();
		if (!g_on[i])
			digitalWriteFast(LED_PINS[i], HIGH);
		g_on[i] = true;
	}

	void poll(uint32_t ms) {
		for (uint8_t i = 0; i < 6; ++i)
			if (g_on[i] && ms - g_since[i] >= LED_HOLD_MS) {
				digitalWriteFast(LED_PINS[i], LOW);
				g_on[i] = false;
			}
	}

	void off() {
		for (uint8_t i = 0; i < 6; ++i) {
			digitalWriteFast(LED_PINS[i], LOW);
			g_on[i] = false;
		}
	}
}  // namespace leds
