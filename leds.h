#pragma once
#include <Arduino.h>


// activity LEDs (config.h LED_PINS), loop context only
namespace leds {
	void begin();
	void blink(uint8_t i);	// on for LED_HOLD_MS
	void poll(uint32_t ms);
	void off();
}  // namespace leds
