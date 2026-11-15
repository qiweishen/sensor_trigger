#pragma once
#include <Arduino.h>


// Hardware PWM trigger pulses; PWM ISR logs each rising edge as a T event.
// Group 0 = FX (ch 0/1, FlexPWM2 SM0), group 1 = JAI (ch 2/3, SM3).
namespace triggers {
	struct Timing {
		double actual_hz = 0, pulse_us = 0, clock_hz = 0;
		uint16_t period = 0, width = 0;	 // PWM clocks
		uint8_t prescale = 0;			 // CTRL[PRSC] exponent
	};

	void begin();
	bool setRates(double fx_hz, double jai_hz);	 // false: not representable; 0 = pair off
	double requested(uint8_t group);
	const Timing &timing(uint8_t group);
	double freqHz(uint8_t ch);					 // 0 for a disabled output or pair
	void start();
	void stop();								 // all trigger pins LOW
}  // namespace triggers
