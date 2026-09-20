#pragma once
#include <Arduino.h>

#include "config.h"

// T = PWM handler GPT1 sample; S = GPIO ISR GPT1 sample. Neither is a latch.
namespace channels {
	void begin();
	bool validateRates(const trigger::Rates &rates);
	bool applyRates(const trigger::Rates &rates);  // atomic, only while idle
	trigger::Rates requestedRates();
	const trigger::Timing &timing(uint8_t group);
	double freqHz(uint8_t ch);
	// STOP first disables PWM with retain_strobes=true, then closes GPIO capture
	// after the bounded exposure tail. Ordinary disable still stops both.
	bool setEnabled(bool on, bool retain_strobes = false);
	uint8_t pendingStrobes();  // observed active exposures, never disconnected idle pins
	bool enabled();
	bool faulted();
	void pollHealth();
	void resetCounts();	 // only while idle
	bool popTrig(uint8_t &ch, uint8_t &level, uint64_t &tick, uint32_t &seq);
	bool popStrobe(uint8_t &ch, uint8_t &level, uint64_t &tick, uint32_t &seq);
	uint32_t trigCount(uint8_t ch);
	uint32_t strobeCount(uint8_t ch);
	uint32_t pwmErrors(uint8_t group);
	uint32_t trigDrops();
	uint32_t strobeDrops();
	bool truncation(uint8_t group, uint32_t &rise_seq, uint64_t &stop_tick);
}  // namespace channels
