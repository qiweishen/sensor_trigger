#pragma once
#include <Arduino.h>


// GPT1 free-running counter extended to 64 bit: the single clock for every log tick
namespace timebase {
	void begin();
	double hz();	 // nominal tick rate; the host fits the true rate from PPS
	uint64_t now();	 // loop or ISRs at or below GPT1 priority
}  // namespace timebase
