#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "cmsis_gcc.h"
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLDOWN 2
#define INPUT_PULLUP 3
#define CHANGE 4
#define RISING 5
#define FALLING 6
#define CORE_NUM_DIGITAL 55
#define IRQ_GPIO6789 0
#define IRQ_FLEXPWM2_0 1
#define IRQ_FLEXPWM2_3 2
inline uint32_t F_BUS_ACTUAL = 150000000;
inline uint32_t GPT1_OCR1 = 0, GPT1_IR = 0, GPT1_SR = 0, GPT1_CNT = 0;
inline uint64_t fake_tick = 0, now_step = 25;
inline uint32_t fake_ms = 0;
inline uint32_t millis_step = 0;
inline uint32_t millis() {
	fake_ms += millis_step;
	return fake_ms;
}
inline void (*delay_hook)() = nullptr;
inline void delay(uint32_t ms) {
	fake_ms += ms;
	if (delay_hook)
		delay_hook();
}
inline uint32_t muxes[CORE_NUM_DIGITAL] = {};
inline int gpio_latches[CORE_NUM_DIGITAL] = {};
inline uint32_t *portConfigRegister(uint8_t pin) {
	return &muxes[pin];
}
inline int modes[CORE_NUM_DIGITAL] = {}, levels[CORE_NUM_DIGITAL] = {};
inline void (*callbacks[CORE_NUM_DIGITAL])() = {};
struct PinEdge {
	uint8_t pin, level;
	uint64_t tick;
};
inline std::vector<PinEdge> pin_edges;
inline void set_tick(uint64_t tick) {
	fake_tick = tick;
	GPT1_CNT = static_cast<uint32_t>(tick);
}
inline void digitalWriteFast(uint8_t pin, uint8_t level) {
	gpio_latches[pin] = level;
	if (muxes[pin] != 0 && muxes[pin] != 5)
		return;
	if (levels[pin] != level)
		pin_edges.push_back({ pin, level, fake_tick });
	levels[pin] = level;
}
inline int digitalReadFast(uint8_t pin) {
	return levels[pin];
}
inline void pinMode(uint8_t pin, int mode) {
	modes[pin] = mode;
	muxes[pin] = 5;
	if (mode == OUTPUT)
		digitalWriteFast(pin, gpio_latches[pin]);
}
inline int digitalPinToInterrupt(uint8_t pin) {
	return pin;
}
inline void attachInterrupt(int pin, void (*fn)(), int) {
	callbacks[pin] = fn;
}
inline void __disable_irq() {
	fake_primask = 1;
}
inline void __enable_irq() {
	fake_primask = 0;
}
inline void (*irq_vectors[3])() = {};
inline int irq_priority[3] = {};
inline bool irq_enabled[3] = {};
inline void NVIC_SET_PRIORITY(int irq, int pri) {
	irq_priority[irq] = pri;
}
inline void NVIC_ENABLE_IRQ(int irq) {
	irq_enabled[irq] = true;
}
inline void NVIC_CLEAR_PENDING(int) {}
inline void attachInterruptVector(int irq, void (*fn)()) {
	irq_vectors[irq] = fn;
}
struct FakeSerial {
	std::string output, input;
	int space = 4096;
	operator bool() const { return true; }
	void begin(unsigned) {}
	int availableForWrite() const { return space; }
	int available() const { return int(input.size()); }
	int read() {
		char c = input[0];
		input.erase(0, 1);
		return c;
	}
	void write(const uint8_t *data, size_t n) { output.append(reinterpret_cast<const char *>(data), n); }
	void println(const char *line) {
		output += line;
		output += '\n';
	}
	template<class... T>
	void printf(const char *format, T... args) {
		char text[1024];
		const int n = std::snprintf(text, sizeof(text), format, args...);
		if (n > 0)
			output.append(text, std::min<size_t>(n, sizeof(text) - 1));
	}
};
inline FakeSerial Serial;

#include "pwm_registers.h"
