#pragma once
#include <cstdint>
#include <cstdio>
#include <vector>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLDOWN 2
#define INPUT_PULLUP 3
#define CHANGE 4
#define CORE_NUM_DIGITAL 55
#define IRQ_GPIO6789 0
inline uint32_t GPT1_OCR1 = 0, GPT1_IR = 0, GPT1_SR = 0, GPT1_CNT = 0;
inline uint64_t fake_tick = 0, now_step = 25;
inline int modes[CORE_NUM_DIGITAL] = {}, levels[CORE_NUM_DIGITAL] = {};
inline void (*callbacks[CORE_NUM_DIGITAL])() = {};
struct PinEdge { uint8_t pin, level; uint64_t tick; };
inline std::vector<PinEdge> pin_edges;
inline void set_tick(uint64_t tick) { fake_tick = tick; GPT1_CNT = static_cast<uint32_t>(tick); }
inline void digitalWriteFast(uint8_t pin, uint8_t level) {
    if (levels[pin] != level) pin_edges.push_back({pin, level, fake_tick});
    levels[pin] = level;
}
inline int digitalReadFast(uint8_t pin) { return levels[pin]; }
inline void pinMode(uint8_t pin, int mode) { modes[pin] = mode; }
inline int digitalPinToInterrupt(uint8_t pin) { return pin; }
inline void attachInterrupt(int pin, void (*fn)(), int) { callbacks[pin] = fn; }
inline void __disable_irq() {}
inline void __enable_irq() {}
inline void NVIC_SET_PRIORITY(int, int) {}
struct FakeSerial { template<class... T> void printf(const char*, T...) {} };
inline FakeSerial Serial;
