#pragma once
#include <cstdint>
inline uint32_t fake_primask = 0;
inline uint32_t __get_PRIMASK() { return fake_primask; }
inline void __set_PRIMASK(uint32_t value) { fake_primask = value; }
inline void __DMB() {}
inline void __DSB() {}
