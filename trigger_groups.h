#pragma once
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>


// Shared by firmware and host. Numeric camera IDs remain log IDs, never groups.
namespace trigger {
	enum class Group : uint8_t { FX, JAI };
	constexpr unsigned GROUP_COUNT = 2;
	inline const char *name(Group g) {
		return g == Group::FX ? "FX" : "JAI";
	}
	struct Rates {
		double hz[GROUP_COUNT] = { 50.0, 0.0 };
	};
	inline bool validRate(Group g, double hz) {
		return std::isfinite(hz) && (hz == 0.0 || (g == Group::FX ? hz >= 20.0 : hz >= 1.0 && hz <= 10.0));
	}

	// Atomic parse: omitted groups retain their prior rate; duplicate/empty items fail.
	// A null spec means no override. An explicitly empty freq= is an error.
	inline bool parseRates(const char *spec, Rates &rates, uint8_t *fields = nullptr) {
		if (!spec) {
			if (fields) {
				*fields = 0;
			}
			return true;
		}
		if (!*spec) {
			return false;
		}
		Rates next = rates;
		uint8_t seen = 0;
		const char *p = spec;
		while (*p) {
			unsigned g;
			if (std::strncmp(p, "FX:", 3) == 0) {
				g = 0;
				p += 3;
			} else if (std::strncmp(p, "JAI:", 4) == 0) {
				g = 1;
				p += 4;
			} else {
				return false;
			}
			if (seen & (1u << g)) {
				return false;
			}
			if (!((*p >= '0' && *p <= '9') || *p == '.')) {
				return false;
			}
			char *end = nullptr;
			const double hz = std::strtod(p, &end);
			if (end == p || (*end && *end != ',') || !validRate(static_cast<Group>(g), hz)) {
				return false;
			}
			next.hz[g] = hz;
			seen |= 1u << g;
			if (!*end) {
				break;
			}
			p = end + 1;
			if (!*p) {
				return false;
			}
		}
		rates = next;
		if (fields) {
			*fields = seen;
		}
		return true;
	}

	struct Timing {
		uint16_t period = 0, width = 0;
		uint8_t prescale = 0;  // CTRL[PRSC] exponent, not divisor
		double clock_hz = 0, actual_hz = 0, pulse_us = 0;
	};

	inline bool calculate(Group group, double hz, uint32_t pulse_us, double bus_hz, Timing &out) {
		if (!validRate(group, hz) || !std::isfinite(bus_hz) || bus_hz <= 0 || pulse_us == 0) {
			return false;
		}
		Timing t;
		// SM0 always /128. SM3 further divides the fixed SM0 AUX_CLK.
		t.clock_hz = bus_hz / 128.0;
		t.prescale = group == Group::FX ? 7 : 0;
		// A disabled SM0 still needs valid register values when supplying JAI's clock.
		const double rate = hz == 0 ? (group == Group::FX ? 50.0 : 1.0) : hz;
		if (group == Group::JAI) {
			while (std::ceil(t.clock_hz / rate) > 65535.0 && t.prescale < 7) {
				++t.prescale;
				t.clock_hz /= 2.0;
			}
		}
		const double n = std::ceil(t.clock_hz / rate);
		const double w = std::ceil(t.clock_hz * pulse_us / 1e6);
		// Rise at W and fall at 2W leave a defined low startup interval.
		if (n > 65535 || w < 1 || n <= 2 * w + 1) {
			return false;
		}
		t.period = static_cast<uint16_t>(n);
		t.width = static_cast<uint16_t>(w);
		t.actual_hz = hz == 0 ? 0 : t.clock_hz / n;
		t.pulse_us = 1e6 * w / t.clock_hz;
		out = t;
		return true;
	}
}  // namespace trigger
