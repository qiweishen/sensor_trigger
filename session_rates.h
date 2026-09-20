#pragma once

#include <charconv>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "trigger_groups.h"

namespace sensorsync {
	// Preserve physical channel IDs in the C++ API, but encode the firmware's PWM
	// groups on the wire. Channels 0/1 share FX; channels 2/3 share JAI. Conflicting
	// rates within one pair cannot be implemented by the hardware and must fail.
	inline bool BuildFrequencySpec(const std::vector<std::pair<int, double>> &channels, std::string &spec, std::string &error) {
		spec.clear();
		error.clear();
		double rates[trigger::GROUP_COUNT]{};
		bool seen[trigger::GROUP_COUNT]{};
		for (const auto &[channel, hz]: channels) {
			if (channel < 0 || channel >= 4) {
				error = "trigger channel must be in [0, 3]";
				return false;
			}
			const auto group = static_cast<trigger::Group>(channel / 2);
			if (!trigger::validRate(group, hz)) {
				error = std::string(trigger::name(group)) +
						(group == trigger::Group::FX ? " rate must be 0 or finite >= 20 Hz" : " rate must be 0 or within [1, 10] Hz");
				return false;
			}
			const unsigned index = static_cast<unsigned>(group);
			if (seen[index] && rates[index] != hz) {
				error = std::string("conflicting rates for paired ") + trigger::name(group) + " outputs";
				return false;
			}
			seen[index] = true;
			rates[index] = hz;
		}
		std::string encoded;
		for (unsigned g = 0; g < trigger::GROUP_COUNT; ++g) {
			if (!seen[g])
				continue;
			char value[64];
			const auto converted = std::to_chars(value, value + sizeof(value), rates[g] == 0 ? 0.0 : rates[g],
												 std::chars_format::general, std::numeric_limits<double>::max_digits10);
			if (converted.ec != std::errc{}) {
				error = "cannot encode trigger frequency";
				return false;
			}
			if (!encoded.empty())
				encoded += ',';
			encoded += trigger::name(static_cast<trigger::Group>(g));
			encoded += ':';
			encoded.append(value, converted.ptr);
		}
		spec = std::move(encoded);
		return true;
	}
}  // namespace sensorsync
