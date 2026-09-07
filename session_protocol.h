#pragma once

#include <charconv>
#include <cstdint>
#include <initializer_list>
#include <string_view>

// Host-side integrity checks for the existing SensorSync text protocol.
// This tracks loss/restarts, not a GPS/UTC solution or PPS-to-ToD association.
struct SensorSyncProtocol {
    unsigned starts = 0;
    bool stop_received = false;
    bool final_health_received = false;
    bool failed = false;

    void observe(std::string_view line, bool stopping) {
        if (line.starts_with("#LOG,") && line.ends_with(",cap=0")) failed = true;
        if (line.starts_with("#SESSION,START,")) {
            if (++starts != 1 || stopping) failed = true;
        } else if (line.starts_with("#SESSION,STOP,")) {
            stop_received = true;
            if (!stopping || !final_health_received) failed = true;
        } else if (line.starts_with("#ERR") || (line.starts_with("#IDLE") && !stopping)) {
            failed = true;
        }
        if (line.starts_with("#HFINAL ")) {
            final_health_received = true;
            if (!stopping) failed = true;
        }
        if (line.starts_with("#H ") || line.starts_with("#HFINAL ") || line.starts_with("#Ht ")) {
            for (const auto key : {"ppsdrops=", "todtrunc=", "tdrops=", "sdrops=", "host=", "skip=", "stall="}) {
                auto pos = line.find(key);
                if (pos == std::string_view::npos) {
                    if (line.starts_with("#HFINAL ")) failed = true;
                    continue;
                }
                pos += std::string_view(key).size();
                const auto end = line.find(' ', pos);
                const auto value = line.substr(pos, end == std::string_view::npos ? end : end - pos);
                std::uint64_t count = 0;
                const auto parsed = std::from_chars(value.data(), value.data() + value.size(), count);
                if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() || count != 0) failed = true;
            }
        }
    }
};
