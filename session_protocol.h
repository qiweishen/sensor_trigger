#pragma once
#include <charconv>
#include <cstdint>
#include <initializer_list>
#include <string_view>

// Version 2 live client. postprocess.py separately supports historical logs.
struct SensorSyncProtocol {
    unsigned starts = 0, truncations = 0;
    bool header = false, time_mode = false, ready = false;
    bool stop_received = false, final_health_received = false, failed = false;
    uint8_t cut_groups = 0;

    static bool integer(std::string_view text, std::uint64_t &value) {
        const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
        return parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size();
    }
    void observe(std::string_view line, bool stopping) {
        if (line.starts_with("#LOG,")) {
            header = line.starts_with("#LOG,SensorSync-logger,2,");
            if (!header) failed = true; // never reinterpret old per-camera commands
        } else if (line.starts_with("#TIME,")) {
            time_mode = line.find("T=PWM_ISR_sample,") != std::string_view::npos;
            if (!time_mode) failed = true;
        } else if (line == "#READY") {
            if (ready || stopping || starts != 1 || !header || !time_mode) failed = true;
            ready = true;
        } else if (line.starts_with("#SESSION,START,")) {
            if (++starts != 1 || stopping) failed = true;
        } else if (line.starts_with("#SESSION,STOP,")) {
            stop_received = true;
            if (!stopping || !final_health_received) failed = true;
        } else if (line.starts_with("#ERR") || (line.starts_with("#IDLE") && !stopping)) {
            failed = true;
        } else if (line.starts_with("#TRUNC,")) {
            line.remove_prefix(7);
            std::uint64_t values[3] = {};
            for (unsigned i = 0; i < 3; ++i) {
                const auto end = line.find(',');
                if ((i < 2) != (end != std::string_view::npos) || !integer(line.substr(0, end), values[i])) {
                    failed = true; return;
                }
                if (i < 2) line.remove_prefix(end + 1);
            }
            if (!stopping || values[0] >= 2 || !(values[1] & 1) || !values[2] ||
                (cut_groups & (1u << values[0]))) { failed = true; return; }
            cut_groups |= 1u << values[0];
            ++truncations;
            return;
        }
        const bool final = line.starts_with("#HFINAL ");
        if (final) {
            final_health_received = true;
            if (!stopping || !ready) failed = true;
        }
        if (line.starts_with("#H ") || final || line.starts_with("#Ht ")) {
            for (const auto key : {"ppsdrops=", "todtrunc=", "tdrops=", "sdrops=", "host=", "pwmerr=", "trunc="}) {
                auto pos = line.find(key);
                // Match whole health tokens: trunc= must not match todtrunc=.
                while (pos != std::string_view::npos && pos > 0 && line[pos - 1] != ' ')
                    pos = line.find(key, pos + 1);
                if (pos == std::string_view::npos) { if (final) failed = true; continue; }
                pos += std::string_view(key).size();
                const auto end = line.find(' ', pos);
                std::uint64_t value = 0;
                if (!integer(line.substr(pos, end == std::string_view::npos ? end : end - pos), value)) {
                    failed = true; continue;
                }
                if (std::string_view(key) == "trunc=") {
                    if (final && value != truncations) failed = true;
                } else if (value != 0) failed = true;
            }
        }
    }
};
