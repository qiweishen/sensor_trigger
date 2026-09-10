#pragma once

#include <charconv>
#include <cstdint>
#include <initializer_list>
#include <string_view>

// Pre-session synchronization only. An unknown command with a fresh token is
// a round-trip barrier supported by existing firmware. Old replies (including
// errors from a residual half-line) belong to diagnostics, not capture data.
struct SensorSyncStartupProtocol {
    std::string_view barrier_reply;
    std::string_view completion_reply;
    bool barrier_received = false;
    bool idle_received = false;
    bool completed = false;
    bool failed = false;

    void observe(std::string_view line) {
        if (line.empty()) return;
        if (!barrier_received) {
            if (line == barrier_reply) barrier_received = true;
            return;
        }
        if (line == completion_reply) {
            if (!idle_received || completed) failed = true;
            else completed = true;
            return;
        }
        constexpr std::string_view prefix = "#IDLE,up=";
        if (line.starts_with(prefix)) {
            const auto value = line.substr(prefix.size());
            std::uint64_t uptime = 0;
            const auto parsed = std::from_chars(value.data(), value.data() + value.size(), uptime);
            if (parsed.ec == std::errc{} && parsed.ptr == value.data() + value.size()) {
                idle_received = true;
                return;
            }
        }
        failed = true; // only idle replies and the final round-trip marker are acceptable
    }
};

// Host-side integrity checks for the existing SensorSync text protocol.
// This tracks loss/restarts, not a GPS/UTC solution or PPS-to-ToD association.
struct SensorSyncProtocol {
    unsigned starts = 0;
    unsigned version = 1; // legacy log checks; live v2 acquisition requires READY
    bool header_received = false;
    bool ready = false;
    bool stop_received = false;
    bool final_health_received = false;
    bool failed = false;
    unsigned truncations = 0;
    unsigned truncation_mask = 0;

    static bool number(std::string_view text, std::uint64_t &value) {
        const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
        return parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size();
    }

    static bool counter(std::string_view line, std::string_view key, bool required, std::uint64_t &value) {
        std::size_t pos = 0;
        while ((pos = line.find(key, pos)) != std::string_view::npos) {
            if (pos > 0 && line[pos - 1] != ' ') { ++pos; continue; }
            pos += key.size();
            const auto end = line.find(' ', pos);
            return number(line.substr(pos, end == std::string_view::npos ? end : end - pos), value);
        }
        return !required;
    }

    void observe(std::string_view line, bool stopping) {
        if (line.starts_with("#LOG,") && line.ends_with(",cap=0")) failed = true;
        if (line.starts_with("#LOG,")) {
            constexpr std::string_view prefix = "#LOG,SensorSync-logger,";
            if (!line.starts_with(prefix) || header_received || starts != 1 || stopping) {
                failed = true;
                return;
            }
            const auto fields = line.substr(prefix.size());
            std::uint64_t v = 0;
            if (!number(fields.substr(0, fields.find(',')), v) || (v != 1 && v != 2)) {
                failed = true;
                return;
            }
            version = static_cast<unsigned>(v);
            header_received = true;
        }
        if (line.starts_with("#SESSION,START,")) {
            if (++starts != 1 || stopping) failed = true;
        } else if (line == "#READY") {
            if (version != 2 || !header_received || starts != 1 || ready || stopping || final_health_received)
                failed = true;
            else ready = true;
        } else if (line.starts_with("#SESSION,STOP,")) {
            stop_received = true;
            if (!stopping || !final_health_received || (version == 2 && !ready)) failed = true;
        } else if (line.starts_with("#ERR") || (line.starts_with("#IDLE") && !stopping)) {
            failed = true;
        }
        if (line.starts_with("#TRUNC,")) {
            // Stopping during HIGH is an audited cut, not a fabricated falling
            // edge or PWM event loss. It must agree with the final trunc count.
            if (version != 2 || !stopping || !ready || final_health_received) { failed = true; return; }
            auto fields = line.substr(7);
            std::uint64_t values[3]{};
            for (unsigned i = 0; i < 3; ++i) {
                const auto end = fields.find(',');
                if ((i < 2 && end == std::string_view::npos) || (i == 2 && end != std::string_view::npos) ||
                    !number(fields.substr(0, end), values[i])) { failed = true; return; }
                if (i < 2) fields.remove_prefix(end + 1);
            }
            const auto group = values[0], seq = values[1];
            if (group >= 2 || seq == 0 || !(seq & 1) || (truncation_mask & (1u << group))) {
                failed = true;
                return;
            }
            truncation_mask |= 1u << group;
            ++truncations;
        }
        const bool final = line.starts_with("#HFINAL ");
        if (final) {
            if (final_health_received || !stopping) failed = true;
            final_health_received = true;
        }
        if (line.starts_with("#H ") || final || line.starts_with("#Ht ")) {
            const auto require_zero = [&](std::string_view key) {
                std::uint64_t value = 0;
                if (!counter(line, key, final, value) || value != 0) failed = true;
            };
            for (const auto key : {"ppsdrops=", "todtrunc=", "tdrops=", "sdrops=", "host="}) require_zero(key);
            if (version == 2) {
                require_zero("pwmerr=");
                if (final) {
                    std::uint64_t cuts = 0;
                    if (!counter(line, "trunc=", true, cuts) || cuts != truncations) failed = true;
                }
            } else {
                require_zero("skip=");
                require_zero("stall=");
            }
        }
    }
};
