// =============================================================================
//  SensorSync-Logger - Teensy 4.1
//
//  Raw event logger for post-processed GNSS timing. No on-board time solution:
//  the host pairs ToD<->PPS offline and fits tick->GNSS with a centered window.
//
//  Log lines ('#' = comment / status):
//    #SESSION,START,<path>                         session begin (path echo only)
//    #LOG,SensorSync-logger,2,tick_hz=<f>          header
//    #TRIG,<i>,<name>,<pin>,<freq>,ah=<0|1>        per trigger channel
//    #STROBE,<i>,<name>,<pin>,ah=<0|1>             per strobe channel
//    P,<seq>,<tick>                                PPS edge
//    T,<seq>,<ch>,<level>,<tick>                   trigger PWM handler sample
//    S,<seq>,<ch>,<level>,<tick>                   strobe edge
//    Z,<seq>,<tick>,<raw NMEA sentence>            ToD sentence + read-out tick
//    #H / #Ht / #Hs                                health, every 5 s while running
//    #SESSION,STOP,<path>                          session end
//    #IDLE,up=<ms>                                 heartbeat while idle
//  Per-stream <seq> starts at 1 each session; advances even on drop -> gaps visible.
// =============================================================================

#include "channels.h"
#include "config.h"
#include "leds.h"
#include "timebase.h"
#include "timesync.h"


// ---------------------------------------------------------------------------
// Session state: idle after boot; logs only between START and STOP.
// START zeroes all counters -> every stream counts from 0 per session.
static volatile bool g_running = false;
static char g_session_path[224] = { 0 };  // >= max cmd payload; no truncation


// ---------------------------------------------------------------------------
// PPS input

static void ppsIsr() {
	if (g_running) {  // gate: no counting while idle
		timesync::onPpsEvent(timebase::now());
	}
}

// ---------------------------------------------------------------------------
// Output discipline: never block loop() on USB CDC (blocks ~120 ms with an
// attached-but-not-reading host). Data lines: drop + count. Session-boundary
// lines: bounded wait, then drop + count.
static uint32_t g_host_drops = 0;
static constexpr int kLineMax = 160;  // line buffer size everywhere below

static void emit(const char *line, int n) {
	if (n <= 0) {
		return;
	}
	if (n >= kLineMax) {  // snprintf overflow: clamp to what's in the buffer
		n = kLineMax - 1;
	}
	if (Serial.availableForWrite() >= n) {
		Serial.write(reinterpret_cast<const uint8_t *>(line), static_cast<size_t>(n));
	} else {
		g_host_drops++;
	}
}

// bounded wait for TX room; false (+count) on timeout
static bool waitTx(int bytes, uint32_t timeout_ms) {
	const uint32_t t0 = millis();
	while (Serial.availableForWrite() < bytes) {
		if (millis() - t0 >= timeout_ms) {
			g_host_drops++;
			return false;
		}
	}
	return true;
}


// ---------------------------------------------------------------------------
static void printHealth(bool final) {
    if (Serial.availableForWrite() < 512) return;
    const uint32_t errors = channels::pwmErrors(0) + channels::pwmErrors(1);
    uint32_t seq; uint64_t tick;
    const unsigned cuts = unsigned(channels::truncation(0, seq, tick)) + unsigned(channels::truncation(1, seq, tick));
    Serial.printf(final
        ? "#HFINAL up=%lu en=%d pps=%lu ppsdt=%.6f ppsdrops=%lu tod=%lu todb=%lu todtrunc=%lu tdrops=%lu sdrops=%lu host=%lu pwmerr=%lu trunc=%u\n"
        : "#H up=%lu en=%d pps=%lu ppsdt=%.6f ppsdrops=%lu tod=%lu todb=%lu todtrunc=%lu tdrops=%lu sdrops=%lu host=%lu pwmerr=%lu trunc=%u\n",
        millis(), channels::enabled() ? 1 : 0, timesync::ppsRawCount(), timesync::lastPpsIntervalSec(),
        timesync::ppsDrops(), timesync::todCount(), timesync::todBytes(), timesync::todTruncated(),
        channels::trigDrops(), channels::strobeDrops(), static_cast<unsigned long>(g_host_drops),
        static_cast<unsigned long>(errors), cuts);
    if (final) return;
    for (uint8_t i = 0; i < TRIG_MAX; ++i) {
        if (Serial.availableForWrite() < 128) return;
        Serial.printf("#Ht %u %s n=%lu pwmerr=%lu\n", i, TRIG_CFG[i].name,
                      channels::trigCount(i), channels::pwmErrors(i / 2));
    }
    for (uint8_t i = 0; i < STROBE_MAX; ++i) {
        if (Serial.availableForWrite() < 128) return;
        Serial.printf("#Hs %u %s n=%lu\n", i, STROBE_CFG[i].name, channels::strobeCount(i));
    }
}

static bool printHeader() {
    if (!waitTx(128, 250)) return false;
    Serial.printf("#LOG,SensorSync-logger,2,tick_hz=%.3f\n", timebase::nominalHz());
    const auto requested = channels::requestedRates();
    for (uint8_t g = 0; g < 2; ++g) {
        if (!waitTx(192, 250)) return false;
        const auto &t = channels::timing(g);
        Serial.printf("#GROUP,%s,requested=%.8f,actual=%.8f,pulse_us=%.6f,clock_hz=%.6f,div=%u,period=%u,width=%u\n",
                      trigger::name(static_cast<trigger::Group>(g)), requested.hz[g], t.actual_hz,
                      t.pulse_us, t.clock_hz, 1u << t.prescale, t.period, t.width);
    }
    for (uint8_t i = 0; i < TRIG_MAX; ++i) {
        if (!waitTx(128, 250)) return false;
        Serial.printf("#TRIG,%u,%s,%u,%.8f,ah=%u\n", i, TRIG_CFG[i].name, TRIG_CFG[i].pin,
                      channels::freqHz(i), TRIG_CFG[i].active_high ? 1 : 0);
    }
    for (uint8_t i = 0; i < STROBE_MAX; ++i) {
        if (!waitTx(128, 250)) return false;
        Serial.printf("#STROBE,%u,%s,%u,ah=%u\n", i, STROBE_CFG[i].name, STROBE_CFG[i].pin,
                      STROBE_CFG[i].active_high ? 1 : 0);
    }
    if (!waitTx(128, 250)) return false;
    Serial.println("#TIME,T=PWM_ISR_sample,S=GPIO_ISR_sample,Z=sentence_read,P=GPIO_ISR_sample");
    return true;
}


// idle heartbeat
static void printIdle() {
	if (Serial.availableForWrite() < 128) {
		return;
	}
	Serial.printf("#IDLE,up=%lu\n", (unsigned long) millis());
}


// ---------------------------------------------------------------------------
// Drain every queued event through emit() before a session boundary, so the
// tail of a session is not wiped with the rings. Producers must already be
// gated off (g_running false + channels disabled). Bounded by `deadline_ms`.
static void drainAll(uint32_t deadline_ms) {
	char buf[kLineMax];
	const uint32_t t0 = millis();
	bool more = true;
	while (more && (millis() - t0) < deadline_ms) {
		more = false;
		uint64_t tick;
		uint32_t seq;
		uint8_t ch, level;
		if (timesync::popPps(tick, seq)) {
			more = true;
			if (waitTx(64, 50)) {
				emit(buf, snprintf(buf, sizeof(buf), "P,%lu,%llu\n", (unsigned long) seq, (unsigned long long) tick));
			}
		}
		if (channels::popTrig(ch, level, tick, seq)) {
			more = true;
			if (waitTx(64, 50)) {
				emit(buf, snprintf(buf, sizeof(buf), "T,%lu,%u,%u,%llu\n", (unsigned long) seq, ch, level, (unsigned long long) tick));
			}
		}
		if (channels::popStrobe(ch, level, tick, seq)) {
			more = true;
			if (waitTx(64, 50)) {
				emit(buf, snprintf(buf, sizeof(buf), "S,%lu,%u,%u,%llu\n", (unsigned long) seq, ch, level, (unsigned long long) tick));
			}
		}
		timesync::TodMsg m;
		if (timesync::readTod(m)) {
			more = true;
			if (waitTx(160, 50)) {
				emit(buf, snprintf(buf, sizeof(buf), "Z,%lu,%llu,%s\n", (unsigned long) m.seq, (unsigned long long) m.tick, m.nmea));
			}
		}
	}
	if (more) ++g_host_drops; // deadline prevented confirmation that the tail was drained
}


// ---------------------------------------------------------------------------
// Session control (host-facing API).
//   START [freq=FX:<hz>,JAI:<hz>] [path]   zero all counts, optionally retune
//                 trigger rates, enable capture, emit #SESSION,START + header
//   STOP          drain tail, emit #SESSION,STOP, park idle, clear counts
// <path> is echo only (board cannot write the host filesystem); it makes the
// log self-document where the host saved it. freq= overrides persist until the
// next override or reboot; the #TRIG header always shows the effective rate.
static void closeRunningSession() {
	g_running = false;
	channels::setEnabled(false);
	leds::reset();
    // STOP may truncate a high pulse; no synthetic T falling edge is emitted.
    for (uint8_t g = 0; g < 2; ++g) {
        uint32_t seq; uint64_t tick;
        if (channels::truncation(g, seq, tick) && waitTx(128, 250))
            Serial.printf("#TRUNC,%u,%lu,%llu\n", g, static_cast<unsigned long>(seq),
                          static_cast<unsigned long long>(tick));
    }
    drainAll(500); // flush the session tail before the marker
	// Preserve final loss counters BEFORE resetStats/resetCounts. A normal STOP
	// marker is emitted only after the final health block could be sent.
	const bool final_stats_sent = waitTx(512, 250);
	if (final_stats_sent) printHealth(true);
	if (final_stats_sent && waitTx(512, 250)) {	 // marker line can exceed kLineMax (long path)
		Serial.printf("#SESSION,STOP,%s\n", g_session_path);
	}
	// all-zero while idle; next START counts from 0
	timesync::flushInput();
	timesync::resetStats();
	channels::resetCounts();
	g_host_drops = 0;
	g_session_path[0] = 0;
}


static void stopSession() {
	if (!g_running) {  // idle STOP: ack, no phantom marker
		printIdle();
		return;
	}
	closeRunningSession();
}


// Validate the entire request before closing an existing session or changing rates.
static void startSession(const char *path, const char *freqspec = nullptr) {
    auto rates = channels::requestedRates();
    if (!trigger::parseRates(freqspec, rates) || !channels::validateRates(rates)) {
        if (waitTx(64, 50)) Serial.println("#ERR,bad_freq");
        return;
    }
    if (g_running) closeRunningSession();
    timesync::flushInput();
    timesync::resetStats();
    channels::resetCounts();
    g_host_drops = 0;
    leds::reset();
    if (!channels::applyRates(rates)) {
        if (waitTx(64, 50)) Serial.println("#ERR,pwm_config");
        return;
    }
    strncpy(g_session_path, path ? path : "", sizeof(g_session_path) - 1);
    g_session_path[sizeof(g_session_path) - 1] = 0;
    if (!waitTx(512, 250)) return;
    Serial.printf("#SESSION,START,%s\n", g_session_path);
    if (!printHeader() || !waitTx(64, 250)) return; // no output without a complete header
    g_running = true;
    if (!channels::setEnabled(true)) {
        if (waitTx(64, 50)) Serial.println("#ERR,pwm_start");
        closeRunningSession();
        return;
    }
    // Version 2 clients acknowledge only READY, after the hardware actually starts.
    Serial.println("#READY");
}


// Line commands. Host: "START <path>" / "STOP". Human: s / h also work.
// Keywords are token-matched: next char must be end-of-line or whitespace,
// so a garbled line ("STOPPED", "STARTx") cannot toggle session state.
static void handleCommand(char *s) {
	while (*s == ' ' || *s == '\t') {
		s++;
	}
	const auto boundary = [](char c) { return c == '\0' || c == ' ' || c == '\t'; };
	if (strncmp(s, "START", 5) == 0 && boundary(s[5])) {
		char *p = s + 5;
		while (*p == ' ' || *p == '\t') {
			p++;
		}
		// optional leading option: "freq=FX:<hz>,JAI:<hz>", then the path
		char *freq = nullptr;
		if (strncmp(p, "freq=", 5) == 0) {
			freq = p + 5;
			char *sp = freq;
			while (*sp && *sp != ' ' && *sp != '\t') {
				sp++;
			}
			if (*sp) {
				*sp = 0;
				p = sp + 1;
				while (*p == ' ' || *p == '\t') {
					p++;
				}
			} else {
				p = sp;	 // freq only, empty path
			}
		}
		startSession(p, freq);
	} else if (strncmp(s, "STOP", 4) == 0 && boundary(s[4])) {
		stopSession();
	} else if (strcmp(s, "s") == 0 || strcmp(s, "STATUS") == 0) {
		g_running ? printHealth(false) : printIdle();
	} else if (strcmp(s, "h") == 0 || strcmp(s, "HEADER") == 0) {
		printHeader();
	} else if (Serial.availableForWrite() >= kLineMax) {  // aid driver debugging; drop if no room
		Serial.printf("#ERR,unknown_command,%s\n", s);
	}
}


// ---------------------------------------------------------------------------
void setup() {
	leds::begin();
	Serial.begin(115200);
	uint32_t t0 = millis();
	while (!Serial && (millis() - t0) < 3000) {
	}

	timebase::begin();

	// bias to idle level: floating PPS line reads quiet, not noisy
	pinMode(PPS_PIN, PPS_RISING_EDGE ? INPUT_PULLDOWN : INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsIsr, PPS_RISING_EDGE ? RISING : FALLING);

	timesync::begin();
	channels::begin();

	// boot idle: triggers off, nothing counted, until START
	Serial.println("# SensorSync-logger ready (idle). commands: START [freq=FX:hz,JAI:hz] [path] | STOP | s (status) | h (header)");
}

// ---------------------------------------------------------------------------
void loop() {
    channels::pollHealth();
    if (g_running && channels::faulted()) {
        if (waitTx(64, 50)) Serial.println("#ERR,pwm_event_loss");
        closeRunningSession();
    }
	char buf[kLineMax];

	// drain only while a session runs; idle ISRs push nothing
	if (g_running) {
		// PPS
		{
			uint64_t tick;
			uint32_t seq;
			for (int d = 0; d < DRAIN_PER_LOOP && timesync::popPps(tick, seq); d++) {
				emit(buf, snprintf(buf, sizeof(buf), "P,%lu,%llu\n", (unsigned long) seq, (unsigned long long) tick));
			}
		}
		// trigger edges
		{
			uint8_t ch, level;
			uint64_t tick;
			uint32_t seq;
			for (int d = 0; d < DRAIN_PER_LOOP && channels::popTrig(ch, level, tick, seq); d++) {
				emit(buf, snprintf(buf, sizeof(buf), "T,%lu,%u,%u,%llu\n", (unsigned long) seq, ch, level, (unsigned long long) tick));
			}
		}
		// strobe edges
		{
			uint8_t ch, level;
			uint64_t tick;
			uint32_t seq;
			for (int d = 0; d < DRAIN_PER_LOOP && channels::popStrobe(ch, level, tick, seq); d++) {
				emit(buf, snprintf(buf, sizeof(buf), "S,%lu,%u,%u,%llu\n", static_cast<unsigned long>(seq), ch, level,
								   static_cast<unsigned long long>(tick)));
			}
		}
		// ToD sentences
		{
			timesync::TodMsg m;
			for (int d = 0; d < DRAIN_PER_LOOP && timesync::readTod(m); d++) {
				emit(buf, snprintf(buf, sizeof(buf), "Z,%lu,%llu,%s\n", static_cast<unsigned long>(m.seq),
								   static_cast<unsigned long long>(m.tick), m.nmea));
			}
		}
	}

	// command input: accumulate to newline, then dispatch
	// (256: "START freq=... " + a long absolute log path must fit untruncated)
	static char cmd[256];
	static uint16_t cmdlen = 0;
	static bool cmd_overflow = false;
	while (Serial.available()) {
		const char c = static_cast<char>(Serial.read());
		if (c == '\n' || c == '\r') {
			if (cmd_overflow) {
				if (waitTx(64, 50)) Serial.println("#ERR,command_too_long");
				cmdlen = 0;
				cmd_overflow = false;
			} else if (cmdlen > 0) {
				cmd[cmdlen] = 0;
				handleCommand(cmd);
				cmdlen = 0;
			}
		} else if (cmdlen < sizeof(cmd) - 1) {
			cmd[cmdlen++] = c;
		} else {
			cmd_overflow = true;
		}
	}

	// Visible activity from the counters; keep GPIO work out of the timing ISRs.
	if (g_running) leds::poll();

	// heartbeat: #H while running, #IDLE while waiting
	static uint32_t last = 0;
	if (millis() - last > 5000) {
		last = millis();
		g_running ? printHealth(false) : printIdle();
	}
}
