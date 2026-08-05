// =============================================================================
//  SensorSync-Logger - Teensy 4.1
//
//  Raw event logger for post-processed GNSS timing. No on-board time solution:
//  the host pairs ToD<->PPS offline and fits tick->GNSS with a centered window.
//
//  Log lines ('#' = comment / status):
//    #SESSION,START,<path>                         session begin (path echo only)
//    #LOG,SensorSync-logger,1,tick_hz=<f>          header
//    #TRIG,<i>,<name>,<pin>,<freq>,ah=<0|1>        per trigger channel
//    #STROBE,<i>,<name>,<pin>,ah=<0|1>             per strobe channel
//    P,<seq>,<tick>                                PPS edge
//    T,<seq>,<ch>,<level>,<tick>                   trigger edge (tick = compare value)
//    S,<seq>,<ch>,<level>,<tick>                   strobe edge
//    Z,<seq>,<tick>,<raw NMEA sentence>            ToD sentence + read-out tick
//    #H / #Ht / #Hs                                health, every 5 s while running
//    #SESSION,STOP,<path>                          session end
//    #IDLE,up=<ms>                                 heartbeat while idle
//  Per-stream <seq> starts at 1 each session; advances even on drop -> gaps visible.
// =============================================================================

#include "channels.h"
#include "config.h"
#include "timebase.h"
#include "timesync.h"


// ---------------------------------------------------------------------------
// Session state: idle after boot; logs only between START and STOP.
// START zeroes all counters -> every stream counts from 0 per session.
static volatile bool g_running = false;
static char g_session_path[224] = { 0 };  // >= max cmd payload; no truncation


// ---------------------------------------------------------------------------
// PPS input
#if !USE_HW_CAPTURE
static void ppsIsr() {
	if (g_running) {  // gate: no counting while idle
		timesync::onPpsEvent(timebase::now());
	}
}
#endif

#if USE_HW_CAPTURE
static_assert(PPS_PIN == 48, "USE_HW_CAPTURE=1 requires PPS on pin 48; GPIO_EMC_24 is the only GPT1_CAPTURE1 pad on Teensy 4.1");
// Gate for gptHook's IF1 forwarding: failed capture never fabricates PPS.
static bool g_capture_ok = false;
#endif

// GPT1 status word forwarded from the timebase ISR
static void gptHook(uint32_t sr, uint32_t icr1) {
#if USE_HW_CAPTURE
	if (g_running && g_capture_ok && (sr & SS_GPT_SR_IF1)) {
		timesync::onPpsEvent(timebase::extend(icr1));
	}
#else
	(void) icr1;
#endif
	channels::gptIsr(sr);
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
static void printHealth() {
	// whole block < 512 B; skip rather than block
	if (Serial.availableForWrite() < 512) {
		return;
	}
	Serial.printf("#H up=%lu en=%d pps=%lu ppsdt=%.6f ppsdrops=%lu tod=%lu todb=%lu todtrunc=%lu tdrops=%lu sdrops=%lu host=%lu\n",
				  millis(), channels::enabled() ? 1 : 0, timesync::ppsRawCount(), timesync::lastPpsIntervalSec(),
				  timesync::ppsDrops(), timesync::todCount(), timesync::todBytes(), timesync::todTruncated(),
				  channels::trigDrops(), channels::strobeDrops(), static_cast<unsigned long>(g_host_drops));
	for (uint8_t i = 0; i < TRIG_MAX; i++) {
		Serial.printf("#Ht %u %-8s n=%lu skip=%lu stall=%lu\n", i, TRIG_CFG[i].name, channels::trigCount(i),
					  channels::skippedCount(i), channels::stalledCount(i));
	}
	for (uint8_t i = 0; i < STROBE_MAX; i++) {
		Serial.printf("#Hs %u %-10s n=%lu\n", i, STROBE_CFG[i].name, channels::strobeCount(i));
	}
}


// ---------------------------------------------------------------------------
// Header: tick_hz + channel map; postprocess.py needs it to be self-contained.
// Re-emitted on 'h', so a mid-session (re)connect still captures it.
static void printHeader() {
	if (!waitTx(512, 250)) {  // all lines fit in 512 B once room exists
		return;
	}
#if USE_HW_CAPTURE
	Serial.printf("#LOG,SensorSync-logger,1,tick_hz=%.3f,cap=%d\n", timebase::nominalHz(), g_capture_ok ? 1 : 0);
#else
	Serial.printf("#LOG,SensorSync-logger,1,tick_hz=%.3f\n", timebase::nominalHz());
#endif
	for (uint8_t i = 0; i < TRIG_MAX; i++) {
		// effective rate: config.h default, or the session's freq= override
		Serial.printf("#TRIG,%u,%s,%u,%.4f,ah=%u\n", i, TRIG_CFG[i].name, TRIG_CFG[i].pin, channels::freqHz(i),
					  TRIG_CFG[i].active_high ? 1 : 0);
	}
	for (uint8_t i = 0; i < STROBE_MAX; i++) {
		Serial.printf("#STROBE,%u,%s,%u,ah=%u\n", i, STROBE_CFG[i].name, STROBE_CFG[i].pin, STROBE_CFG[i].active_high ? 1 : 0);
	}
}


// idle heartbeat
static void printIdle() {
	if (Serial.availableForWrite() < 128) {
		return;
	}
	Serial.printf("#IDLE,up=%lu\n", (unsigned long) millis());
#if USE_HW_CAPTURE
	if (!g_capture_ok) {  // keep a boot-time capture failure visible to a late-attaching host
		Serial.println("# !! GPT1_CAPTURE1 self-test FAILED at boot -- PPS will NOT be logged");
	}
#endif
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
}


// ---------------------------------------------------------------------------
// Session control (host-facing API).
//   START [freq=<ch>:<hz>,...] [path]   zero all counts, optionally retune
//                 trigger rates, enable capture, emit #SESSION,START + header
//   STOP          drain tail, emit #SESSION,STOP, park idle, clear counts
// <path> is echo only (board cannot write the host filesystem); it makes the
// log self-document where the host saved it. freq= overrides persist until the
// next override or reboot; the #TRIG header always shows the effective rate.
static void closeRunningSession() {
	g_running = false;
	channels::setEnabled(false);
	drainAll(500);	// flush the session tail before the marker
	if (waitTx(512, 250)) {	 // marker line can exceed kLineMax (long path)
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


// Apply a "freq=" START option: "<ch>:<hz>[,<ch>:<hz>...]", hz 0 = channel off.
// Invalid items answer #ERR and keep the old rate; the header shows the truth.
static void applyFreqSpec(char *spec) {
	for (char *tok = strtok(spec, ","); tok; tok = strtok(nullptr, ",")) {
		bool ok = false;
		char *colon = strchr(tok, ':');
		if (colon) {
			*colon = 0;
			char *e1 = nullptr;
			char *e2 = nullptr;
			const long ch = strtol(tok, &e1, 10);
			const double hz = strtod(colon + 1, &e2);
			ok = e1 && *e1 == 0 && e1 != tok && e2 && *e2 == 0 && e2 != colon + 1 && ch >= 0 && ch < TRIG_MAX &&
				 channels::setFreqHz(static_cast<uint8_t>(ch), hz);
			*colon = ':';  // restore for the error echo
		}
		if (!ok && Serial.availableForWrite() >= kLineMax) {
			Serial.printf("#ERR,bad_freq,%s\n", tok);
		}
	}
}


static void startSession(const char *path, char *freqspec = nullptr) {
	if (g_running) {
		closeRunningSession();	// paired STOP marker for the old session
	}
	timesync::flushInput();
	timesync::resetStats();
	channels::resetCounts();
	g_host_drops = 0;
	if (freqspec) {
		applyFreqSpec(freqspec);  // while disabled; new rates land in the header below
	}

	// announce, header first so no data line precedes it
	strncpy(g_session_path, path ? path : "", sizeof(g_session_path) - 1);
	g_session_path[sizeof(g_session_path) - 1] = 0;
	if (waitTx(512, 250)) {	 // marker line can exceed kLineMax (long path)
		Serial.printf("#SESSION,START,%s\n", g_session_path);
	}
	printHeader();

	// go: PPS gated on g_running; triggers + strobe on channels::enabled()
	g_running = true;
	channels::setEnabled(true);
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
		// optional leading option: "freq=<ch>:<hz>[,...]", then the path
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
		g_running ? printHealth() : printIdle();
	} else if (strcmp(s, "h") == 0 || strcmp(s, "HEADER") == 0) {
		printHeader();
	} else if (Serial.availableForWrite() >= kLineMax) {  // aid driver debugging; drop if no room
		Serial.printf("#ERR,unknown_command,%s\n", s);
	}
}


// ---------------------------------------------------------------------------
void setup() {
	Serial.begin(115200);
	uint32_t t0 = millis();
	while (!Serial && (millis() - t0) < 3000) {
	}

	timebase::begin();
	timebase::setIsrHook(gptHook);

#if USE_HW_CAPTURE
	{
		uint8_t used = 0;
		const uint8_t drive = HW_CAPTURE_SELFTEST ? (uint8_t) HW_SELFTEST_DRIVE_PIN : (uint8_t) 0xFF;
		g_capture_ok = timebase::setupCapture1(PPS_RISING_EDGE ? true : false, (uint8_t) HW_PPS_DAISY, drive, used);
		if (g_capture_ok) {
			Serial.printf("# hw capture OK: pin %d (GPIO_EMC_24 ALT4), daisy=%u\n", (int) PPS_PIN, (unsigned) used);
			if (HW_PPS_DAISY == HW_PPS_DAISY_AUTO) {
				Serial.printf("# -> pin HW_PPS_DAISY to %u, then remove the jumper and set HW_CAPTURE_SELFTEST 0\n", (unsigned) used);
			}
		} else {
			Serial.println("# !! FATAL: GPT1_CAPTURE1 self-test FAILED -- PPS will NOT be logged");
			Serial.printf("# !!   is PPS on pin %d? is the self-test jumper %d -> %d fitted?\n", (int) PPS_PIN,
						  (int) HW_SELFTEST_DRIVE_PIN, (int) PPS_PIN);
		}
	}
#else
	// bias to idle level: floating PPS line reads quiet, not noisy
	pinMode(PPS_PIN, PPS_RISING_EDGE ? INPUT_PULLDOWN : INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsIsr, PPS_RISING_EDGE ? RISING : FALLING);
#endif

	timesync::begin();
	channels::begin();

	// boot idle: triggers off, nothing counted, until START
	Serial.println("# SensorSync-logger ready (idle). commands: START [freq=ch:hz,...] [path] | STOP | s (status) | h (header)");
}

// ---------------------------------------------------------------------------
void loop() {
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
	while (Serial.available()) {
		const char c = static_cast<char>(Serial.read());
		if (c == '\n' || c == '\r') {
			if (cmdlen > 0) {
				cmd[cmdlen] = 0;
				handleCommand(cmd);
				cmdlen = 0;
			}
		} else if (cmdlen < sizeof(cmd) - 1) {
			cmd[cmdlen++] = c;
		}
	}

	// heartbeat: #H while running, #IDLE while waiting
	static uint32_t last = 0;
	if (millis() - last > 5000) {
		last = millis();
		g_running ? printHealth() : printIdle();
	}
}
