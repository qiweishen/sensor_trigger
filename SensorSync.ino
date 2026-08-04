// =============================================================================
//  SensorSync-Logger - Teensy 4.1
//
//  A RAW EVENT LOGGER for post-processed GNSS timing. The board computes no GNSS
//  time; it logs the tick of every PPS edge, every trigger edge, every strobe
//  edge, and every raw NMEA ToD sentence. A host script pairs ToD<->PPS and does
//  a CENTERED sliding-window fit to convert every tick to GNSS time offline -
//  which removes the causal extrapolation bias of an on-board trailing fit and
//  can be re-run with better estimators.
//
//  Log format (each line ends with '\n'; '#' lines are comments / status):
//    #LOG,SensorSync-logger,1,tick_hz=<f>          session header
//    #TRIG,<i>,<name>,<pin>,<freq>,ah=<0|1>        one per trigger channel
//    #STROBE,<i>,<name>,<pin>,ah=<0|1>             one per strobe channel
//    P,<seq>,<tick>                                a PPS edge
//    T,<seq>,<ch>,<level>,<tick>                   a trigger edge (tick = exact compare)
//    S,<seq>,<ch>,<level>,<tick>                   a strobe edge
//    Z,<seq>,<tick>,<raw NMEA sentence>            a ToD sentence + read-out tick
//    #H ...                                        health, every 5 s
//  Per-stream <seq> advances even on a drop, so any gap in the log is detectable.
// =============================================================================

#include "channels.h"
#include "config.h"
#include "timebase.h"
#include "timesync.h"

// ---------------------------------------------------------------------------
// PPS input
#if !USE_HW_CAPTURE
static void ppsIsr() {
	timesync::onPpsEvent(timebase::now());
}
#endif

#if USE_HW_CAPTURE
static_assert(PPS_PIN == 48, "USE_HW_CAPTURE=1 requires PPS on pin 48; GPIO_EMC_24 is the only GPT1_CAPTURE1 pad on Teensy 4.1");
// Gates gptHook's IF1 forwarding: a failed/reverted capture can never fabricate a PPS event.
static bool g_capture_ok = false;
#endif

// timebase forwards the GPT1 status word here
static void gptHook(uint32_t sr, uint32_t icr1) {
#if USE_HW_CAPTURE
	if (g_capture_ok && (sr & SS_GPT_SR_IF1)) {
		timesync::onPpsEvent(timebase::extend(icr1));
	}
#else
	(void) icr1;
#endif
	channels::gptIsr(sr);
}

// ---------------------------------------------------------------------------
// Output: never block loop(). USB CDC busy-waits up to 120 ms when the host is
// attached but not reading, which would starve the event drains and drop lock
// on the timing. Drop and count instead of spinning.
static uint32_t g_host_drops = 0;

static void emit(const char *line, int n) {
	if (n <= 0) {
		return;
	}
	if (n >= 0 && Serial.availableForWrite() >= n) {
		Serial.write(reinterpret_cast<const uint8_t *>(line), static_cast<size_t>(n));
	} else {
		g_host_drops++;
	}
}

// ---------------------------------------------------------------------------
static void printHealth() {
	// One block < ~512 B; skip if the USB TX buffer can't take it, rather than block.
	if (Serial.availableForWrite() < 512) {
		return;
	}
	Serial.printf("#H up=%lu en=%d pps=%lu ppsdt=%.6f ppsdrops=%lu tod=%lu todb=%lu tdrops=%lu sdrops=%lu host=%lu\n",
				  (unsigned long) millis(), channels::enabled() ? 1 : 0, (unsigned long) timesync::ppsRawCount(),
				  timesync::lastPpsIntervalSec(), (unsigned long) timesync::ppsDrops(), (unsigned long) timesync::todCount(),
				  (unsigned long) timesync::todBytes(), (unsigned long) channels::trigDrops(), (unsigned long) channels::strobeDrops(),
				  (unsigned long) g_host_drops);
	for (uint8_t i = 0; i < TRIG_MAX; i++) {
		Serial.printf("#Ht %u %-8s n=%lu skip=%lu\n", i, TRIG_CFG[i].name, (unsigned long) channels::trigCount(i),
					  (unsigned long) channels::skippedCount(i));
	}
	for (uint8_t i = 0; i < STROBE_MAX; i++) {
		Serial.printf("#Hs %u %-10s n=%lu\n", i, STROBE_CFG[i].name, (unsigned long) channels::strobeCount(i));
	}
}

// ---------------------------------------------------------------------------
// Machine-readable session header. Also re-emitted on the 'h' console command, so a
// logger that connects (or reconnects) mid-session still captures tick_hz + channel map.
static void printHeader() {
	Serial.printf("#LOG,SensorSync-logger,1,tick_hz=%.3f\n", timebase::nominalHz());
	for (uint8_t i = 0; i < TRIG_MAX; i++) {
		Serial.printf("#TRIG,%u,%s,%u,%.4f,ah=%u\n", i, TRIG_CFG[i].name, TRIG_CFG[i].pin, TRIG_CFG[i].freq_hz,
					  TRIG_CFG[i].active_high ? 1 : 0);
	}
	for (uint8_t i = 0; i < STROBE_MAX; i++) {
		Serial.printf("#STROBE,%u,%s,%u,ah=%u\n", i, STROBE_CFG[i].name, STROBE_CFG[i].pin, STROBE_CFG[i].active_high ? 1 : 0);
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
	// Bias to the idle level so a floating/unpowered PPS line reads quiet, not noisy.
	pinMode(PPS_PIN, PPS_RISING_EDGE ? INPUT_PULLDOWN : INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsIsr, PPS_RISING_EDGE ? RISING : FALLING);
#endif

	timesync::begin();
	channels::begin();

	// Session header - the post-processor reads these to be self-contained.
	printHeader();
	Serial.println("# logger ready. commands: s=health  r=reset  a=enable trig  d=disable trig  h=reprint header");
}

// ---------------------------------------------------------------------------
void loop() {
	char buf[160];

	// PPS
	{
		uint64_t tick;
		uint32_t seq;
		for (int d = 0; d < DRAIN_PER_LOOP && timesync::popPps(tick, seq); d++) {
			emit(buf, snprintf(buf, sizeof(buf), "P,%lu,%llu\n", (unsigned long) seq, (unsigned long long) tick));
		}
	}
	// Trigger edges
	{
		uint8_t ch, level;
		uint64_t tick;
		uint32_t seq;
		for (int d = 0; d < DRAIN_PER_LOOP && channels::popTrig(ch, level, tick, seq); d++) {
			emit(buf, snprintf(buf, sizeof(buf), "T,%lu,%u,%u,%llu\n", (unsigned long) seq, ch, level, (unsigned long long) tick));
		}
	}
	// Strobe edges
	{
		uint8_t ch, level;
		uint64_t tick;
		uint32_t seq;
		for (int d = 0; d < DRAIN_PER_LOOP && channels::popStrobe(ch, level, tick, seq); d++) {
			emit(buf, snprintf(buf, sizeof(buf), "S,%lu,%u,%u,%llu\n", static_cast<unsigned long>(seq), ch, level, static_cast<unsigned long long>(tick)));
		}
	}
	// ToD sentences
	{
		timesync::TodMsg m;
		for (int d = 0; d < DRAIN_PER_LOOP && timesync::readTod(m); d++) {
			emit(buf, snprintf(buf, sizeof(buf), "Z,%lu,%llu,%s\n", static_cast<unsigned long>(m.seq), static_cast<unsigned long long>(m.tick), m.nmea));
		}
	}

	// Console
	while (Serial.available()) {
		const char c = static_cast<char>(Serial.read());
		if (c == 's') {
			printHealth();
		} else if (c == 'r') {
			timesync::resetStats();
			g_host_drops = 0;
		} else if (c == 'a') {
			channels::setEnabled(true);
		} else if (c == 'd') {
			channels::setEnabled(false);
		} else if (c == 'h') {
			printHeader();
		}
	}

	// Periodic health
	static uint32_t last = 0;
	if (millis() - last > 5000) {
		last = millis();
		printHealth();
	}
}
