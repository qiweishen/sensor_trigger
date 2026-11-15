// =============================================================================
//  SensorSync-Logger - Teensy 4.1
//  Logs GPT1 ticks of PPS, trigger and exposure edges plus raw NMEA over USB.
//  The host fits tick -> GNSS time offline (postprocess.py).
//
//  Commands:  START [freq=FX:<hz>,JAI:<hz>] [path] | STOP | s (status)
//
//  Log lines:
//    #SESSION,START,<path>                     path: echo only
//    #LOG,SensorSync-logger,3,tick_hz=<f>
//    #GROUP,<FX|JAI>,requested=<hz>,actual=<hz>,pulse_us=<us>
//    #TRIG,<ch>,<name>,<pin>,<hz>              hz 0: output off
//    #STROBE,<ch>,<name>,<pin>,ah=<0|1>        ah: level during exposure
//    #READY                                    PWM running
//    P,<seq>,<tick>                            PPS rising edge
//    T,<seq>,<ch>,1,<tick>                     trigger rising edge (PWM ISR)
//    S,<seq>,<ch>,<level>,<tick>               exposure input edge (GPIO ISR)
//    Z,<seq>,<tick>,<NMEA>                     sentence; tick at terminator (1 ms poll)
//    #H up=<ms> pps= tod= T=a,b,c,d S=a,b,c,d drops=<n>   every 5 s and at STOP
//    #SESSION,STOP,<path>
//  seq: per stream from 1 each session; drops = lines lost (queue or USB full)
// =============================================================================

#include <stdarg.h>

#include "config.h"
#include "events.h"
#include "leds.h"
#include "timebase.h"
#include "triggers.h"


static bool g_running = false;
static char g_path[200] = "";
static uint32_t g_tod_seq = 0, g_usb_drops = 0, g_health_ms = 0;
static char g_nmea[100];
static uint8_t g_nmea_len = 0;


// data line: dropped (and counted) when the host is not reading; never blocks
static void emit(const char *fmt, ...) {
	char line[160];
	va_list ap;
	va_start(ap, fmt);
	const int n = vsnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);
	if (n > 0 && n < int(sizeof(line)) && Serial.availableForWrite() >= n) {
		Serial.write(line, n);
	}
	else {
		++g_usb_drops;
	}
}


static void printEvent(const events::Event &e) {
	const unsigned long seq = e.seq;
	const unsigned long long tick = e.tick;
	if (e.kind == events::PPS) {
		emit("P,%lu,%llu\n", seq, tick);
		leds::blink(0);
	} else {
		const bool trig = e.kind == events::TRIG;
		emit("%c,%lu,%u,%u,%llu\n", trig ? 'T' : 'S', seq, e.ch, e.level, tick);
		leds::blink((e.ch < 2 ? 2 : 4) + (trig ? 0 : 1));
	}
}


// raw NMEA -> Z line; overlong sentences get truncated and fail their checksum offline
static void readTod() {
	int c;
	while ((c = TOD_SERIAL.read()) >= 0) {
		if (c == '$') {
			g_nmea_len = 0;
		}
		if (c == '\r' || c == '\n') {
			if (g_nmea_len > 6 && g_nmea[0] == '$') {
				g_nmea[g_nmea_len] = 0;
				emit("Z,%lu,%llu,%s\n", (unsigned long) ++g_tod_seq, (unsigned long long) timebase::now(), g_nmea);
				leds::blink(1);
			}
			g_nmea_len = 0;
		} else if (g_nmea_len < sizeof(g_nmea) - 1) {
			g_nmea[g_nmea_len++] = char(c);
		}
	}
}


static void printHealth() {
	using events::count;
	emit("#H up=%lu pps=%lu tod=%lu T=%lu,%lu,%lu,%lu S=%lu,%lu,%lu,%lu drops=%lu\n", millis(), count(events::PPS),
		 g_tod_seq, count(events::TRIG, 0), count(events::TRIG, 1), count(events::TRIG, 2), count(events::TRIG, 3),
		 count(events::STROBE, 0), count(events::STROBE, 1), count(events::STROBE, 2), count(events::STROBE, 3),
		 events::drops() + g_usb_drops);
}


static void printHeader() {
	Serial.printf("#SESSION,START,%s\n", g_path);
	Serial.printf("#LOG,SensorSync-logger,3,tick_hz=%.3f\n", timebase::hz());
	for (uint8_t g = 0; g < 2; ++g) {
		const auto &t = triggers::timing(g);
		Serial.printf("#GROUP,%s,requested=%.8f,actual=%.8f,pulse_us=%.6f\n", g ? "JAI" : "FX", triggers::requested(g),
					  t.actual_hz, t.pulse_us);
	}
	for (uint8_t i = 0; i < 4; ++i) {
		Serial.printf("#TRIG,%u,%s,%u,%.8f\n", i, TRIG_CFG[i].name, TRIG_CFG[i].pin, triggers::freqHz(i));
	}
	for (uint8_t i = 0; i < 4; ++i) {
		Serial.printf("#STROBE,%u,%s,%u,ah=%u\n", i, STROBE_CFG[i].name, STROBE_CFG[i].pin, STROBE_CFG[i].active_high);
	}
}


static void printIdle() {
	Serial.printf("#IDLE,up=%lu\n", millis());
}


static void stopSession() {
	triggers::stop();
	delay(STOP_TAIL_MS);  // last exposure may still be open
	events::capture(false);
	events::Event e;
	while (events::pop(e)) {
		printEvent(e);
	}
	readTod();
	printHealth();
	Serial.printf("#SESSION,STOP,%s\n", g_path);
	leds::off();
	g_running = false;
}


// "FX:<hz>,JAI:<hz>", either item optional
static bool parseRates(char *spec, double hz[2]) {
	for (char *item = strtok(spec, ","); item; item = strtok(nullptr, ",")) {
		const int g = strncmp(item, "FX:", 3) == 0 ? 0 : strncmp(item, "JAI:", 4) == 0 ? 1 : -1;
		if (g < 0) {
			return false;
		}
		char *num = strchr(item, ':') + 1, *end;
		hz[g] = strtod(num, &end);
		if (end == num || *end) {
			return false;
		}
	}
	return true;
}


// args: "[freq=FX:<hz>,JAI:<hz>] [path]"; rates persist until changed or reboot
static void startSession(char *args) {
	double hz[2] = { triggers::requested(0), triggers::requested(1) };
	char *path = args;
	if (strncmp(args, "freq=", 5) == 0) {
		char *spec = args + 5;
		char *space = strchr(spec, ' ');
		path = space ? space + 1 : spec + strlen(spec);
		if (space) {
			*space = 0;
		}
		if (!parseRates(spec, hz) || !triggers::setRates(hz[0], hz[1])) {
			Serial.println("#ERR,bad_freq");
			return;
		}
	}
	while (*path == ' ') {
		++path;
	}
	if (g_running) {
		stopSession();
	}
	strncpy(g_path, path, sizeof(g_path) - 1);
	events::reset();
	g_tod_seq = g_usb_drops = 0;
	g_nmea_len = 0;
	while (TOD_SERIAL.read() >= 0) {  // stale idle bytes
	}
	printHeader();
	g_running = true;
	events::capture(true);
	triggers::start();
	g_health_ms = millis();
	Serial.println("#READY");
}


static void handleCommand(char *cmd) {
	if (strncmp(cmd, "START", 5) == 0 && (cmd[5] == ' ' || cmd[5] == 0)) {
		startSession(cmd + 5 + (cmd[5] == ' '));
	}
	else if (strcmp(cmd, "STOP") == 0) {
		g_running ? stopSession() : printIdle();
	}
	else if (strcmp(cmd, "s") == 0) {
		g_running ? printHealth() : printIdle();
	}
	else {
		Serial.printf("#ERR,unknown_command,%s\n", cmd);
	}
}


static void readCommands() {
	static char cmd[256];
	static uint16_t n = 0;
	while (Serial.available()) {
		const char c = Serial.read();
		if (c == '\n' || c == '\r') {
			cmd[n] = 0;
			if (n) {
				handleCommand(cmd);
			}
			n = 0;
		} else if (n < sizeof(cmd) - 1) {
			cmd[n++] = c;
		}
	}
}


void setup() {
	Serial.begin(115200);  // USB CDC: nominal
	leds::begin();
	timebase::begin();
	TOD_SERIAL.begin(TOD_BAUD);
	static uint8_t rx_extra[512];  // INS bursts
	TOD_SERIAL.addMemoryForRead(rx_extra, sizeof(rx_extra));
	events::begin();
	triggers::begin();
	Serial.println("# SensorSync-logger ready. commands: START [freq=FX:hz,JAI:hz] [path] | STOP | s");
}


void loop() {
	events::Event e;
	while (events::pop(e)) {
		printEvent(e);
	}
	readCommands();

	// 1 kHz housekeeping; UART reads stay out of every pass (core masks IRQs on empty reads)
	static uint32_t last_ms = 0;
	const uint32_t ms = millis();
	if (ms == last_ms) {
		return;
	}
	last_ms = ms;
	leds::poll(ms);
	if (!g_running) {
		return;
	}
	readTod();
	if (ms - g_health_ms >= HEALTH_MS) {
		g_health_ms = ms;
		printHealth();
	}
}
