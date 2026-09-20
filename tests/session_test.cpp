// Exercise the real .ino command path, including validation before old-session
// shutdown, READY ordering, STOP metadata and automatic error shutdown.
#include <cassert>
#include <string>

#include "SensorSync.ino"
#include "session_protocol.h"

namespace timebase {
	void begin() {}
	double nominalHz() {
		return 75000000.;
	}
	uint64_t now() {
		set_tick(fake_tick + 25);
		return fake_tick;
	}
}  // namespace timebase
namespace timesync {
	unsigned resets = 0;
	void begin() {}
	void onPpsEvent(uint64_t) {}
	bool popPps(uint64_t &, uint32_t &) {
		return false;
	}
	bool readTod(TodMsg &) {
		return false;
	}
	uint32_t ppsRawCount() {
		return 0;
	}
	uint32_t ppsDrops() {
		return 0;
	}
	uint32_t todCount() {
		return 0;
	}
	uint32_t todBytes() {
		return 0;
	}
	uint32_t todTruncated() {
		return 0;
	}
	double lastPpsIntervalSec() {
		return 0;
	}
	void resetStats() {
		++resets;
	}
	void flushInput() {}
}  // namespace timesync
namespace leds {
	void begin() {}
	void reset() {}
	void poll() {}
}  // namespace leds
static void command(const char *text) {
	char cmd[256];
	std::snprintf(cmd, sizeof(cmd), "%s", text);
	handleCommand(cmd);
}
static void observe(SensorSyncProtocol &p, const std::string &text, bool stopping) {
	size_t pos = 0;
	while (pos < text.size()) {
		const auto end = text.find('\n', pos);
		p.observe(std::string_view(text).substr(pos, end - pos), stopping);
		if (end == std::string::npos)
			break;
		pos = end + 1;
	}
}
int main() {
	millis_step = 1;
	set_tick(100000);
	channels::begin();
	command("START freq=FX:25,JAI:5 run");
	assert(g_running && channels::enabled());
	assert(channels::requestedRates().hz[0] == 25 && channels::requestedRates().hz[1] == 5);
	assert(Serial.output.find("#GROUP,JAI,") != std::string::npos);
	assert(Serial.output.find("#TIME,T=PWM_ISR_sample") < Serial.output.find("#READY"));
	SensorSyncProtocol protocol;
	observe(protocol, Serial.output, false);
	assert(protocol.ready && !protocol.failed);
	const unsigned resets = timesync::resets;
	Serial.output.clear();
	for (const char *bad: { "START freq=FX:50,JAI:11 bad", "START freq=FX:50,FX:100 bad", "START freq=0:50 bad", "START freq= bad" }) {
		command(bad);
		assert(g_running && channels::enabled());
		assert(channels::requestedRates().hz[0] == 25 && channels::requestedRates().hz[1] == 5);
		assert(timesync::resets == resets);
	}
	assert(Serial.output.find("#SESSION,") == std::string::npos);
	// A known pending rise at STOP is sampled, then marked as truncated. No
	// falling T edge is invented and the host still accepts a complete STOP.
	Serial.output.clear();
	IMXRT_FLEXPWM2.SM[0].STS.raise(1u << 2);
	command("STOP");
	assert(!g_running && !channels::enabled());
	for (const auto &c: TRIG_CFG)
		assert(levels[c.pin] == LOW && muxes[c.pin] == 5);
	assert(Serial.output.find("#TRUNC,0,1,") != std::string::npos);
	assert(Serial.output.find("T,1,0,1,") != std::string::npos);
	assert(Serial.output.find("T,2,0,0,") == std::string::npos);
	observe(protocol, Serial.output, true);
	if (protocol.failed)
		std::fprintf(stderr, "%s", Serial.output.c_str());
	assert(!protocol.failed && protocol.stop_received && protocol.truncations == 1);

	// STOP during exposure must retain the actual GPIO falling edge, while
	// PWM outputs are already parked. No synthetic edge is permitted.
	Serial.output.clear();
	command("START freq=FX:50,JAI:0 exposure_tail");
	levels[STROBE_CFG[0].pin] = HIGH;
	callbacks[STROBE_CFG[0].pin]();
	assert(channels::pendingStrobes() == 1);
	delay_hook = [] {
		assert(!channels::enabled());
		levels[STROBE_CFG[0].pin] = LOW;
		callbacks[STROBE_CFG[0].pin]();
		delay_hook = nullptr;
	};
	command("STOP");
	assert(channels::pendingStrobes() == 0);
	assert(Serial.output.find("S,2,0,0,") < Serial.output.find("#HFINAL"));
	assert(Serial.output.find("exposure_tail_timeout") == std::string::npos);

	// A stuck exposure is a failed tail, even with zero queue/drop counters.
	Serial.output.clear();
	command("START freq=FX:50,JAI:0 stuck_exposure");
	levels[STROBE_CFG[0].pin] = HIGH;
	callbacks[STROBE_CFG[0].pin]();
	command("STOP");
	assert(Serial.output.find("#ERR,exposure_tail_timeout,mask=1") != std::string::npos);
	assert(Serial.output.find("S,2,0,0,") == std::string::npos);
	const auto recorded_edges = channels::strobeCount(0);
	levels[STROBE_CFG[0].pin] = LOW;
	callbacks[STROBE_CFG[0].pin]();
	assert(channels::strobeCount(0) == recorded_edges);	 // capture closed, no next-epoch leak

	// Incomplete USB header cannot enable hardware or acknowledge readiness.
	Serial.output.clear();
	Serial.space = 0;
	command("START freq=FX:50,JAI:1 blocked");
	assert(!g_running && !channels::enabled() && Serial.output.find("#READY") == std::string::npos);
	Serial.space = 4096;
	command("START freq=FX:50,JAI:1 retry");
	assert(g_running && channels::enabled());
	Serial.output.clear();
	IMXRT_FLEXPWM2.SM[0].STS.raise((1u << 2) | (1u << 3));
	irq_vectors[IRQ_FLEXPWM2_0]();
	assert(channels::faulted());
	loop();
	assert(!g_running && !channels::enabled());
	assert(Serial.output.find("#ERR,pwm_event_loss") != std::string::npos);
	assert(Serial.output.find("pwmerr=1") != std::string::npos);
	assert(Serial.output.find("#SESSION,STOP,retry") != std::string::npos);
	// The next START is an explicit new epoch, with fault counters cleared.
	Serial.output.clear();
	command("START freq=FX:0,JAI:1 jai_only");
	assert(g_running && !channels::faulted());
	assert((IMXRT_FLEXPWM2.MCTRL & 0x900) == 0x900);
	command("STOP");
	puts("firmware session regressions passed");
}
