#include "timesync.h"

#include "timebase.h"

namespace timesync {

	// ---------------------------------------------------------------------------
	// PPS event queue, capture ISR -> loop (single producer context, SPSC)
	static volatile uint64_t g_pps_tick[PPS_RING_SIZE];
	static volatile uint32_t g_pps_seq_slot[PPS_RING_SIZE];
	static volatile uint16_t g_pps_head = 0, g_pps_tail = 0;
	static volatile uint32_t g_pps_seq = 0;	  // total edges seen (incl. dropped) = gap detector
	static volatile uint32_t g_pps_drops = 0;

	// last inter-PPS interval, ticks (health readout only)
	static volatile uint64_t g_prev_pps_tick = 0;
	static volatile uint64_t g_last_interval = 0;
	static volatile bool g_have_prev = false;


	void onPpsEvent(const uint64_t tick) {
		const uint32_t s = ++g_pps_seq;	 // advance even on drop -> seq gap marks the loss
		if (g_have_prev) {
			g_last_interval = tick - g_prev_pps_tick;
		}
		g_prev_pps_tick = tick;
		g_have_prev = true;

		const uint16_t h = g_pps_head;
		const auto nh = static_cast<uint16_t>((h + 1) % PPS_RING_SIZE);
		if (nh == g_pps_tail) {
			g_pps_drops++;	// ring full: edge dropped, seq spent
			return;
		}
		g_pps_tick[h] = tick;
		g_pps_seq_slot[h] = s;
		g_pps_head = nh;
	}


	bool popPps(uint64_t &tick, uint32_t &seq) {
		if (g_pps_tail == g_pps_head) {
			return false;
		}
		tick = g_pps_tick[g_pps_tail];
		seq = g_pps_seq_slot[g_pps_tail];
		g_pps_tail = static_cast<uint16_t>((g_pps_tail + 1) % PPS_RING_SIZE);
		return true;
	}


	// ---------------------------------------------------------------------------
	// ToD assembly (loop context only)
	static char g_nmea[100];
	static uint8_t g_nmea_len = 0;
	static bool g_nmea_overflow = false;
	static uint32_t g_tod_count = 0;
	static uint32_t g_tod_bytes = 0;
	static uint32_t g_tod_trunc = 0;  // oversize sentences discarded

	// default 64-B RX buffer = ~5.5 ms at 115200; an INS burst is easily 300 B
	static uint8_t g_rx_extra[512];


	void begin() {
		TOD_INPUT_SERIAL.begin(TOD_BAUD);
		TOD_INPUT_SERIAL.addMemoryForRead(g_rx_extra, sizeof(g_rx_extra));
		resetStats();
	}


	// full session reset: counters, PPS ring, interval history, half-assembled
	// sentence; call only while paused (no producer running)
	void resetStats() {
		__disable_irq();
		g_pps_seq = 0;
		g_pps_drops = 0;
		g_tod_count = 0;
		g_tod_bytes = 0;
		g_tod_trunc = 0;
		g_pps_head = g_pps_tail = 0;
		g_have_prev = false;
		g_prev_pps_tick = 0;
		g_last_interval = 0;  // else first #H of the next session shows a stale ppsdt
		g_nmea_len = 0;
		g_nmea_overflow = false;
		__enable_irq();
	}

	// drop UART bytes buffered while idle -> session starts on a clean boundary
	void flushInput() {
		while (TOD_INPUT_SERIAL.available()) {
			(void) TOD_INPUT_SERIAL.read();
		}
		g_nmea_len = 0;
		g_nmea_overflow = false;
	}


	bool readTod(TodMsg &out) {
		while (TOD_INPUT_SERIAL.available()) {
			const char c = static_cast<char>(TOD_INPUT_SERIAL.read());
			g_tod_bytes++;
			if (c == '$') {	 // sentence start / resync
				g_nmea_len = 0;
				g_nmea_overflow = false;
				g_nmea[g_nmea_len++] = c;
				continue;
			}
			if (g_nmea_len == 0) {
				continue;  // no '$' yet
			}
			if (c == '\r' || c == '\n') {  // terminator
				const uint64_t arrive = timebase::now();
				g_nmea[g_nmea_len] = 0;
				if (g_nmea_overflow) {	// truncated: discard + count, don't emit garbage
					g_tod_trunc++;
					g_nmea_len = 0;
					g_nmea_overflow = false;
					continue;
				}
				if (g_nmea_len > 6) {  // skip runts; host validates the checksum
					out.seq = ++g_tod_count;
					out.tick = arrive;
					memcpy(out.nmea, g_nmea, g_nmea_len + 1);
					g_nmea_len = 0;
					return true;
				}
				g_nmea_len = 0;
				continue;
			}
			if (g_nmea_len < sizeof(g_nmea) - 1) {
				g_nmea[g_nmea_len++] = c;
			} else {
				g_nmea_overflow = true;
			}
		}
		return false;
	}


	// ---------------------------------------------------------------------------
	uint32_t ppsRawCount() {
		return g_pps_seq;
	}
	uint32_t ppsDrops() {
		return g_pps_drops;
	}
	uint32_t todCount() {
		return g_tod_count;
	}
	uint32_t todBytes() {
		return g_tod_bytes;
	}
	uint32_t todTruncated() {
		return g_tod_trunc;
	}
	double lastPpsIntervalSec() {
		// IRQ-masked snapshot: 64-bit read would tear against the PPS ISR
		__disable_irq();
		const bool have = g_have_prev;
		const uint64_t iv = g_last_interval;
		__enable_irq();
		return have ? (static_cast<double>(iv) / timebase::nominalHz()) : 0.0;
	}

}  // namespace timesync
