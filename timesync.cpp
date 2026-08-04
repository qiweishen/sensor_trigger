#include "timesync.h"

#include "timebase.h"

namespace timesync {

	// ---------------------------------------------------------------------------
	// PPS event queue, capture ISR -> loop (single producer context, SPSC)
	static volatile uint64_t g_pps_tick[PPS_RING_SIZE];
	static volatile uint32_t g_pps_seq_slot[PPS_RING_SIZE];
	static volatile uint16_t g_pps_head = 0, g_pps_tail = 0;
	static volatile uint32_t g_pps_seq = 0;	  // total PPS edges seen (incl. dropped) = gap detector
	static volatile uint32_t g_pps_drops = 0;

	// Interval between the last two PPS edges, in ticks (for the health readout).
	static volatile uint64_t g_prev_pps_tick = 0;
	static volatile uint64_t g_last_interval = 0;
	static volatile bool g_have_prev = false;


	void onPpsEvent(const uint64_t tick) {
		const uint32_t s = ++g_pps_seq;	 // advance even on drop, so a gap in seq = a lost edge
		if (g_have_prev) {
			g_last_interval = tick - g_prev_pps_tick;
		}
		g_prev_pps_tick = tick;
		g_have_prev = true;

		const uint16_t h = g_pps_head;
		const auto nh = static_cast<uint16_t>((h + 1) % PPS_RING_SIZE);
		if (nh == g_pps_tail) {
			g_pps_drops++;	// ring full: the edge is dropped, but its seq is spent -> visible gap
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
	static uint32_t g_tod_count = 0;
	static uint32_t g_tod_bytes = 0;

	// Teensy's default 64-byte RX buffer holds only ~5.5 ms at 115200; an INS NMEA burst
	// is easily 300 bytes. One loop() hiccup longer than that would truncate a sentence.
	static uint8_t g_rx_extra[512];


	void begin() {
		TOD_INPUT_SERIAL.begin(TOD_BAUD);
		TOD_INPUT_SERIAL.addMemoryForRead(g_rx_extra, sizeof(g_rx_extra));
		resetStats();
	}


	void resetStats() {
		g_pps_seq = 0;
		g_pps_drops = 0;
		g_tod_count = 0;
		g_tod_bytes = 0;
	}


	bool readTod(TodMsg &out) {
		while (TOD_INPUT_SERIAL.available()) {
			const char c = static_cast<char>(TOD_INPUT_SERIAL.read());
			g_tod_bytes++;
			if (c == '$') {	 // sentence start
				g_nmea_len = 0;
				g_nmea[g_nmea_len++] = c;
				continue;
			}
			if (g_nmea_len == 0) {
				continue;  // no '$' yet, ignore
			}
			if (c == '\r' || c == '\n') {  // terminator: sentence complete
				const uint64_t arrive = timebase::now();
				g_nmea[g_nmea_len] = 0;
				if (g_nmea_len > 6) {  // ignore runts; the host validates the checksum
					out.seq = ++g_tod_count;
					out.tick = arrive;
					out.len = g_nmea_len;
					memcpy(out.nmea, g_nmea, g_nmea_len + 1);
					g_nmea_len = 0;
					return true;
				}
				g_nmea_len = 0;
				continue;
			}
			if (g_nmea_len < sizeof(g_nmea) - 1) {
				g_nmea[g_nmea_len++] = c;
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
	double lastPpsIntervalSec() {
		return g_have_prev ? (static_cast<double>(g_last_interval) / timebase::nominalHz()) : 0.0;
	}

}  // namespace timesync
