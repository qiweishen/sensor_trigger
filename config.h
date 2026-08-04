#pragma once
#include <Arduino.h>

// =============================================================================
//  SensorSync-Logger - Teensy 4.1 raw event logger for POST-PROCESSED timing
//
//  The board does NOT compute any GNSS time. It only:
//    - runs a free-running GPT1 tick counter (the ruler),
//    - logs the tick of every PPS edge,
//    - forwards every raw NMEA ToD sentence with the tick it was read,
//    - emits trigger pulses on a plain periodic timer and logs each edge's tick,
//    - logs the tick of every strobe / exposure-active edge.
//  A host script pairs ToD<->PPS and does a CENTERED sliding-window least-squares
//  fit to convert every logged tick to GNSS time - which removes the causal
//  extrapolation bias of an on-board trailing fit and can be re-run offline.
// =============================================================================


// -----------------------------------------------------------------------------
// 1. Timebase (GPT1 free-running counter)
// -----------------------------------------------------------------------------
#define TB_CLKSRC 1        // GPT clock source: 1 = perclk
#define TB_PERCLK_DIV 2    // perclk = ipg_clk(150 MHz) / 2 = 75 MHz -> 13.3 ns/tick
#define TB_PRESCALER 1     // GPT's own prescaler (1..4096); 1 = none


// -----------------------------------------------------------------------------
// Interrupt priorities (M7 implements only the top 4 bits, so keep >= 32 apart)
// -----------------------------------------------------------------------------
#define TB_IRQ_PRIORITY 32       // GPT1: trigger compare + rollover + (opt.) PPS capture
#define STROBE_IRQ_PRIORITY 64   // GPIO: PPS (software mode) + strobe read-back


// -----------------------------------------------------------------------------
// 2. PPS input
// -----------------------------------------------------------------------------
#define PPS_PIN 3            // must be 48 for hardware capture (see section 6)
#define PPS_RISING_EDGE 1    // 1 = rising edge active, 0 = falling


// -----------------------------------------------------------------------------
// 3. ToD input (raw NMEA from the INS - parsed OFFLINE, not here)
// -----------------------------------------------------------------------------
#define TOD_INPUT_SERIAL Serial2  // RX2 = pin 7, TX2 = pin 8
#define TOD_BAUD 115200
// No TIME_SCALE / GPS-offset / preceding-vs-following / lag window here: the host
// script parses the raw sentence, picks the PPS<->ToD convention by consistency,
// and applies any GPS/UTC offset. Nothing about time labels lives on the board.


// -----------------------------------------------------------------------------
// 4. Trigger outputs (plain periodic timer; every edge's exact tick is logged)
// -----------------------------------------------------------------------------
// Triggers are NOT GNSS-aligned - the board free-runs them off GPT1 and logs the
// exact compare tick of each edge, so the host derives their precise GNSS time.
// The rate is nominal_hz / round(nominal_hz / freq_hz), i.e. very close to freq_hz
// but tied to the crystal; that is fine because the logged ticks are exact.
#define TRIG_MAX 3

struct TriggerCfg {
	const char *name;
	uint8_t pin;
	double freq_hz;      // must be >= 1 Hz (period is a sub-1-second Q64 fraction internally)
	double phase_s;      // initial offset from the arbitrary start, [0, 1/freq); staggers channels
	uint32_t pulse_us;   // pulse width
	bool active_high;    // true = idle low, pulse high
};

static const TriggerCfg TRIG_CFG[TRIG_MAX] = {
	// name  pin  freq_hz  phase_s  pulse_us  active_high
	{ "FX10E", 24, 50.0, 0.0, 50, true },
	{ "JAI", 25, 1.0, 0.0, 50, true },
	{ "SPARE", 26, 1.0, 0.0, 1000, true },
};


// -----------------------------------------------------------------------------
// 5. Strobe / Exposure-Active read-back (BOTH edges logged, raw)
// -----------------------------------------------------------------------------
// The board logs every edge's tick and level; the host reconstructs exposure
// start/end, applies read-back path delays, glitch-filters, and matches frames.
#define STROBE_MAX 3

struct StrobeCfg {
	const char *name;
	uint8_t pin;
	bool active_high;    // used only to bias the idle pull; level is logged raw
};

static const StrobeCfg STROBE_CFG[STROBE_MAX] = {
	// name  pin  active_high
	{ "FX10E_EXP", 39, true },
	{ "JAI_EXP", 40, true },
	{ "AUX_EXP", 41, true },
};


// -----------------------------------------------------------------------------
// 6. PPS hardware capture (optional; GPT1_CAPTURE1)
// -----------------------------------------------------------------------------
// 1 = latch the PPS edge in GPT1_CAPTURE1 (pin 48, GPIO_EMC_24 ALT4) instead of a
//     GPIO ISR - drops PPS timestamp jitter to one tick. Requires PPS on pin 48.
// 0 = timestamp PPS inside the GPIO ISR (software). PPS is the input whose jitter
//     matters most for the fit, so this switch is the main data-quality lever.
#define USE_HW_CAPTURE 0

#define HW_PPS_DAISY_AUTO 0xFF
#define HW_PPS_DAISY HW_PPS_DAISY_AUTO   // 0 / 1, or AUTO to probe both at boot
#define HW_CAPTURE_SELFTEST 1            // power-on loopback self-test; refuses to arm on failure
#define HW_SELFTEST_DRIVE_PIN 31         // jumper this -> pin 48 for the self-test
#define HW_SELFTEST_TOL_US 2

#if USE_HW_CAPTURE && !HW_CAPTURE_SELFTEST && (HW_PPS_DAISY == HW_PPS_DAISY_AUTO)
	#error "With HW_CAPTURE_SELFTEST=0, HW_PPS_DAISY must be the concrete value the self-test reported, not AUTO"
#endif


// -----------------------------------------------------------------------------
// 7. Host log
// -----------------------------------------------------------------------------
// ISR -> loop rings, sized for the worst-case host stall (USB CDC blocks up to
// 120 ms when the host is attached but not reading). Each stream carries its own
// sequence number, so a gap in the log is detectable and drops are counted.
#define PPS_RING_SIZE 32            // PPS ticks (1 Hz -> tiny)
#define EVT_RING_SIZE 4096          // trigger + strobe edges (~1 s at 330 Hz x 3 x 2 edges)
#define DRAIN_PER_LOOP 96           // max lines emitted per stream per loop() pass
