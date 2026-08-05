#pragma once
#include <Arduino.h>

// =============================================================================
//  SensorSync-Logger - Teensy 4.1 raw event logger for post-processed timing.
//
//  No GNSS time on the board. It only:
//    - runs a free-running GPT1 tick counter (the ruler),
//    - logs the tick of every PPS edge,
//    - forwards every raw NMEA ToD sentence with its read-out tick,
//    - emits free-running trigger pulses, logging each edge's tick,
//    - logs the tick of every strobe / exposure-active edge.
//  Host side: postprocess.py pairs ToD<->PPS, fits tick->GNSS with a centered
//  sliding window (no causal extrapolation bias; re-runnable offline).
// =============================================================================


// -----------------------------------------------------------------------------
// 1. Timebase (GPT1 free-running counter)
// -----------------------------------------------------------------------------
#define TB_CLKSRC 1        // GPT clock source: 1 = perclk
#define TB_PERCLK_DIV 2    // perclk = ipg_clk(150 MHz) / 2 = 75 MHz -> 13.3 ns/tick
#define TB_PRESCALER 1     // GPT's own prescaler (1..4096); 1 = none


// -----------------------------------------------------------------------------
// Interrupt priorities (M7 uses only the top 4 bits -> keep >= 32 apart)
// -----------------------------------------------------------------------------
#define TB_IRQ_PRIORITY 32       // GPT1: trigger compare + rollover + (opt.) PPS capture
#define STROBE_IRQ_PRIORITY 64   // GPIO: PPS (software mode) + strobe read-back


// -----------------------------------------------------------------------------
// 2. PPS input
// -----------------------------------------------------------------------------
#define PPS_PIN 3            // must be 48 for hardware capture (section 6)
#define PPS_RISING_EDGE 1    // 1 = rising edge active, 0 = falling


// -----------------------------------------------------------------------------
// 3. ToD input (raw NMEA from the INS - parsed offline, not here)
// -----------------------------------------------------------------------------
#define TOD_INPUT_SERIAL Serial2  // RX2 = pin 7, TX2 = pin 8
#define TOD_BAUD 115200
// no time-scale / GPS-offset / pairing-convention config on the board: the host
// parses the raw sentence and decides all of that offline


// -----------------------------------------------------------------------------
// 4. Trigger outputs (free-running periodic timer; every edge tick logged)
// -----------------------------------------------------------------------------
// NOT GNSS-aligned: rate = nominal_hz / round(nominal_hz / freq_hz), tied to the
// crystal. Fine, because the logged ticks are exact and the host converts them.
// begin() validates each channel (>= 1 Hz, period >= 4x ISR margin, phase >= 0)
// and disables invalid ones loudly.
#define TRIG_MAX 3

struct TriggerCfg {
	const char *name;
	uint8_t pin;
	double freq_hz;      // >= 1 Hz (whole-tick period)
	double phase_s;      // start stagger, folded modulo the period; >= 0
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
// 5. Strobe / Exposure-Active read-back (both edges logged, raw)
// -----------------------------------------------------------------------------
// Host reconstructs exposure start/end, corrects path delays, matches frames.
#define STROBE_MAX 3

struct StrobeCfg {
	const char *name;
	uint8_t pin;
	bool active_high;    // biases the idle pull only; level is logged raw
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
// 1 = latch PPS in GPT1_CAPTURE1 (pin 48, GPIO_EMC_24 ALT4): one-tick jitter.
// 0 = timestamp PPS in the GPIO ISR (software): ~0.3-2 us jitter.
// PPS jitter bounds the offline fit -> this is the main data-quality lever.
#define USE_HW_CAPTURE 0

#define HW_PPS_DAISY_AUTO 0xFF
#define HW_PPS_DAISY HW_PPS_DAISY_AUTO   // 0 / 1, or AUTO to probe both at boot
#define HW_CAPTURE_SELFTEST 1            // power-on loopback test; refuses to arm on failure
#define HW_SELFTEST_DRIVE_PIN 31         // jumper this -> pin 48 for the self-test
#define HW_SELFTEST_TOL_US 2

#if USE_HW_CAPTURE && !HW_CAPTURE_SELFTEST && (HW_PPS_DAISY == HW_PPS_DAISY_AUTO)
	#error "With HW_CAPTURE_SELFTEST=0, HW_PPS_DAISY must be the concrete value the self-test reported, not AUTO"
#endif


// -----------------------------------------------------------------------------
// 7. Host log
// -----------------------------------------------------------------------------
// ISR -> loop rings, sized for the worst-case host stall (USB CDC blocks up to
// ~120 ms when attached but unread). Per-stream seq numbers make drops visible.
#define PPS_RING_SIZE 32            // PPS ticks (1 Hz -> tiny)
#define EVT_RING_SIZE 4096          // trigger + strobe edges; ~19 s at the default 50 Hz config
#define DRAIN_PER_LOOP 96           // max lines emitted per stream per loop() pass
