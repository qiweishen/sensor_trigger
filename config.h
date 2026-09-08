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
#define TB_CLKSRC 1		 // GPT clock source: 1 = perclk
#define TB_PERCLK_DIV 2	 // perclk = ipg_clk(150 MHz) / 2 = 75 MHz -> 13.3 ns/tick
#define TB_PRESCALER 1	 // GPT's own prescaler (1..4096); 1 = none


// -----------------------------------------------------------------------------
// Interrupt priorities: GPT1 must outrank the GPIO sampling ISR.
// -----------------------------------------------------------------------------
#define TB_IRQ_PRIORITY 32		// GPT1: trigger scheduler + rollover
#define STROBE_IRQ_PRIORITY 64	// GPIO: PPS (software mode) + strobe read-back


// -----------------------------------------------------------------------------
// 2. PPS input
// -----------------------------------------------------------------------------
#define PPS_PIN 19
#define PPS_RISING_EDGE 1  // 1 = rising edge active, 0 = falling


// -----------------------------------------------------------------------------
// 3. ToD input (raw NMEA from the INS - parsed offline, not here)
// -----------------------------------------------------------------------------
#define TOD_INPUT_SERIAL Serial1  // RX1 = pin 0, TX1 = pin 1
#define TOD_BAUD 115200
// no time-scale / GPS-offset / pairing-convention config on the board: the host
// parses the raw sentence and decides all of that offline


// -----------------------------------------------------------------------------
// 4. Trigger outputs (free-running periodic timer; every edge tick logged)
// -----------------------------------------------------------------------------
// NOT GNSS-aligned: rate = nominal_hz / round(nominal_hz / freq_hz), tied to the
// crystal. Edge ticks are sampled immediately after the GPIO write.
// begin() validates each channel (>= 1 Hz, period >= 4x ISR margin, phase >= 0)
// and disables invalid ones loudly.

struct TriggerCfg {
	const char *name;
	uint8_t pin;
	double freq_hz;		// >= 1 Hz (whole-tick period)
	double phase_s;		// start stagger, folded modulo the period; >= 0
	uint32_t pulse_us;	// pulse width
	bool active_high;	// true = idle low, pulse high
};

static constexpr TriggerCfg TRIG_CFG[] = {
	// name  pin  freq_hz  phase_s  pulse_us  active_high
	{ "FX10E-1", 33, 50.0, 0.0, 50, true },
	{ "FX10E-2", 4, 0.0, 0.0, 50, true },
	{ "JAI-1", 37, 0.0, 0.0, 50, true },
	{ "JAI-2", 36, 0.0, 0.0, 50, true }
};
// Keep each wired pin explicit; use freq=0 to disable it, not an empty entry.
static constexpr uint8_t TRIG_MAX = sizeof(TRIG_CFG) / sizeof(TRIG_CFG[0]);


// -----------------------------------------------------------------------------
// 5. Strobe / Exposure-Active read-back (both edges logged, raw)
// -----------------------------------------------------------------------------
// Host reconstructs exposure start/end, corrects path delays, matches frames.

struct StrobeCfg {
	const char *name;
	uint8_t pin;
	bool active_high;  // exposure polarity for post-processing; verify at the camera
};

static constexpr StrobeCfg STROBE_CFG[] = {
	// name  pin  active_high
	{ "FX10E_EXP-1", 18, true },
	{ "FX10E_EXP-2", 10, true },
	{ "JAI_EXP-1", 14, true },
	{ "JAI_EXP-2", 15, true }
};
static constexpr uint8_t STROBE_MAX = sizeof(STROBE_CFG) / sizeof(STROBE_CFG[0]);
// The schematic's external pull-ups + non-inverting buffers drive HIGH on
// disconnected camera inputs. Internal MCU pulls cannot change that level.


// -----------------------------------------------------------------------------
// 6. Host log
// -----------------------------------------------------------------------------
// ISR -> loop rings, sized for the worst-case host stall (USB CDC blocks up to
// ~120 ms when attached but unread). Per-stream seq numbers make drops visible.
#define PPS_RING_SIZE 32	// PPS ticks (1 Hz -> tiny)
#define EVT_RING_SIZE 4096	// each event ring; USB output still drops immediately on backpressure
#define DRAIN_PER_LOOP 96	// max lines emitted per stream per loop() pass
