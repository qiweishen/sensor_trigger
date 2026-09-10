#pragma once
#include <Arduino.h>
#include "trigger_groups.h"

// =============================================================================
//  SensorSync-Logger - Teensy 4.1 raw event logger for post-processed timing.
//
//  No GNSS time on the board. It only:
//    - runs a free-running GPT1 tick counter (the ruler),
//    - logs the tick of every PPS edge,
//    - forwards every raw NMEA ToD sentence with its read-out tick,
//    - emits two groups of hardware PWM pulses, logging PWM ISR samples,
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
#define TB_IRQ_PRIORITY 32		// GPT1 rollover
#define PWM_IRQ_PRIORITY 48		// both PWM IRQs: serialized Trigger queue producers
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
// 4. Two hardware PWM groups (free-running, not GNSS-aligned)
// -----------------------------------------------------------------------------
// Group defaults persist until overridden by START freq=FX:...,JAI:....
// FX: >=20 Hz while representable; JAI: 1..10 Hz. Zero disables the whole group.
// The camera's permissible frame rate must also be respected by the caller.
static constexpr trigger::Rates TRIGGER_DEFAULT_RATES = {{50.0, 0.0}};
static constexpr uint32_t GROUP_PULSE_US[trigger::GROUP_COUNT] = {50, 50};

struct TriggerCfg {
    const char *name;
    uint8_t pin;
    bool active_high;
};
static constexpr TriggerCfg TRIG_CFG[] = {
    {"FX10E-1", 33, true}, // SM0 B
    {"FX10E-2",  4, true}, // SM0 A
    {"JAI-1",   37, true}, // SM3 B
    {"JAI-2",   36, true}  // SM3 A
};
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


// -----------------------------------------------------------------------------
// 7. Activity LEDs (schematic D211..D216, HIGH = on through 330 ohms to GND)
// -----------------------------------------------------------------------------
// Arduino GPIO numbers, not U201 terminal numbers. Keep this order: leds::poll()
// pairs these with PPS, RX bytes, FX trigger/exposure and JAI trigger/exposure.
static constexpr uint8_t LED_PINS[] = {
	2,  // D211 LED_PPS
	3,  // D212 LED_RS232
	6,  // D213 LED_TRIG_FX  (channels 0 + 1)
	5,  // D214 LED_EXP_FX   (channels 0 + 1)
	8,  // D215 LED_TRIG_JAI (channels 2 + 3)
	7   // D216 LED_EXP_JAI  (channels 2 + 3)
};
static constexpr uint8_t LED_MAX = sizeof(LED_PINS) / sizeof(LED_PINS[0]);
#define LED_HOLD_MS 10  // retriggerable visible activity indication, serviced in loop()
// D111 (+12V), D112 (+5V), D113 (+3V3) are wired to the rails; no GPIO control.
