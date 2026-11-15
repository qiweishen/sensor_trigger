#pragma once
#include <Arduino.h>

// =============================================================================
//  SensorSync-Logger - Teensy 4.1 configuration
//  Arduino GPIO numbers, not schematic terminal numbers.
// =============================================================================

// timebase: GPT1 on perclk = ipg 150 MHz / 2 = 75 MHz (13.3 ns/tick); no IntervalTimer
#define TB_PERCLK_DIV 2

// IRQ priority, lower = more urgent: GPT1 rollover > PWM > GPIO
#define TB_IRQ_PRIORITY 32
#define PWM_IRQ_PRIORITY 48
#define GPIO_IRQ_PRIORITY 64

// INS: PPS rising edge + raw NMEA (115200, keep)
#define PPS_PIN 19
#define TOD_SERIAL Serial1	// RX1 = pin 0
#define TOD_BAUD 115200

// triggers: FX pair = ch 0/1 (FlexPWM2 SM0), JAI pair = ch 2/3 (SM3); one rate per pair
#define FX_DEFAULT_HZ 50.0
#define JAI_DEFAULT_HZ 0.0	// 0 = pair off
#define PULSE_US 50

struct TriggerCfg {
	const char *name;
	uint8_t pin;
	bool enabled;			  // false: pin held LOW, no T records
};
static constexpr TriggerCfg TRIG_CFG[4] = {
	{ "FX10E-1", 33, true },  // SM0 B
	{ "FX10E-2", 4, false },  // SM0 A; not installed
	{ "JAI-1", 37, true },	  // SM3 B
	{ "JAI-2", 36, false },	  // SM3 A; not installed
};

// exposure-active inputs; active_high = line level during exposure
struct StrobeCfg {
	const char *name;
	uint8_t pin;
	bool active_high;
};
static constexpr StrobeCfg STROBE_CFG[4] = {
	{ "FX10E_EXP-1", 18, true },
	{ "FX10E_EXP-2", 10, true },
	{ "JAI_EXP-1", 14, false },	 // sinks line during exposure
	{ "JAI_EXP-2", 15, false },	 // same model; not yet verified
};

#define EVENT_QUEUE_SIZE 1024	 // ISR -> loop
#define STOP_TAIL_MS 200		 // after PWM off, keep exposure capture for the last frame
#define HEALTH_MS 5000

// activity LEDs: PPS, NMEA, FX trigger, FX exposure, JAI trigger, JAI exposure
static constexpr uint8_t LED_PINS[6] = { 2, 3, 6, 5, 8, 7 };
#define LED_HOLD_MS 10
