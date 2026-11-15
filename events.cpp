#include "events.h"

#include "config.h"
#include "timebase.h"


namespace events {
	static Event g_queue[EVENT_QUEUE_SIZE];
	static volatile uint16_t g_head = 0, g_tail = 0;
	static uint32_t g_seq[3][4];
	static uint32_t g_drops = 0;
	static volatile bool g_capture = false;

	static void ppsIsr() {
		const uint64_t tick = timebase::now();
		if (g_capture)
			push(PPS, 0, HIGH, tick);
	}

	template<uint8_t ch>
	static void strobeIsr() {
		const uint64_t tick = timebase::now();
		if (g_capture)
			push(STROBE, ch, digitalReadFast(STROBE_CFG[ch].pin), tick);
	}

	void begin() {
		pinMode(PPS_PIN, INPUT_PULLDOWN);
		attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsIsr, RISING);
		void (*const isr[4])() = { strobeIsr<0>, strobeIsr<1>, strobeIsr<2>, strobeIsr<3> };
		for (uint8_t ch = 0; ch < 4; ++ch) {
			pinMode(STROBE_CFG[ch].pin, INPUT);	 // driven by board buffers
			attachInterrupt(digitalPinToInterrupt(STROBE_CFG[ch].pin), isr[ch], CHANGE);
		}
		NVIC_SET_PRIORITY(IRQ_GPIO6789, GPIO_IRQ_PRIORITY);
	}

	void capture(bool on) {
		g_capture = on;
	}

	void reset() {
		g_head = g_tail = 0;
		memset(g_seq, 0, sizeof(g_seq));
		g_drops = 0;
	}

	void push(Kind kind, uint8_t ch, uint8_t level, uint64_t tick) {
		__disable_irq();  // PWM ISR may preempt a GPIO ISR
		const uint32_t seq = ++g_seq[kind][ch];
		const uint16_t next = (g_head + 1) % EVENT_QUEUE_SIZE;
		if (next == g_tail) {
			++g_drops;
		} else {
			g_queue[g_head] = { kind, ch, level, seq, tick };
			g_head = next;
		}
		__enable_irq();
	}

	bool pop(Event &e) {
		if (g_tail == g_head)
			return false;  // empty check unmasked: idle loop never blocks IRQs
		__disable_irq();
		e = g_queue[g_tail];
		g_tail = (g_tail + 1) % EVENT_QUEUE_SIZE;
		__enable_irq();
		return true;
	}

	uint32_t count(Kind kind, uint8_t ch) {
		return g_seq[kind][ch];
	}

	uint32_t drops() {
		return g_drops;
	}
}  // namespace events
