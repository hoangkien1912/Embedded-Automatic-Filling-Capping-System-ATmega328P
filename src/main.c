/*
 * De so 3 - He thong chiet rot, dong nap, dan nhan tu dong (ATmega328P)
 *
 * Cac FSM chay song song (moi FSM la 1 switch-case):
 *   1. MAIN SYSTEM   (Q0)      : SYS_OFF <-> SYS_ON
 *   2. HEAD          (I2, I3)  : bat chai o dau bang tai, phan loai 1 lan / chai
 *   3. DAY CHAI LOI  (Q6)      : IDLE -> DOING(1 s) -> WAIT_CLEAR -> IDLE
 *   4. CHIET ROT     (Q1, Q2)  : IDLE -> DOING(5 s) -> WAIT_CLEAR -> IDLE
 *   5. DONG NAP      (Q3, Q4)  : IDLE -> DOING(2 s) -> WAIT_CLEAR -> IDLE
 *   6. DAN NHAN      (Q5)      : IDLE -> DOING(2 s) -> WAIT_CLEAR -> IDLE
 *   7. AUTO OFF 30 s (timer)   : TM_RESET -> TM_COUNTING -> TM_TRIGGER
 *
 * Bien nho (hang doi chai) cho tung tram:
 *   reject_queue : chai loi dang cho day ra
 *   fill_queue   : chai hop le dang cho toi vi tri chiet rot (I4)
 *   cap_queue    : chai da rot xong, dang cho toi vi tri dong nap (I5)
 *   label_queue  : chai da dong nap, dang cho toi vi tri dan nhan (I6)
 * Tram chi kich hoat khi hang doi cua no > 0 VA cam bien vi tri tac dong,
 * nen co nhieu chai lien tuc thi cac tram chay doc lap nhau (pipeline).
 *
 * Input: I0..I6 (da debounce), logic duong: nhan / co tin hieu = 1.
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include "gpio.h"

#include <avr/interrupt.h>
#include <util/atomic.h>

#define REJECT_TIME_MS      1000UL
#define FILL_TIME_MS        5000UL
#define CAP_TIME_MS         2000UL
#define LABEL_TIME_MS       2000UL
#define AUTO_OFF_TIME_MS    30000UL

/* Cua so quan sat o dau bang tai: trong khoang nay ghi nho I2/I3 da tung bat.
 * Mo phong bang tay co the tang len ~1000 de bam I2, I3 lech nhau cung duoc. */
#define HEAD_CONFIRM_MS     200UL

#define DEBOUNCE_MS         20U
#define INPUT_COUNT         7U

/* ---------- Trang thai ---------- */
typedef enum {
	MAIN_SYS_OFF = 0,
	MAIN_SYS_ON
} main_state_t;

typedef enum {
	HEAD_IDLE = 0,
	HEAD_CONFIRM,
	HEAD_WAIT_CLEAR
} head_state_t;

typedef enum {
	ST_IDLE = 0,
	ST_DOING,
	ST_WAIT_CLEAR
} station_state_t;

typedef enum {
	TM_RESET = 0,
	TM_COUNTING,
	TM_TRIGGER
} timer_state_t;

static volatile uint32_t system_millis;

static main_state_t    main_state   = MAIN_SYS_OFF;
static head_state_t    head_state   = HEAD_IDLE;
static station_state_t reject_state = ST_IDLE;
static station_state_t fill_state   = ST_IDLE;
static station_state_t cap_state    = ST_IDLE;
static station_state_t label_state  = ST_IDLE;
static timer_state_t   timer_state  = TM_RESET;

static uint32_t head_started_at;
static uint32_t reject_started_at;
static uint32_t fill_started_at;
static uint32_t cap_started_at;
static uint32_t label_started_at;
static uint32_t timer_started_at;

static uint8_t head_seen_i2;
static uint8_t head_seen_i3;
static uint8_t flag_30s;

/* Bien nho: so chai dang cho o moi tram */
static uint8_t reject_queue;
static uint8_t fill_queue;
static uint8_t cap_queue;
static uint8_t label_queue;

/* Input sau debounce, chi so theo gpio_input_t */
static uint8_t input_state[INPUT_COUNT];
static uint8_t input_counter[INPUT_COUNT];
static uint32_t last_sample_ms;

#define I0_START   (input_state[GPIO_INPUT_START])
#define I1_STOP    (input_state[GPIO_INPUT_STOP])
#define I2         (input_state[GPIO_INPUT_BOTTLE_LOWER])
#define I3         (input_state[GPIO_INPUT_BOTTLE_UPPER])
#define I4         (input_state[GPIO_INPUT_FILL_POSITION])
#define I5         (input_state[GPIO_INPUT_CAP_POSITION])
#define I6         (input_state[GPIO_INPUT_LABEL_POSITION])

#define SYS_ON     (main_state == MAIN_SYS_ON)

/* ---------- Timer 1 ms ---------- */
ISR(TIMER1_COMPA_vect)
{
	system_millis++;
}

static uint32_t millis_now(void)
{
	uint32_t now;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		now = system_millis;
	}

	return now;
}

static void timer1_init(void)
{
	TCCR1A = 0;
	TCCR1B = _BV(WGM12);                         /* CTC */
	TCNT1 = 0;
	OCR1A = (uint16_t)(F_CPU / 64UL / 1000UL - 1UL); /* 1 ms, prescaler 64 */
	TIFR1 = _BV(OCF1A);
	TIMSK1 = _BV(OCIE1A);
	TCCR1B |= _BV(CS11) | _BV(CS10);             /* clk/64 */
}

/* ---------- Input (debounce moi 1 ms) ---------- */
static void inputs_update(uint32_t now)
{
	uint8_t i;

	if (now == last_sample_ms) {
		return;
	}
	last_sample_ms = now;

	for (i = 0; i < INPUT_COUNT; i++) {
		uint8_t raw = gpio_read_input((gpio_input_t)i);

		if (raw == input_state[i]) {
			input_counter[i] = 0;
		} else if (++input_counter[i] >= DEBOUNCE_MS) {
			input_state[i] = raw;
			input_counter[i] = 0;
		}
	}
}

/* ---------- Bien nho ---------- */
static void queue_push(uint8_t *queue)
{
	if (*queue < 255U) {
		(*queue)++;
	}
}

/* ---------- Output ---------- */
static void outputs_off(void)
{
	gpio_write_output(GPIO_OUTPUT_CONVEYOR, 0);
	gpio_write_output(GPIO_OUTPUT_FILL_VALVE, 0);
	gpio_write_output(GPIO_OUTPUT_FILL_STOPPER, 0);
	gpio_write_output(GPIO_OUTPUT_CAP_HEAD, 0);
	gpio_write_output(GPIO_OUTPUT_CAP_STOPPER, 0);
	gpio_write_output(GPIO_OUTPUT_LABELER, 0);
	gpio_write_output(GPIO_OUTPUT_REJECT_PUSHER, 0);
}

/* SYS_OFF: tat het output, dua moi FSM con ve IDLE, xoa het hang doi */
static void system_off(void)
{
	outputs_off();
	main_state   = MAIN_SYS_OFF;
	head_state   = HEAD_IDLE;
	reject_state = ST_IDLE;
	fill_state   = ST_IDLE;
	cap_state    = ST_IDLE;
	label_state  = ST_IDLE;
	timer_state  = TM_RESET;
	flag_30s     = 0;
	reject_queue = 0;
	fill_queue   = 0;
	cap_queue    = 0;
	label_queue  = 0;
}

/* ---------- 1. MAIN SYSTEM (Q0) ---------- */
static void main_fsm(void)
{
	switch (main_state) {
	case MAIN_SYS_OFF:
		if (I0_START && !I1_STOP) {
			gpio_write_output(GPIO_OUTPUT_CONVEYOR, 1);
			main_state = MAIN_SYS_ON;
		}
		break;

	case MAIN_SYS_ON:
		if (I1_STOP || flag_30s) {
			system_off();
		}
		break;
	}
}

/* ---------- 7. AUTO OFF 30 s ---------- */
static void timer_fsm(uint32_t now)
{
	if (!SYS_ON) {
		timer_state = TM_RESET;
		flag_30s = 0;
		return;
	}

	switch (timer_state) {
	case TM_RESET:
		flag_30s = 0;
		if (!I2 && !I3) {
			timer_started_at = now;
			timer_state = TM_COUNTING;
		}
		break;

	case TM_COUNTING:
		if (I2 || I3) {                         /* co chai den */
			timer_state = TM_RESET;
		} else if ((uint32_t)(now - timer_started_at) >= AUTO_OFF_TIME_MS) {
			flag_30s = 1;
			timer_state = TM_TRIGGER;
		}
		break;

	case TM_TRIGGER:
		/* main_fsm thay flag_30s = 1 -> system_off() -> TM_RESET */
		break;
	}
}

/* ---------- 2. HEAD: phan loai chai o dau bang tai (I2, I3) ----------
 * Moi chai chi duoc danh gia 1 lan, roi cho I2 = I3 = 0 moi nhan chai tiep.
 *   I2 & I3        -> chai hop le  -> fill_queue++
 *   chi I2         -> chai loi     -> reject_queue++
 *   chi I3         -> bo qua
 */
static void head_fsm(uint32_t now)
{
	switch (head_state) {
	case HEAD_IDLE:
		if (SYS_ON && (I2 || I3)) {
			head_seen_i2 = I2;
			head_seen_i3 = I3;
			head_started_at = now;
			head_state = HEAD_CONFIRM;
		}
		break;

	case HEAD_CONFIRM:
		if (I2) {
			head_seen_i2 = 1;
		}
		if (I3) {
			head_seen_i3 = 1;
		}
		if ((uint32_t)(now - head_started_at) >= HEAD_CONFIRM_MS) {
			if (head_seen_i2 && head_seen_i3) {
				queue_push(&fill_queue);
			} else if (head_seen_i2) {
				queue_push(&reject_queue);
			}
			head_state = HEAD_WAIT_CLEAR;
		}
		break;

	case HEAD_WAIT_CLEAR:
		if (!I2 && !I3) {
			head_state = HEAD_IDLE;
		}
		break;
	}
}

/* ---------- 3. DAY CHAI LOI (Q6) ---------- */
static void reject_fsm(uint32_t now)
{
	switch (reject_state) {
	case ST_IDLE:
		if (SYS_ON && reject_queue > 0) {
			reject_queue--;
			gpio_write_output(GPIO_OUTPUT_REJECT_PUSHER, 1);
			reject_started_at = now;
			reject_state = ST_DOING;
		}
		break;

	case ST_DOING:
		if ((uint32_t)(now - reject_started_at) >= REJECT_TIME_MS) {
			gpio_write_output(GPIO_OUTPUT_REJECT_PUSHER, 0);
			reject_state = ST_WAIT_CLEAR;
		}
		break;

	case ST_WAIT_CLEAR:
		if (!I2) {
			reject_state = ST_IDLE;
		}
		break;
	}
}

/* ---------- 4. CHIET ROT (Q1, Q2) ---------- */
static void fill_fsm(uint32_t now)
{
	switch (fill_state) {
	case ST_IDLE:
		if (SYS_ON && fill_queue > 0 && I4) {
			fill_queue--;
			gpio_write_output(GPIO_OUTPUT_FILL_VALVE, 1);
			gpio_write_output(GPIO_OUTPUT_FILL_STOPPER, 1);
			fill_started_at = now;
			fill_state = ST_DOING;
		}
		break;

	case ST_DOING:
		if ((uint32_t)(now - fill_started_at) >= FILL_TIME_MS) {
			gpio_write_output(GPIO_OUTPUT_FILL_VALVE, 0);
			gpio_write_output(GPIO_OUTPUT_FILL_STOPPER, 0);
			queue_push(&cap_queue);             /* chai di tiep toi tram dong nap */
			fill_state = ST_WAIT_CLEAR;
		}
		break;

	case ST_WAIT_CLEAR:
		if (!I4) {
			fill_state = ST_IDLE;
		}
		break;
	}
}

/* ---------- 5. DONG NAP (Q3, Q4) ---------- */
static void cap_fsm(uint32_t now)
{
	switch (cap_state) {
	case ST_IDLE:
		if (SYS_ON && cap_queue > 0 && I5) {
			cap_queue--;
			gpio_write_output(GPIO_OUTPUT_CAP_HEAD, 1);
			gpio_write_output(GPIO_OUTPUT_CAP_STOPPER, 1);
			cap_started_at = now;
			cap_state = ST_DOING;
		}
		break;

	case ST_DOING:
		if ((uint32_t)(now - cap_started_at) >= CAP_TIME_MS) {
			gpio_write_output(GPIO_OUTPUT_CAP_HEAD, 0);
			gpio_write_output(GPIO_OUTPUT_CAP_STOPPER, 0);
			queue_push(&label_queue);           /* chai di tiep toi tram dan nhan */
			cap_state = ST_WAIT_CLEAR;
		}
		break;

	case ST_WAIT_CLEAR:
		if (!I5) {
			cap_state = ST_IDLE;
		}
		break;
	}
}

/* ---------- 6. DAN NHAN (Q5) ---------- */
static void label_fsm(uint32_t now)
{
	switch (label_state) {
	case ST_IDLE:
		if (SYS_ON && label_queue > 0 && I6) {
			label_queue--;
			gpio_write_output(GPIO_OUTPUT_LABELER, 1);
			label_started_at = now;
			label_state = ST_DOING;
		}
		break;

	case ST_DOING:
		if ((uint32_t)(now - label_started_at) >= LABEL_TIME_MS) {
			gpio_write_output(GPIO_OUTPUT_LABELER, 0);
			label_state = ST_WAIT_CLEAR;
		}
		break;

	case ST_WAIT_CLEAR:
		if (!I6) {
			label_state = ST_IDLE;
		}
		break;
	}
}

int main(void)
{
	gpio_init();
	outputs_off();
	timer1_init();
	sei();

	while (1) {
		uint32_t now = millis_now();

		inputs_update(now);

		timer_fsm(now);     /* 7. cap nhat co 30 s                 */
		main_fsm();         /* 1. START / STOP / co 30 s           */

		head_fsm(now);      /* 2. phan loai chai o dau bang tai    */
		reject_fsm(now);    /* 3. Q6                               */
		fill_fsm(now);      /* 4. Q1, Q2                           */
		cap_fsm(now);       /* 5. Q3, Q4                           */
		label_fsm(now);     /* 6. Q5                               */
	}
}
