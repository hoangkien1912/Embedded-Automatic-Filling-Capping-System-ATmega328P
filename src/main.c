/* Cac tram chay doc lap, khong dung delay. Input tac dong o muc 0. */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include "gpio.h"

#include <avr/interrupt.h>
#include <util/atomic.h>

#define AUTO_OFF_TIME_MS    30000UL
#define HEAD_CONFIRM_MS     200UL
#define DEBOUNCE_MS         20U
#define INPUT_COUNT         7U

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
static timer_state_t   timer_state  = TM_RESET;

static uint32_t head_started_at;
static uint32_t timer_started_at;

static uint8_t head_seen_i2;
static uint8_t head_seen_i3;
static uint8_t flag_30s;

enum { REJECT, FILL, CAP, LABEL, STATION_COUNT };
typedef struct {
    station_state_t state;
    uint8_t queue;
    uint32_t started_at;
} station_t;
static station_t station[STATION_COUNT];
static const uint16_t duration[STATION_COUNT] = { 1000, 5000, 2000, 2000 };

static uint8_t input_state[INPUT_COUNT];
static uint8_t input_counter[INPUT_COUNT];
static uint32_t last_sample_ms;

#define I0_START   (input_state[GPIO_INPUT_START])
#define I1_STOP    (input_state[GPIO_INPUT_STOP])
#define I2         (input_state[GPIO_INPUT_BOTTLE_LOWER])
#define I3         (input_state[GPIO_INPUT_BOTTLE_UPPER])

#define SYS_ON     (main_state == MAIN_SYS_ON)

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
	TCCR1B = _BV(WGM12);
	TCNT1 = 0;
	OCR1A = (uint16_t)(F_CPU / 64UL / 1000UL - 1UL);
	TIFR1 = _BV(OCF1A);
	TIMSK1 = _BV(OCIE1A);
	TCCR1B |= _BV(CS11) | _BV(CS10);
}

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

static void queue_push(uint8_t *queue)
{
	if (*queue < 255U) {
		(*queue)++;
	}
}

static void outputs_off(void)
{
    for (uint8_t i = 0; i <= GPIO_OUTPUT_REJECT_PUSHER; i++) {
        gpio_write_output((gpio_output_t)i, 0);
    }
}

static void system_off(void)
{
	outputs_off();
	main_state   = MAIN_SYS_OFF;
	head_state   = HEAD_IDLE;

	timer_state  = TM_RESET;
	flag_30s     = 0;
    for (uint8_t i = 0; i < STATION_COUNT; i++) {
        station[i].state = ST_IDLE;
        station[i].queue = 0;
    }
}

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
		if (I2 || I3) {
			timer_state = TM_RESET;
		} else if ((uint32_t)(now - timer_started_at) >= AUTO_OFF_TIME_MS) {
			flag_30s = 1;
			timer_state = TM_TRIGGER;
		}
		break;

	case TM_TRIGGER:

		break;
	}
}

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
				queue_push(&station[FILL].queue);
			} else if (head_seen_i2) {
				queue_push(&station[REJECT].queue);
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

static void station_output(uint8_t index, uint8_t active)
{
    switch (index) {
    case REJECT:
        gpio_write_output(GPIO_OUTPUT_REJECT_PUSHER, active);
        break;
    case FILL:
        gpio_write_output(GPIO_OUTPUT_FILL_VALVE, active);
        gpio_write_output(GPIO_OUTPUT_FILL_STOPPER, active);
        break;
    case CAP:
        gpio_write_output(GPIO_OUTPUT_CAP_HEAD, active);
        gpio_write_output(GPIO_OUTPUT_CAP_STOPPER, active);
        break;
    case LABEL:
        gpio_write_output(GPIO_OUTPUT_LABELER, active);
        break;
    }
}

/* Moi tram giu trang thai rieng, cac tram van chay song song. */
static void stations_update(uint32_t now)
{
    for (uint8_t i = 0; i < STATION_COUNT; i++) {
        station_t *s = &station[i];
        uint8_t sensor = (i == REJECT) ? I2 : input_state[GPIO_INPUT_FILL_POSITION + i - FILL];
        switch (s->state) {
        case ST_IDLE:
            if (SYS_ON && s->queue && (i == REJECT || sensor)) {
                s->queue--;
                station_output(i, 1);
                s->started_at = now;
                s->state = ST_DOING;
            }
            break;
        case ST_DOING:
            if ((uint32_t)(now - s->started_at) >= duration[i]) {
                station_output(i, 0);
                if (i == FILL || i == CAP) queue_push(&station[i + 1].queue);
                s->state = ST_WAIT_CLEAR;
            }
            break;
        case ST_WAIT_CLEAR:
            if (!sensor) s->state = ST_IDLE;
            break;
        }
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

		timer_fsm(now);
		main_fsm();

		head_fsm(now);
		stations_update(now);
	}
}



