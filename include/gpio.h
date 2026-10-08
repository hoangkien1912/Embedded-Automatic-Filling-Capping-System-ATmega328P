#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

typedef enum {
    GPIO_INPUT_START = 0,
    GPIO_INPUT_STOP,
    GPIO_INPUT_BOTTLE_LOWER,
    GPIO_INPUT_BOTTLE_UPPER,
    GPIO_INPUT_FILL_POSITION,
    GPIO_INPUT_CAP_POSITION,
    GPIO_INPUT_LABEL_POSITION
} gpio_input_t;

typedef enum {
    GPIO_OUTPUT_CONVEYOR = 0,
    GPIO_OUTPUT_FILL_VALVE,
    GPIO_OUTPUT_FILL_STOPPER,
    GPIO_OUTPUT_CAP_HEAD,
    GPIO_OUTPUT_CAP_STOPPER,
    GPIO_OUTPUT_LABELER,
    GPIO_OUTPUT_REJECT_PUSHER
} gpio_output_t;

/* Uno pinout: I0-I5 = D2-D7 (PD2-PD7), I6 = D8 (PB0).
 * Conveyor = A1 (PC1), fill valve = A0 (PC0),
 * fill stopper = D13 (PB5), cap head = D12 (PB4),
 * cap stopper = D11 (PB3), labeler = D10 (PB2), reject pusher = D9 (PB1).
 */
/* Inputs use internal pull-ups: a pressed switch/active sensor reads as 1. */
void gpio_init(void);
uint8_t gpio_read_input(gpio_input_t input);
void gpio_write_output(gpio_output_t output, uint8_t active);

#endif