#include "gpio.h"

#include <avr/io.h>

void gpio_init(void)
{
    /* PD2-PD7, PB0: input voi pull-up. */
    DDRD  &= 0x03;
    PORTD |= 0xFC;
    DDRB  &= 0xFE;
    PORTB |= 0x01;

    /* Output: ghi muc thap truoc, sau do moi dat huong ra */
    PORTB &= 0xC1;
    DDRB  |= 0x3E;
    PORTC &= 0xFC;
    DDRC  |= 0x03;
}

uint8_t gpio_read_input(gpio_input_t input)
{
    uint8_t pin_is_high;

    switch (input) {
    case GPIO_INPUT_START:
        pin_is_high = (PIND & _BV(PD2)) != 0;
        break;
    case GPIO_INPUT_STOP:
        pin_is_high = (PIND & _BV(PD3)) != 0;
        break;
    case GPIO_INPUT_BOTTLE_LOWER:
        pin_is_high = (PIND & _BV(PD4)) != 0;
        break;
    case GPIO_INPUT_BOTTLE_UPPER:
        pin_is_high = (PIND & _BV(PD5)) != 0;
        break;
    case GPIO_INPUT_FILL_POSITION:
        pin_is_high = (PIND & _BV(PD6)) != 0;
        break;
    case GPIO_INPUT_CAP_POSITION:
        pin_is_high = (PIND & _BV(PD7)) != 0;
        break;
    case GPIO_INPUT_LABEL_POSITION:
        pin_is_high = (PINB & _BV(PB0)) != 0;
        break;
    default:
        return 0;
    }

    return (uint8_t)!pin_is_high;
}

void gpio_write_output(gpio_output_t output, uint8_t active)
{
    switch (output) {
    case GPIO_OUTPUT_CONVEYOR:
        if (active) PORTC |= _BV(PC1); else PORTC &= (uint8_t)~_BV(PC1);
        break;
    case GPIO_OUTPUT_FILL_VALVE:
        if (active) PORTC |= _BV(PC0); else PORTC &= (uint8_t)~_BV(PC0);
        break;
    case GPIO_OUTPUT_FILL_STOPPER:
        if (active) PORTB |= _BV(PB5); else PORTB &= (uint8_t)~_BV(PB5);
        break;
    case GPIO_OUTPUT_CAP_HEAD:
        if (active) PORTB |= _BV(PB4); else PORTB &= (uint8_t)~_BV(PB4);
        break;
    case GPIO_OUTPUT_CAP_STOPPER:
        if (active) PORTB |= _BV(PB3); else PORTB &= (uint8_t)~_BV(PB3);
        break;
    case GPIO_OUTPUT_LABELER:
        if (active) PORTB |= _BV(PB2); else PORTB &= (uint8_t)~_BV(PB2);
        break;
    case GPIO_OUTPUT_REJECT_PUSHER:
        if (active) PORTB |= _BV(PB1); else PORTB &= (uint8_t)~_BV(PB1);
        break;
    default:
        break;
    }
}

