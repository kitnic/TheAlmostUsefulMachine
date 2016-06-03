/* 
 * motor.c
 * tinkerlog.com

 * motor output pin: PB0/OC0A

 * MOSFET
 * IRF1010N
 * VDSS = 55V
 * RDS = 12mOhm
 * ID = 85A
 * Pins  1 2 3
 *       G D S
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LED PB1
#define MOTOR PB0
#define END_STOP PB4

#define START_VOLTAGE 80

static volatile uint8_t selected_speed = 0;
static volatile uint8_t end_stop = 0;
static uint8_t act_speed = 0;
static uint8_t prescale = 0;

/*
 * Read speed selector pot, the end stop switch and set PWM value accordingly.
 * Actual motor speed is ramped up and down slightly.
 */
SIGNAL(ADC_vect) {
  end_stop = !(PINB & (1 << END_STOP));
  selected_speed = end_stop ? ADCH : 0;  
  if (act_speed != selected_speed) {
    if (prescale++ == 20) {
      prescale = 0;
      act_speed = (selected_speed > act_speed) ? act_speed + 1 : act_speed - 1;
    }
  }  
  OCR0A = (uint8_t)(act_speed * ((255.0 - START_VOLTAGE) / 256.0) + START_VOLTAGE);
}


int main(void) {

  uint8_t i;

  // enable outputs
  DDRB |= (1 << LED) | (1 << MOTOR);

  // enable input
  DDRB &= ~(1 << END_STOP);
  // enable pull up
  PORTB |= (1 << END_STOP);

  // f = fclk / n * 256 (n = prescaler)
  // 8MHz / (1 * 256) = 31250 Hz
  TCCR0A |=
    (1 << WGM01) |                 // fast-pwm
    (1 << WGM00) | (1 << COM0A1);  // non-inverting on OC0A
  TCCR0B |= (0 << CS02) | (0 << CS01) | (1 << CS00);   // prescaler 0
  
  // enable adc
  ADCSRA |= 
    (1 << ADEN) |   // enable ADC
    (1 << ADATE) |  // enable auto triggering
    (1 << ADIE) |   // enable ADC interrupt
    (1 << ADSC) |   // start conversion
    (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);        // prescaler 128 --> 62500 samples/s

  // select ADC channel
  ADMUX |= 
    (0 << REFS0) |  // select Vcc as reference
    (1 << ADLAR) |  // ADC output as 8-bit, left adjusted
    (1 << MUX0);    // select ADC1

  // intro
  for (i = 0; i < 5; i++) {
    PORTB |= (1 << LED);	
    _delay_ms(200);
    PORTB &= ~(1 << LED);
    _delay_ms(200);
  }

  // enable all interrupts
  sei();

  while (1) {
    if ((selected_speed > 0) && end_stop) {
      // if (end_stop) {
      // if (!(PINB & (1 << END_STOP))) {
      PORTB |= (1 << LED);
    }
    else {
      PORTB &= ~(1 << LED);
    }
  }
  
  return 0;
}
