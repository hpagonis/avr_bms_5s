/******************************
 * TODO
 * Read ADC values for cell 1 and 2
 * Compare values to target
 * Activate/Deactivate Discarge
 *
 * Author: Harris Pagonis
 ******************************/
#include <avr/io.h>
#include <avr/interrupt.h>

#define OV (4200L * 1024) / 5000
#define UV (3000L * 1024) / 5000
#define SLAVE_ADDRESS 1
#define CELL_NUMBER 2
#define DC1PIN PIND3
#define DC2PIN PIND4
#define LEDPIN PINB5

uint16_t cellv[CELL_NUMBER];
uint8_t dc_pin[CELL_NUMBER] = {PIND3, PIND4};

void transmit_voltage(uint16_t voltage);

int main() {
  // Initialize indicator LED
  DDRB |= _BV(LEDPIN);
  PORTB = _BV(LEDPIN);
  // Initialize DC pins
  for (int i=0; i<CELL_NUMBER; i++) {
    DDRD |= _BV(dc_pin[i]);
    if ( i != 0) PORTD |= _BV(dc_pin[i]);
  }

  // Initialize ADC
  // Set AVcc as voltage reference, enable ADC and interrupt
  // Set prescaler to 32 for 500KHz ADC clock
  ADMUX = _BV(REFS0);
  ADCSRA = _BV(ADEN) | _BV(ADPS2);

  // Initialize UART
  // Enable MPCM mode at 9600 baud 9N1
  UCSRA = _BV(MPCM);
  UCSRB = _BV(RXCIE) | _BV(RXEN) | _BV(TXEN) | _BV(UCSZ2) | _BV(TXB8);
  UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);
  UBRRL = 51;

  //sei();

  while(1) {
    // Read cell voltages
    for (uint8_t i = 0; i < CELL_NUMBER; i++) {
      ADMUX &= 0xF0;
      ADMUX |= i;
      ADCSRA |= _BV(ADSC);
      while (ADCSRA & _BV(ADSC));
      uint8_t *pcellv = &cellv[i];
      *pcellv = ADCL;
      pcellv++;
      *pcellv = ADCH;
      transmit_voltage(cellv[i]);

      if (cellv[i] > OV) {
        if (i == 0) {
          PORTD |= _BV(dc_pin[i]);
        } else {
          PORTD &= ~_BV(dc_pin[i]);
        }
      } else {
        if (i == 0) {
          PORTD &= ~_BV(dc_pin[i]);
        } else {
          PORTD |= _BV(dc_pin[i]);
        }
      }
    }
  }
}

void transmit_voltage(uint16_t voltage) {
  while (!(UCSRA & _BV(UDRE)));
  UDR = (uint8_t) voltage;
  while (!(UCSRA & _BV(UDRE)));
  UDR = (uint8_t) (voltage >> 8);
}

ISR(USART_RXC_vect) {
  // Determine if address or data
  if (UCSRB & _BV(RXB8)) {
    if (UDR == SLAVE_ADDRESS) {
      UCSRA = 0;
    }
  } else {
    // Add character to receive buffer
  }
}

ISR(USART_TXC_vect) {
}
