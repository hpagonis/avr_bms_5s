/******************************
 * TODO
 * - Implement communication
 * - Minimize energy usage
 * 
 * Author: Harris Pagonis
 ******************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#define OV (4200L * 1024) / 5000
#define UV (3000L * 1024) / 5000
#define CELL_NUMBER 5
#define LEDPIN PINB5
#define CMD_SET_SLAVE_ADDRESS 0x10
#define CMD_READ_VOLTAGES 0x20
#define CMD_SET_OFFSETS 0x30

uint16_t cellv[CELL_NUMBER];
uint8_t dc_pin[CELL_NUMBER] = {PIND4, PIND3, PIND2, PIND5, PIND6};
int8_t offset[CELL_NUMBER];
uint8_t volatile address;
uint8_t EEMEM e_address = 3;
int8_t EEMEM e_offset[CELL_NUMBER] = {9, 7, -10, 14, 4};
uint8_t volatile cmd;

void transmit_voltage(uint16_t voltage);

int main() {
  // Initialize indicator LED
  DDRB |= _BV(LEDPIN);
  //PORTB = _BV(LEDPIN);
  // Initialize DC pins
  for (int i=0; i<CELL_NUMBER; i++) {
    DDRD |= _BV(dc_pin[i]);
    if ( i != 0) PORTD |= _BV(dc_pin[i]);
  }

  // Initialize Hysterisis Timer
  // around 30Hz
  TCCR0 = _BV(CS02) | _BV(CS00);
  TIMSK = _BV(TOIE0);

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
  UBRRL = 103;

  // Read settings from EEPROM
  address = eeprom_read_byte(&e_address);
  eeprom_read_block((void*)&offset, (const void*)&e_offset, CELL_NUMBER);

  sei();

  while(1) {
    // Read cell voltages
    for (uint8_t i = 0; i < CELL_NUMBER; i++) {
      ADMUX &= 0xF0;
      ADMUX |= CELL_NUMBER - i - 1;
      ADCSRA |= _BV(ADSC);
      while (ADCSRA & _BV(ADSC));
      uint8_t *pcellv = (uint8_t *)&cellv[i];
      *pcellv = ADCL;
      pcellv++;
      *pcellv = ADCH;
      cellv[i] += offset[i];

      if (cellv[i] > OV) {
        if (i == 0) {
          PORTD |= _BV(dc_pin[i]);
        } else {
          PORTD &= ~_BV(dc_pin[i]);
        }
      } 
    }

    if (cmd) {
      uint8_t cmd_part = cmd & 0xF0;
      uint8_t data_part = cmd & 0x0F;
      if (cmd_part == CMD_SET_SLAVE_ADDRESS) {
        address = data_part;
        eeprom_write_byte(&e_address, data_part);
      }
      if (cmd_part == CMD_SET_OFFSETS) {
        // TODO
      }
      if (cmd_part == CMD_READ_VOLTAGES) {
        for (uint8_t i = 0; i < CELL_NUMBER; i++) {
          transmit_voltage(cellv[i]);
        }
      }
      cmd = 0;
    }
  }
}

void transmit_voltage(uint16_t voltage) {
  while (!(UCSRA & _BV(UDRE)));
  UDR = (uint8_t) voltage;
  while (!(UCSRA & _BV(UDRE)));
  UDR = (uint8_t) (voltage >> 8);
}

ISR(TIMER0_OVF_vect) {
  static uint8_t counter = 0;
  counter++;
  if (counter == 15) {
    for (int i=1; i<CELL_NUMBER; i++) {
      PORTD |= _BV(dc_pin[i]);
    }
    PORTD &= ~_BV(dc_pin[0]);
    counter = 0;
  }
}

ISR(USART_RXC_vect) {
  // Determine if address or data
  if (UCSRB & _BV(RXB8)) {
    if (UDR == address) {
      UCSRA = 0;
    }
  } else {
    cmd = UDR;
    UCSRA = _BV(MPCM);
  }
}

ISR(USART_TXC_vect) {
}
