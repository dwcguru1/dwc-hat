/*****************************************************************************
*
* Atmel Corporation
*
* File              : USI_TWI_Slave.c
* Compiler          : IAR EWAAVR 2.28a/3.10a
* Revision          : $Revision: 1.7 $
* Date              : $Date: 26. mai 2004 11:35:34 $
* Updated by        : $Author: ltwa $
*
* Support mail      : avr@atmel.com
*
* Supported devices : All device with USI module can be used.
*                     The example is written for the ATmega169, ATtiny26 & ATtiny2313
*
* AppNote           : AVR312 - Using the USI module as a TWI slave
*
* Description       : Example showing how to use the USI_TWI drivers;
*                     Loops back received data.
*
* Changes for WinAVR by Frank Jonischkies, fjonisch@gmx.de, 11.08.2005
*
****************************************************************************/

#include <avr/pgmspace.h>
#include <util/delay.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include "USI_TWI_Slave.h"

// gcc doesn't know this and ledger's SDK cannot be compiled with Werror!
//#pragma GCC diagnostic error "-Werror"
#pragma GCC diagnostic error "-Wpedantic"
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wextra"

const int sinewave_length = 256;
const unsigned char sinewave_data[] PROGMEM = {
    0x80, 0x83, 0x86, 0x89, 0x8c, 0x8f, 0x92, 0x95, 0x98, 0x9c, 0x9f, 0xa2, 0xa5, 0xa8, 0xab, 0xae,
    0xb0, 0xb3, 0xb6, 0xb9, 0xbc, 0xbf, 0xc1, 0xc4, 0xc7, 0xc9, 0xcc, 0xce, 0xd1, 0xd3, 0xd5, 0xd8,
    0xda, 0xdc, 0xde, 0xe0, 0xe2, 0xe4, 0xe6, 0xe8, 0xea, 0xec, 0xed, 0xef, 0xf0, 0xf2, 0xf3, 0xf5,
    0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfc, 0xfd, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfd, 0xfc, 0xfc, 0xfb, 0xfa, 0xf9, 0xf8, 0xf7,
    0xf6, 0xf5, 0xf3, 0xf2, 0xf0, 0xef, 0xed, 0xec, 0xea, 0xe8, 0xe6, 0xe4, 0xe2, 0xe0, 0xde, 0xdc,
    0xda, 0xd8, 0xd5, 0xd3, 0xd1, 0xce, 0xcc, 0xc9, 0xc7, 0xc4, 0xc1, 0xbf, 0xbc, 0xb9, 0xb6, 0xb3,
    0xb0, 0xae, 0xab, 0xa8, 0xa5, 0xa2, 0x9f, 0x9c, 0x98, 0x95, 0x92, 0x8f, 0x8c, 0x89, 0x86, 0x83,
    0x80, 0x7c, 0x79, 0x76, 0x73, 0x70, 0x6d, 0x6a, 0x67, 0x63, 0x60, 0x5d, 0x5a, 0x57, 0x54, 0x51,
    0x4f, 0x4c, 0x49, 0x46, 0x43, 0x40, 0x3e, 0x3b, 0x38, 0x36, 0x33, 0x31, 0x2e, 0x2c, 0x2a, 0x27,
    0x25, 0x23, 0x21, 0x1f, 0x1d, 0x1b, 0x19, 0x17, 0x15, 0x13, 0x12, 0x10, 0x0f, 0x0d, 0x0c, 0x0a,
    0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x03, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0a, 0x0c, 0x0d, 0x0f, 0x10, 0x12, 0x13, 0x15, 0x17, 0x19, 0x1b, 0x1d, 0x1f, 0x21, 0x23,
    0x25, 0x27, 0x2a, 0x2c, 0x2e, 0x31, 0x33, 0x36, 0x38, 0x3b, 0x3e, 0x40, 0x43, 0x46, 0x49, 0x4c,
    0x4f, 0x51, 0x54, 0x57, 0x5a, 0x5d, 0x60, 0x63, 0x67, 0x6a, 0x6d, 0x70, 0x73, 0x76, 0x79, 0x7c};

// 1000*256/(8E6/256)

// with RC-oscillator, timer frequency is more like 33.7kHz
// akku_inkr = (uint32_t) (dds_frequency * 256ll * (1ll<<24ll) / 33700ll);
volatile uint32_t akku_inkr = 0x079D5157;   // dds frequency 1kHz

volatile uint32_t akku = 0x00000000;
volatile uint8_t stop_dds = 0;
volatile uint16_t adc_val = 0;

#define GETINDEX(a) ((a & 0xff000000) >> 24)

volatile uint32_t t1 = 0;

// This is called at SAMPLE_RATE kHz to load the next sample.
ISR(TIM0_OVF_vect)
{
    if (stop_dds)
    {
        OCR0A = 0x80;
        return;
    }

    akku += akku_inkr;
    OCR0A = pgm_read_byte(&sinewave_data[GETINDEX(akku)]);
}

volatile uint8_t reg_position;

void requestEvent()
{
    uint8_t data = 0x00;
    switch (reg_position)
    {
    case 0:
    case 1:
    case 2:
    case 3:
    {
        uint8_t *pakku_inkr = (uint8_t *)&akku_inkr;
        data = pakku_inkr[reg_position];
        break;
    }
    case 4:
        adc_val = ADCW;
        data = adc_val & 0x00ff;
        break;
    case 5:
        data = (adc_val & 0xff00) >> 8;
        break;
    default:
        break;
    }

    usiTwiTransmitByte(data);
    // Increment the reg position on each read, and loop back to zero
    reg_position += 1;
}

void receiveEvent(uint8_t left)
{
    // Sanity-check
    if (left < 1 || left > TWI_RX_BUFFER_SIZE)
    {
        return;
    }

    reg_position = usiTwiReceiveByte();
    left--;
    while (left--)
    {
        // write akkuinkr
        if (reg_position < 4)
        {
            uint8_t *pakku_inkr = (uint8_t *)&akku_inkr;
            pakku_inkr[reg_position] = usiTwiReceiveByte();
        }
        else if (reg_position == 4)
        {
            uint8_t tmp = usiTwiReceiveByte();
            stop_dds = !!(tmp & 0x80);
        }
        reg_position++;
    }
}

void TinyWireS_stop_check()
{
    if (!usi_onReceiverPtr)
    {
        // no onReceive callback, nothing to do...
        return;
    }
    if (!(USISR & (1 << USIPF)))
    {
        // Stop not detected
        return;
    }
    uint8_t amount = usiTwiAmountDataInReceiveBuffer();
    if (amount == 0)
    {
        // no data in buffer
        return;
    }
    usi_onReceiverPtr(amount);
}

int main(void)
{
    //	OSCCAL = 0x47; // Wert mit Programiergerät auslesen und hier einsetzen!

    // Own TWI slave address
    uint8_t TWI_slaveAddress = 0x10; // 7 bit address with R/W bit also: (0x10<<1) = 0x20
    usiTwiSlaveInit(TWI_slaveAddress);
    usi_onReceiverPtr = receiveEvent;
    usi_onRequestPtr = requestEvent;

    // init sine dds
    DDRB |= (1 << PB2);
    TCCR0A = 0xc3;
    TCCR0B = 0x01;
    TIMSK0 = 0x01;
    sei();
    OCR0A = pgm_read_byte(&sinewave_data[GETINDEX(akku)]) >> 2;

    // ADC
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 - 125KHz sample rate @ 16MHz

    //ADMUX |= (0 << REFS0); // Set ADC ref to INTERNAL 5v
    //ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading

    DDRA &= ~((1 << PA3) | (1 << PA2));
    PORTA &= ~((1 << PA3) | (1 << PA2));

    ADMUX = 0x10; // VCC Ref, Gain 1x, ADC3 in+, ADC2 in-

    ADCSRA |= (1 << ADATE); // My change.. Tiny free-Running Mode enabled.
    ADCSRA |= (1 << ADEN);  // Enable ADC

    ADCSRA |= (1 << ADSC); // Start A2D Conversions

    //DDRA |= (1 << PA2);
    //PORTA &= ~(1 << PA2);

    // This example is made to work together with the AVR310 USI TWI Master application note. In adition to connecting the TWI
    // pins, also connect PORTB to the LEDS. The code reads a message as a TWI slave and acts according to if it is a
    // general call, or an address call. If it is an address call, then the first byte is considered a command byte and
    // it then responds differently according to the commands.

    // This loop runs forever. If the TWI Transceiver is busy the execution will just continue doing other operations.
    for (;;)
    {
        TinyWireS_stop_check();
    }
}
