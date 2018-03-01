/* Free Software (Beer and Speech) */
/* Please give credit where credit is due. */
/* No stupid legalize, no warranty expressed or implied. */
/* This software is for those who love to create instead of bicker and hamper innovation. */
/* Author: Andrew Jalics */

#ifndef __UNO_QUADRATURE_ENCODER_COUNTER__
#define __UNO_QUADRATURE_ENCODER_COUNTER__ 1

#include "Arduino.h"

class UNOEncoderCounter
{
   public:
	  UNOEncoderCounter();
      void YAxisReset( );
      unsigned long YAxisGetCount( );
	  unsigned char ReadByte();

   private:
      unsigned long count;
      unsigned char busByte;
	  unsigned char dataBit;
};



#define UNO_QUADRATURE_ENCODER_COUNTER_PIN_OE   12 // Active LOW
#define UNO_QUADRATURE_ENCODER_COUNTER_PIN_SEL1 10
#define UNO_QUADRATURE_ENCODER_COUNTER_PIN_SEL2 11
#define UNO_QUADRATURE_ENCODER_COUNTER_PIN_RSTY 13 // Active LOW


// The following comments are for reference only...
// (Comments are based on Arduino UNO 1280, should work for all Arduino UNOs in theory...)

// Arduino UNO Digital Pin 30 [ATMEL ATMEGA PORTC 7] -> HCTL 2032 Pin 11 RSTY - Active LOW
// Arduino UNO Digital Pin 31 [ATMEL ATMEGA PORTC 6] -> HCTL 2032 Pin 12 RSTX - Active LOW
// Arduino UNO Digital Pin 32 [ATMEL ATMEGA PORTC 5] -> HCTL 2032 Pin 26 SEL2
// Arduino UNO Digital Pin 33 [ATMEL ATMEGA PORTC 4] -> HCTL 2032 Pin  6 SEL1
// Arduino UNO Digital Pin 34 [ATMEL ATMEGA PORTC 3] -> HCTL 2032 Pin  3 EN2
// Arduino UNO Digital Pin 35 [ATMEL ATMEGA PORTC 2] -> HCTL 2032 Pin  2 EN1
// Arduino UNO Digital Pin 36 [ATMEL ATMEGA PORTC 1] -> HCTL 2032 Pin  7 OE   - Active LOW
// Arduino UNO Digital Pin 37 [ATMEL ATMEGA PORTC 0] -> HCTL 2032 Pin 32 X/Y 

// Arduino UNO Digital Pin 22 [ATMEL ATMEGA PORTA 0] <- HCTL 2032 Pin  1 D0
// Arduino UNO Digital Pin 23 [ATMEL ATMEGA PORTA 1] <- HCTL 2032 Pin 15 D1
// Arduino UNO Digital Pin 24 [ATMEL ATMEGA PORTA 2] <- HCTL 2032 Pin 14 D2
// Arduino UNO Digital Pin 25 [ATMEL ATMEGA PORTA 3] <- HCTL 2032 Pin 13 D3
// Arduino UNO Digital Pin 26 [ATMEL ATMEGA PORTA 4] <- HCTL 2032 Pin 12 D4
// Arduino UNO Digital Pin 27 [ATMEL ATMEGA PORTA 5] <- HCTL 2032 Pin 11 D5
// Arduino UNO Digital Pin 28 [ATMEL ATMEGA PORTA 6] <- HCTL 2032 Pin 10 D6
// Arduino UNO Digital Pin 29 [ATMEL ATMEGA PORTA 7] <- HCTL 2032 Pin  9 D7

// HCTL-2022 Byte Selected Info
// Byte Selected MSB SEL1  LOW SEL2 HIGH
// Byte Selected 2nd SEL1 HIGH SEL2 HIGH
// Byte Selected 3rd SEL1  LOW SEL2 LOW
// Byte Selected LSB SEL1 HIGH SEL2 LOW

// Quadrature Encoder connections US Digital
// orange solid  5V
// blue solid    A
// orange stripe GND
// blue stripe   B

// Quadrature Encoder connections  HEDM-5500
// Pin 1 GND
// Pin 2 NC
// Pin 3 A
// Pin 4 5V
// Pin 5 B

// Maxon Motor HEDM-5500 Quadrature Encoder connections
// white  5V
// brown  GND
// green  A
// yellow B

#endif
