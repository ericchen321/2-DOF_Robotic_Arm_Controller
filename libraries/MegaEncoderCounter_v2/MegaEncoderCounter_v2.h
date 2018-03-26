/* Free Software (Beer and Speech) */
/* Please give credit where credit is due. */
/* No stupid legalize, no warranty expressed or implied. */
/* This software is for those who love to create instead of bicker and hamper innovation. */
/* Author: Andrew Jalics */

#ifndef __MEGA_QUADRATURE_ENCODER_COUNTER_V2__
#define __MEGA_QUADRATURE_ENCODER_COUNTER_V2__ 1

#include "Arduino.h"

class MEGAEncoderCounter
{
   public:
	  MEGAEncoderCounter();
      void YAxisReset( );
      unsigned long YAxisGetCount( );
	  unsigned char ReadByte();

   private:
      unsigned long count;
      unsigned char busByte;
	  unsigned char dataBit;
};



#define CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE   32 // Active LOW
#define CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1 31
#define CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2 26
#define CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY 23 // Active LOW
#define CHIP0_D0 30
#define CHIP0_D1 39
#define CHIP0_D2 24
#define CHIP0_D3 25
#define CHIP0_D4 27
#define CHIP0_D5 28
#define CHIP0_D6 29
#define CHIP0_D7 33

#endif
