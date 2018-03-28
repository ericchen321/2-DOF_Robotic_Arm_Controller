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

   private:
      unsigned long count;
      unsigned char busByte;
	  unsigned char dataBit;
};



#define CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1 42
#define CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2 43
#define CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY 47 // Active LOW
#define CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE   48 // Active LOW

#endif
