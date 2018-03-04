/* Free Software (Beer and Speech) */
/* Please give credit where credit is due. */
/* No stupid legalize, no warranty expressed or implied. */
/* This software is for those who love to create instead of bicker and hamper innovation. */
/* Author: Andrew Jalics */

#ifndef __DUE_QUADRATURE_ENCODER_COUNTER__
#define __DUE_QUADRATURE_ENCODER_COUNTER__ 1

#include "Arduino.h"

class DUEEncoderCounter
{
   public:
	  DUEEncoderCounter();
      void YAxisReset( );
      unsigned long YAxisGetCount( );
	  unsigned char ReadByte();

   private:
      unsigned long count;
      unsigned char busByte;
	  unsigned char dataBit;
};



#define DUE_QUADRATURE_ENCODER_COUNTER_PIN_OE   46 // Active LOW
#define DUE_QUADRATURE_ENCODER_COUNTER_PIN_SEL1 48
#define DUE_QUADRATURE_ENCODER_COUNTER_PIN_SEL2 50
#define DUE_QUADRATURE_ENCODER_COUNTER_PIN_RSTY 52 // Active LOW

#endif
