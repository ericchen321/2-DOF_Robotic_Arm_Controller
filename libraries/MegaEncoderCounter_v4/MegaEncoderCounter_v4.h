/* Free Software (Beer and Speech) */
/* Please give credit where credit is due. */
/* No stupid legalize, no warranty expressed or implied. */
/* This software is for those who love to create instead of bicker and hamper innovation. */
/* Author: Andrew Jalics */
/* Modified by: Eric Chen*/

#ifndef __MEGA_QUADRATURE_ENCODER_COUNTER_V2__
#define __MEGA_QUADRATURE_ENCODER_COUNTER_V2__ 1

#include "Arduino.h"

class MEGAEncoderCounter
{
   public:
	  MEGAEncoderCounter();
      void YawReset();
      unsigned long YawGetCount();
	  void PitchReset();
	  unsigned long PitchGetCount();

   private:
      int count;
      unsigned char busByte;
};



#define CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1 38
#define CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2 39
#define CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY 40 // Active LOW
#define CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE   41 // Active LOW

#define CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1 42 // Port L.7
#define CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2 43 // Port L.6
#define CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY 47 // Active LOW, Port L.2
#define CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE   48 // Active LOW, Port L.1

#endif
