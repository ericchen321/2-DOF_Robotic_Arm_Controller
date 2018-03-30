/* Free Software (Beer and Speech) */
/* Please give credit where credit is due. */
/* No stupid legalize, no warranty expressed or implied. */
/* This software is for those who love to create instead of bicker and hamper innovation. */
/* Author: Andrew Jalics */

/* Please see header file (MegaEncoderCounter_v2.h) and Avago HCTL-2022 Datasheet for more technical details */

#include "MegaEncoderCounter_v3.h"

MEGAEncoderCounter::MEGAEncoderCounter()
{
   // sets Arduino MEGA (ATMEL ATMEGA) Digital pins as inputs from HCTL-2022 - for yaw and pitch chip
	DDRA = B00000000;
	DDRC = B00000000;

	// reset Chip 0 (Yaw Chip)
   pinMode(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE, OUTPUT);
   pinMode(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, OUTPUT);
   pinMode(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, OUTPUT);
   pinMode(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, OUTPUT);
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE, HIGH);  // Active LOW
   // Byte Selected MSB SEL1  LOW SEL2 HIGH
   // Byte Selected 2nd SEL1 HIGH SEL2 HIGH
   // Byte Selected 3rd SEL1  LOW SEL2 LOW
   // Byte Selected LSB SEL1 HIGH SEL2 LOW
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, HIGH);
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, HIGH);  // Active LOW
   YawReset();

	// reset Chip 1 (Pitch Chip)
   pinMode(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE, OUTPUT);
   pinMode(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, OUTPUT);
   pinMode(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, OUTPUT);
   pinMode(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, OUTPUT);
   digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE, HIGH);  
	// Active LOW
	// Byte Selected MSB SEL1  LOW SEL2 HIGH
	// Byte Selected 2nd SEL1 HIGH SEL2 HIGH
	// Byte Selected 3rd SEL1  LOW SEL2 LOW
	// Byte Selected LSB SEL1 HIGH SEL2 LOW
   digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
   digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, HIGH);
   digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, HIGH);  // Active LOW
   PitchReset();
}





// Communicates with a HCTL-2022 IC to get reset the Yaw encoder count
// see Avago/Agilent/HP HCTL-2022 PDF for details
void MEGAEncoderCounter::YawReset( )
{
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, LOW);
   delayMicroseconds(1);
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, HIGH);
   delayMicroseconds(1);
}




// Communicates with a HCTL-2022 IC to get the Y Axis encoder count via an 8bit parallel bus
// see Avago/Agilent/HP HCTL-2022 PDF for details
unsigned long MEGAEncoderCounter::YawGetCount( )
{
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE,   LOW);
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, HIGH);
   delayMicroseconds(1);
   busByte = PINA;
   count   = busByte;
   count <<= 8;

   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, HIGH);
   delayMicroseconds(1);
   busByte = PINA;
   count  = count | busByte;
   count <<= 8;

   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
   delayMicroseconds(1);
   busByte = PINA;
   count  = count | busByte;
   count <<= 8;

   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
   delayMicroseconds(1);
   busByte = PINA;
   count  = count | busByte;

   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE,  HIGH);

   return count;
}



// Communicates with a HCTL-2022 IC to get reset the Pitch encoder count
// see Avago/Agilent/HP HCTL-2022 PDF for details
void MEGAEncoderCounter::PitchReset()
{
	digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, LOW);
	delayMicroseconds(1);
	digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, HIGH);
	delayMicroseconds(1);
}



// Communicates with a HCTL-2022 IC to get the Y Axis encoder count via an 8bit parallel bus
// see Avago/Agilent/HP HCTL-2022 PDF for details
unsigned long MEGAEncoderCounter::PitchGetCount()
{
	digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE, LOW);
	digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
	digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, HIGH);
	delayMicroseconds(1);
	busByte = PINC;
	count = busByte;
	count <<= 8;

	digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
	digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, HIGH);
	delayMicroseconds(1);
	busByte = PINC;
	count = count | busByte;
	count <<= 8;

	digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
	digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
	delayMicroseconds(1);
	busByte = PINC;
	count = count | busByte;
	count <<= 8;

	digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
	digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
	delayMicroseconds(1);
	busByte = PINC;
	count = count | busByte;

	digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE, HIGH);

	return count;
}