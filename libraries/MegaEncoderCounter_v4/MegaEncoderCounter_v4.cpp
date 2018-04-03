/* Free Software (Beer and Speech) */
/* Please give credit where credit is due. */
/* No stupid legalize, no warranty expressed or implied. */
/* This software is for those who love to create instead of bicker and hamper innovation. */
/* Author: Andrew Jalics */
/* Modified by: Eric Chen*/

/* Please see header file (MegaEncoderCounter_v4.h) and Avago HCTL-2022 Datasheet for more technical details */

#include "MegaEncoderCounter_v4.h"

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
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE, LOW);  // Active LOW
   // Byte Selected MSB SEL1  LOW SEL2 HIGH
   // Byte Selected 2nd SEL1 HIGH SEL2 HIGH
   // Byte Selected 3rd SEL1  LOW SEL2 LOW
   // Byte Selected LSB SEL1 HIGH SEL2 LOW
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, HIGH);  // Active LOW
   delayMicroseconds(10);
   YawReset();

	// reset Chip 1 (Pitch Chip)
   pinMode(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE, OUTPUT);
   pinMode(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, OUTPUT);
   pinMode(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, OUTPUT);
   pinMode(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, OUTPUT);
   digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE, LOW);  
	// Active LOW
	// Byte Selected MSB SEL1  LOW SEL2 HIGH
	// Byte Selected 2nd SEL1 HIGH SEL2 HIGH
	// Byte Selected 3rd SEL1  LOW SEL2 LOW
	// Byte Selected LSB SEL1 HIGH SEL2 LOW
   digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
   digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
   digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, HIGH);  // Active LOW
   delayMicroseconds(10);
   PitchReset();
}





// Communicates with a HCTL-2022 IC to get reset the Yaw encoder count
// see Avago/Agilent/HP HCTL-2022 PDF for details
void MEGAEncoderCounter::YawReset()
{
	digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, LOW);
	delayMicroseconds(100);
	digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, HIGH);
	delayMicroseconds(10);
}




// Communicates with a HCTL-2022 IC to get the Y Axis encoder count via an 8bit parallel bus
// see Avago/Agilent/HP HCTL-2022 PDF for details
unsigned long MEGAEncoderCounter::YawGetCount()
{
	count = 0; // initialize count

	PORTD &= B01111111; // digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
	PORTG &= B11111011; // digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
	delayMicroseconds(10);
	busByte = PINA;
	count += busByte;
	count <<= 8;

	PORTD |= B10000000; // digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
	PORTG &= B11111011; // digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
	delayMicroseconds(10);
	busByte = PINA;
	count += busByte;

	return count;
}




// Communicates with a HCTL-2022 IC to get reset the Pitch encoder count
// see Avago/Agilent/HP HCTL-2022 PDF for details
void MEGAEncoderCounter::PitchReset()
{
	
	digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, LOW);
	delayMicroseconds(100);
	digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, HIGH);
	delayMicroseconds(10);
}



// Communicates with a HCTL-2022 IC to get the Y Axis encoder count via an 8bit parallel bus
// see Avago/Agilent/HP HCTL-2022 PDF for details
unsigned long MEGAEncoderCounter::PitchGetCount()
{
	count = 0; // initialize count

	PORTL &= B01111111; //digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
	PORTL &= B10111111; //digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
	delayMicroseconds(10);
	busByte = PINC;
	count += busByte;
	count <<= 8;

	PORTL |= B10000000; //digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
	PORTL &= B10111111; //digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
	delayMicroseconds(10);
	busByte = PINC;
	count += busByte;

	return count;
}