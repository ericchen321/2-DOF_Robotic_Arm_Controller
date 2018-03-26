/* Free Software (Beer and Speech) */
/* Please give credit where credit is due. */
/* No stupid legalize, no warranty expressed or implied. */
/* This software is for those who love to create instead of bicker and hamper innovation. */
/* Author: Andrew Jalics */

/* Please see header file (MegaEncoderCounter_v2.h) and Avago HCTL-2022 Datasheet for more technical details */

#include "MegaEncoderCounter_v2.h"

MEGAEncoderCounter::MEGAEncoderCounter()
{
   // sets Arduino MEGA (ATMEL ATMEGA) Digital pins as inputs from HCTL-2022 - D0 to D7
   pinMode(CHIP0_D0, INPUT);	// D0
   pinMode(CHIP0_D1, INPUT);	// D1
   pinMode(CHIP0_D2, INPUT);	// D2
   pinMode(CHIP0_D3, INPUT);	// D3
   pinMode(CHIP0_D4, INPUT);	// D4
   pinMode(CHIP0_D5, INPUT);	// D5
   pinMode(CHIP0_D6, INPUT);	// D6
   pinMode(CHIP0_D7, INPUT);	// D7

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

   YAxisReset();
}





// Communicates with a HCTL-2022 IC to get reset the Y encoder count
// see Avago/Agilent/HP HCTL-2022 PDF for details
void MEGAEncoderCounter::YAxisReset( )
{
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, LOW);
   delayMicroseconds(1);
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, HIGH);
   delayMicroseconds(1);
}




// Communicates with a HCTL-2022 IC to get the Y Axis encoder count via an 8bit parallel bus
// see Avago/Agilent/HP HCTL-2022 PDF for details
unsigned long MEGAEncoderCounter::YAxisGetCount( )
{
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE,   LOW);
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, HIGH);
   delayMicroseconds(1);
   busByte = ReadByte();
   count   = busByte;
   count <<= 8;

   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, HIGH);
   delayMicroseconds(1);
   busByte = ReadByte();
   count  = count | busByte;
   count <<= 8;

   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
   delayMicroseconds(1);
   busByte = ReadByte();
   count  = count | busByte;
   count <<= 8;

   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
   delayMicroseconds(1);
   busByte = ReadByte();
   count  = count | busByte;

   digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE,  HIGH);

   return count;
}


unsigned char MEGAEncoderCounter::ReadByte() {
	busByte = B00000000;
	
	dataBit = digitalRead(CHIP0_D0);
	busByte = busByte | (dataBit << 0);

	dataBit = digitalRead(CHIP0_D1);
	busByte = busByte | (dataBit << 1);

	dataBit = digitalRead(CHIP0_D2);
	busByte = busByte | (dataBit << 2);

	dataBit = digitalRead(CHIP0_D3);
	busByte = busByte | (dataBit << 3);

	dataBit = digitalRead(CHIP0_D4);
	busByte = busByte | (dataBit << 4);

	dataBit = digitalRead(CHIP0_D5);
	busByte = busByte | (dataBit << 5);

	dataBit = digitalRead(CHIP0_D6);
	busByte = busByte | (dataBit << 6);

	dataBit = digitalRead(CHIP0_D7);
	busByte = busByte | (dataBit << 7);
	
	return busByte;
}