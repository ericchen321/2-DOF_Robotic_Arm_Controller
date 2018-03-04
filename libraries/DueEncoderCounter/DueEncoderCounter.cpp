/* Free Software (Beer and Speech) */
/* Please give credit where credit is due. */
/* No stupid legalize, no warranty expressed or implied. */
/* This software is for those who love to create instead of bicker and hamper innovation. */
/* Author: Andrew Jalics */

/* Please see header file (DUEEncoderCounter.h) and Avago HCTL-2022 Datasheet for more technical details */

#include "DueEncoderCounter.h"

DUEEncoderCounter::DUEEncoderCounter()
{
   pinMode(30, INPUT); 
   pinMode(32, INPUT);
   pinMode(34, INPUT);
   pinMode(36, INPUT);
   pinMode(38, INPUT);
   pinMode(40, INPUT);
   pinMode(42, INPUT);
   pinMode(44, INPUT);


   pinMode(DUE_QUADRATURE_ENCODER_COUNTER_PIN_OE,   OUTPUT);
   pinMode(DUE_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, OUTPUT);
   pinMode(DUE_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, OUTPUT);
   pinMode(DUE_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, OUTPUT);

   digitalWrite(DUE_QUADRATURE_ENCODER_COUNTER_PIN_OE, HIGH);  // Active LOW

   // Byte Selected MSB SEL1  LOW SEL2 HIGH
   // Byte Selected 2nd SEL1 HIGH SEL2 HIGH
   // Byte Selected 3rd SEL1  LOW SEL2 LOW
   // Byte Selected LSB SEL1 HIGH SEL2 LOW
   digitalWrite(DUE_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
   digitalWrite(DUE_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, HIGH);

   digitalWrite(DUE_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, HIGH);  // Active LOW

   YAxisReset();
}





// Communicates with a HCTL-2022 IC to get reset the Y encoder count
// see Avago/Agilent/HP HCTL-2022 PDF for details
void DUEEncoderCounter::YAxisReset( )
{
   digitalWrite(DUE_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, LOW);
   delayMicroseconds(1);
   digitalWrite(DUE_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, HIGH);
   delayMicroseconds(1);
}




// Communicates with a HCTL-2022 IC to get the Y Axis encoder count via an 8bit parallel bus
// see Avago/Agilent/HP HCTL-2022 PDF for details
unsigned long DUEEncoderCounter::YAxisGetCount( )
{
   digitalWrite(DUE_QUADRATURE_ENCODER_COUNTER_PIN_OE,   LOW);
   digitalWrite(DUE_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
   digitalWrite(DUE_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, HIGH);
   delayMicroseconds(1);
   busByte = ReadByte();
   count   = busByte;
   count <<= 8;

   digitalWrite(DUE_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
   digitalWrite(DUE_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, HIGH);
   delayMicroseconds(1);
   busByte = ReadByte();
   count  += busByte;
   count <<= 8;

   digitalWrite(DUE_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
   digitalWrite(DUE_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
   delayMicroseconds(1);
   busByte = ReadByte();
   count  += busByte;
   count <<= 8;

   digitalWrite(DUE_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
   digitalWrite(DUE_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
   delayMicroseconds(1);
   busByte = ReadByte();
   count  += busByte;

   digitalWrite(DUE_QUADRATURE_ENCODER_COUNTER_PIN_OE,  HIGH);

   return count;
}


unsigned char DUEEncoderCounter::ReadByte() {
	busByte = B00000000;

	int i = 30;
	for (i = 30; i <= 44; i=i+2) {
		dataBit = digitalRead(i);
		busByte = busByte | (dataBit << ((i-30)/2));
	}

	return busByte;
}