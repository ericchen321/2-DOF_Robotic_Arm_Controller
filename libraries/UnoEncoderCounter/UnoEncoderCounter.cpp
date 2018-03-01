/* Free Software (Beer and Speech) */
/* Please give credit where credit is due. */
/* No stupid legalize, no warranty expressed or implied. */
/* This software is for those who love to create instead of bicker and hamper innovation. */
/* Author: Andrew Jalics */

/* Please see header file (UNOEncoderCounter.h) and Avago HCTL-2022 Datasheet for more technical details */

#include "UnoEncoderCounter.h"

UNOEncoderCounter::UNOEncoderCounter()
{
   pinMode(0, INPUT);
   pinMode(1, INPUT);   // sets Arduino UNO (ATMEL ATMEGA) Digital pins 2(PORTD1) to 9(PORTD8) as inputs from HCTL-2022 - D0 to D7
   pinMode(2, INPUT);
   pinMode(3, INPUT);
   pinMode(4, INPUT);
   pinMode(5, INPUT);
   pinMode(6, INPUT);
   pinMode(7, INPUT);
   pinMode(8, INPUT);
   pinMode(9, INPUT);

   pinMode(UNO_QUADRATURE_ENCODER_COUNTER_PIN_OE,   OUTPUT);
   pinMode(UNO_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, OUTPUT);
   pinMode(UNO_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, OUTPUT);
   pinMode(UNO_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, OUTPUT);

   digitalWrite(UNO_QUADRATURE_ENCODER_COUNTER_PIN_OE, HIGH);  // Active LOW

   // Byte Selected MSB SEL1  LOW SEL2 HIGH
   // Byte Selected 2nd SEL1 HIGH SEL2 HIGH
   // Byte Selected 3rd SEL1  LOW SEL2 LOW
   // Byte Selected LSB SEL1 HIGH SEL2 LOW
   digitalWrite(UNO_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
   digitalWrite(UNO_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, HIGH);

   digitalWrite(UNO_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, HIGH);  // Active LOW

   YAxisReset();
}





// Communicates with a HCTL-2022 IC to get reset the Y encoder count
// see Avago/Agilent/HP HCTL-2022 PDF for details
void UNOEncoderCounter::YAxisReset( )
{
   digitalWrite(UNO_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, LOW);
   delayMicroseconds(1);
   digitalWrite(UNO_QUADRATURE_ENCODER_COUNTER_PIN_RSTY, HIGH);
   delayMicroseconds(1);
}




// Communicates with a HCTL-2022 IC to get the Y Axis encoder count via an 8bit parallel bus
// see Avago/Agilent/HP HCTL-2022 PDF for details
unsigned long UNOEncoderCounter::YAxisGetCount( )
{
   digitalWrite(UNO_QUADRATURE_ENCODER_COUNTER_PIN_OE,   LOW);
   digitalWrite(UNO_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
   digitalWrite(UNO_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, HIGH);
   delayMicroseconds(1);
   busByte = ReadByte();
   count   = busByte;
   count <<= 8;

   digitalWrite(UNO_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
   digitalWrite(UNO_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, HIGH);
   delayMicroseconds(1);
   busByte = ReadByte();
   count  += busByte;
   count <<= 8;

   digitalWrite(UNO_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
   digitalWrite(UNO_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
   delayMicroseconds(1);
   busByte = ReadByte();
   count  += busByte;
   count <<= 8;

   digitalWrite(UNO_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
   digitalWrite(UNO_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
   delayMicroseconds(1);
   busByte = ReadByte();
   count  += busByte;

   digitalWrite(UNO_QUADRATURE_ENCODER_COUNTER_PIN_OE,  HIGH);

   return count;
}


unsigned char UNOEncoderCounter::ReadByte() {
	busByte = B00000000;

	int i = 2;
	for (i = 2; i <= 9; i++) {
		dataBit = digitalRead(i);
		busByte = busByte | (dataBit << (i - 2));
	}

	return busByte;
}