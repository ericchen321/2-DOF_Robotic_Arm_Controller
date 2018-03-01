#include <UnoEncoderCounter.h>

UNOEncoderCounter UnoEncoderCounter; // Initializes the Uno Encoder Counter

char incomingByte;
signed long count = 0;

void setup()
{
   UnoEncoderCounter.YAxisReset();
   Serial.begin(2000000);
}

void loop() 
{  
   count = UnoEncoderCounter.YAxisGetCount();
   Serial.print(millis());
   Serial.print(",");
   Serial.print(count);
   Serial.println("");
   delay(10);
}
