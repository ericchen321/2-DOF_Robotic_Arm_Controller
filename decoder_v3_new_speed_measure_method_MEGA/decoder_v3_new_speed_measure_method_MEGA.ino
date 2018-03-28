#include <MegaEncoderCounter_v3.h>

MEGAEncoderCounter MegaEncoderCounter; // Initializes the MEGA Encoder Counter

char incomingByte;
signed long count = 0;

void setup()
{
   MegaEncoderCounter.YAxisReset();
   Serial.begin(2000000);
   delay(1000);
}

void loop() 
{  
    
   count = MegaEncoderCounter.YAxisGetCount();
   Serial.println(count);
   delay(10);
   
}
