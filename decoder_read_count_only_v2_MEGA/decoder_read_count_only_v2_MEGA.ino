#include <MegaEncoderCounter_v5.h>

void setup()
{ 
  /* Reset decoders */
  PitchReset();
  YawReset();
  
  /* Turn on serial */
  Serial.begin(115200);
}



void loop() 
{  
  Serial.println(YawGetCount());
  delay(100);
}
