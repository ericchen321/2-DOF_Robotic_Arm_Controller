#include <MegaEncoderCounter_v4.h>
#include <TimerFive.h>                  // used library found on https://playground.arduino.cc/Code/Timer1

MEGAEncoderCounter MegaEncoderCounter; // Initializes the MEGA Encoder Counter

signed long count = 0;

void setup()
{

  MegaEncoderCounter.PitchReset();
  
  /* Turn on serial */
  Serial.begin(115200);
}

void loop() 
{  
  count = MegaEncoderCounter.PitchGetCount();
  Serial.println(count);
}
