#include <MegaEncoderCounter_v4.h>
#include <TimerFive.h>                  // used library found on https://playground.arduino.cc/Code/Timer1

MEGAEncoderCounter MegaEncoderCounter; // Initializes the MEGA Encoder Counter

int count = 0;

void setup()
{

  /* Turn on serial */
  Serial.begin(115200);
}

void loop() 
{  
  count = MegaEncoderCounter.YawGetCount();
  Serial.println(count);
}
