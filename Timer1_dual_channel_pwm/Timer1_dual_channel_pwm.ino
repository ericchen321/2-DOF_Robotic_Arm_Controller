#include "TimerOne_v2.h"
 
void setup()
{
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  Timer1.initialize(200);         
  Timer1.pwm(11, 512);
  Timer1.pwm(12,900);
  delay(5000);
}
 
void loop()
{
  Timer1.setPwmDuty(11,200);
  delay(5000);
  Timer1.setPwmDuty(12,512);
}
