#include <MegaEncoderCounter_v3.h>
#include <FlexiTimer2.h>                // used library found on https://playground.arduino.cc/Main/FlexiTimer2

#define SPEED_SAMPLE_TIME 5            // speed sampling time, specified in ms
#define RPM_PER_DEG_PER_SEC 0.167

MEGAEncoderCounter MegaEncoderCounter; // Initializes the MEGA Encoder Counter

volatile int sampleFlag = 1;
signed long newCount = 0;
signed long oldCount = 0;
float speedInRPM = 0;

void setup()
{
  /* Reset pitch decoder */
  MegaEncoderCounter.PitchReset();
  delay(1000);

  /* Turn on serial */
  Serial.begin(2000000);
    
   /* Set speed sampling time */
  FlexiTimer2::set(SPEED_SAMPLE_TIME, setSampleFlag);
  FlexiTimer2::start();
}

void loop() 
{  
  if (sampleFlag == 1) {
    sampleFlag = 0;
    newCount = MegaEncoderCounter.PitchGetCount();
    speedInRPM = (newCount - oldCount) * 0.9 / (SPEED_SAMPLE_TIME * 0.001) * RPM_PER_DEG_PER_SEC;
    oldCount = newCount;
    Serial.println(speedInRPM);
  } 
}

void setSampleFlag () {
  sampleFlag = 1;
}
