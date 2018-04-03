#include <MegaEncoderCounter_v3.h>
#include <TimerFive.h>                  // used library found on https://playground.arduino.cc/Code/Timer1

#define SPEED_SAMPLE_TIME           5000           // speed speed sampling time, specified in us
#define SPEED_ARRAY_SIZE            1000           // read speed for this number of times
#define RPM_PER_DEG_PER_SEC         0.167

MEGAEncoderCounter MegaEncoderCounter; // Initializes the MEGA Encoder Counter

volatile int sampleFlag = 1;
signed long newCount = 0;
signed long oldCount = 0;
float speedInRPM;

void setup()
{
  /* Reset pitch decoder */
  MegaEncoderCounter.PitchReset();
  delay(1000);

  /* Turn on serial */
  Serial.begin(2000000);
    
  /* Set speed sampling time */
  Timer5.initialize(SPEED_SAMPLE_TIME);
  Timer5.attachInterrupt(setSampleFlag);
}

void loop() 
{  
  /* before the array is populated, keep checking sampleFlage and get new speed reading */
  if (sampleFlag == 1) {
    sampleFlag = 0;
    newCount = MegaEncoderCounter.PitchGetCount();    
    speedInRPM = (newCount - oldCount) * 0.9 / (SPEED_SAMPLE_TIME * 0.000001) * RPM_PER_DEG_PER_SEC;
    Serial.println(speedInRPM);
    oldCount = newCount;
  }
}

void setSampleFlag () {
  sampleFlag = 1;
}
