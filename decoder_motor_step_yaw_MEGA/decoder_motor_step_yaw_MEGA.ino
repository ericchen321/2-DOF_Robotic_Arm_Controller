#include <MegaEncoderCounter_v5.h>
#include <TimerFive.h>                  // used library found on https://playground.arduino.cc/Code/Timer1

#define SPEED_SAMPLE_TIME           2000           // speed sampling time, specified in us
#define SPEED_ARRAY_SIZE            1000           // read speed for this number of times
#define RPM_PER_DEG_PER_SEC         0.167

volatile int sampleFlag = 1;
int decoderOverflowFlag = 0;
int newCount = 0;
int oldCount = 0;
float speedInRPM;

void setup()
{
  /* Reset decoder */
  YawReset();

  /* Turn on serial */
  Serial.begin(115200);
    
  /* Set speed sampling time */
  Timer5.initialize(SPEED_SAMPLE_TIME);
  Timer5.attachInterrupt(setSampleFlag);
}

void loop() 
{  
  /* before the array is populated, keep checking sampleFlage and get new speed reading */
  if (sampleFlag) {
    sampleFlag = 0;
    newCount = YawGetCount();    
    speedInRPM = (newCount - oldCount) * 0.9 / (SPEED_SAMPLE_TIME * 0.000001) * RPM_PER_DEG_PER_SEC;
    Serial.println(speedInRPM);
    oldCount = newCount;
    if (newCount >= 30000) {
      decoderOverflowFlag = 1;  
    }
  }

  if (decoderOverflowFlag){
    while(1);
  }
}

void setSampleFlag () {
  sampleFlag = 1;
}
