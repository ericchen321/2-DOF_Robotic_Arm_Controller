#include "TimerThree.h"

volatile int flag = 1;
unsigned long flagSetArray[1000];
int i=0;

void setup()
{
  Serial.begin(2000000);
  Timer3.initialize(200);
  Timer3.attachInterrupt(setFlag);
}
 
void loop()
{
  if(flag == 1){
    flag = 0;
    flagSetArray[i] = micros();
    i++;
  }

  if (i==1000) {
    for (i=0; i<1000; i++){
      Serial.println(flagSetArray[i]);  
    }
    while(1);
  }
}


void setFlag() {
  flag = 1;  
}
