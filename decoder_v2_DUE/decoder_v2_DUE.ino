#include <DueEncoderCounter.h>

#define ARRAY_SIZE 600

DUEEncoderCounter DueEncoderCounter; // Initializes the Due Encoder Counter

signed long millisec[ARRAY_SIZE];
signed long count[ARRAY_SIZE];

signed long count_individual;

int i = 0;

void setup()
{
   DueEncoderCounter.YAxisReset(); 
   Serial.begin(115200);
}

void loop() 
{   
  /*
   millisec[i] = millis();
   count[i] = DueEncoderCounter.YAxisGetCount();
   i++;
   delay(10);
   Serial.println

   
   if (i > ARRAY_SIZE) {
    doSerial(); 
    while(1);
   }
   */
   count_individual = DueEncoderCounter.YAxisGetCount();
   Serial.println(count_individual);
   delay(100);
}

void doSerial(){
   for (i=0; i<=ARRAY_SIZE; i++){
    Serial.print(millisec[i]);
    Serial.print(",");
    Serial.print(count[i]);
    Serial.println("");   
   }
}
