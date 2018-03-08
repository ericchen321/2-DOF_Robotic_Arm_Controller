 /*  Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01
    by Dejan Nedelkovski, www.HowToMechatronics.com
*/

#include "TimerOne.h"
#define enA 9
#define in1 12
#define in2 11
#define FREQ 200
// #include <SimpleTimer.h>
// SimpleTimer timer;
int rotDirection = 0;
int pressed = false;
#define encoder0PinA  2
#define encoder0PinB  3
// volatile long signed encoder0Pos = 0;
volatile unsigned rpm=0;
volatile unsigned period_milli;
volatile unsigned long accum_time;
bool change = false;

volatile int encoder0Pos = 0;
volatile int speed = 0;



void setup() {
    Serial.begin(115200);
    // Serial.println("Setting up pins");
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    pinMode(encoder0PinA, INPUT);
    pinMode(encoder0PinB, INPUT);

    // encoder pin on interrupt 0 (pin 2)
    attachInterrupt(0, doEncoderA, CHANGE);
    attachInterrupt(1, doEncoderB, CHANGE);
    
    Timer1.initialize(3000);         // initialize timer1, and set a 1/2 second period
    Timer1.attachInterrupt(timer_ISR);  // attaches callback() as a timer overflow interrupt
}
void loop() {

      if (change)
      {
        Serial.println(speed);
        change = false;
      }

}

void timer_ISR()
{
  speed = encoder0Pos*0.8333*60;
  encoder0Pos = 0;
  change = true;
}




void doEncoderA() {
  encoder0Pos++;
}

void doEncoderB()
{
  encoder0Pos++;
}
