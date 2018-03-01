#include <TimerOne.h>

// encoder
#define encoder0PinA  2
#define encoder0PinB  3

// motor
#define enA 9
#define in1 6
#define in2 7

volatile signed long encoder0Pos = 0;
volatile unsigned char flag = 0;
int pwmOutput = 512; // pwmOutput = 1023 correspond to 100% duty cycle

void setup() {
  // motor
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // Set initial rotation direction
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  Timer1.initialize(5000);         // initialize timer1, and set period in us
  Timer1.pwm(enA, pwmOutput);                // setup pwm on pin 9, and specify duty cycle
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  
  // encoder
  pinMode(encoder0PinA, INPUT);
   
  // first interrupt
  attachInterrupt(0, doEncoderA, RISING);
  
  // Serial
  Serial.begin (2000000);
}

void loop() {
  if (flag == 1){
    flag = 0;
    Serial.print (micros());
    Serial.print (",");
    Serial.print (encoder0Pos);
    Serial.println("");  
  }
}


void callback()
{
  digitalWrite(10, digitalRead(10) ^ 1);
}


void doEncoderA() {
  // look for a low-to-high on channel A
  flag = 1;
  encoder0Pos ++;
}
