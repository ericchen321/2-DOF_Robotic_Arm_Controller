#include <TimerOne.h>

// encoder
#define encoder0PinA  2
#define encoder0PinB  3
#define ENCODER0_RES 3.6
#define RAD_PER_DEG 0.01745

// motor
#define enA 9
#define in1 6
#define in2 7

// serial
#define SERIAL_PERIOD 5

volatile signed long encoder0Pos = 0;
float encoder0Vel = 0;
int pwmOutput = 400; // pwmOutput = 1023 correspond to 100% duty cycle

unsigned long lastTime = 0;
unsigned long currentTime;

void setup() {
  // motor
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // Set initial rotation direction
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  Timer1.initialize(200);         // initialize timer1, and set period in us
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
  currentTime = millis();
    
  if (currentTime >= lastTime + SERIAL_PERIOD){
    encoder0Vel = encoder0Pos * ENCODER0_RES * RAD_PER_DEG / (SERIAL_PERIOD * 0.001);
    Serial.print (currentTime);
    Serial.print (",");
    Serial.print (encoder0Vel);
    Serial.println("");  
    encoder0Pos = 0;
    lastTime = currentTime;
  }
}


void callback()
{
  digitalWrite(10, digitalRead(10) ^ 1);
}


void doEncoderA() {
  // look for a low-to-high on channel A
  encoder0Pos ++;
}
