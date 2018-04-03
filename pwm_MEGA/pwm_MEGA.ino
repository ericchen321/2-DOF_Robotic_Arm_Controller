/* included necessary libraries */
#include <TimerOne_v2.h>                // modified based on library found on https://playground.arduino.cc/Code/Timer1

/* Define pins */
 #define MOTOR1_ENB 12
 #define MOTOR1_IN3 8
 #define MOTOR1_IN4 9
 #define PWM_PERIOD     100 

void setup() { 
  /* set pin modes */
  pinMode(MOTOR1_ENB, OUTPUT);
  pinMode(MOTOR1_IN3, OUTPUT);
  pinMode(MOTOR1_IN4, OUTPUT);
    
  /* Set initial rotation direction and pwm */
  digitalWrite(MOTOR1_IN3, HIGH);
  digitalWrite(MOTOR1_IN4, LOW);
  Timer1.initialize(PWM_PERIOD);                 // initialize timer1, and set frequency based on period defined
  Timer1.pwm(MOTOR1_ENB, 512);                     // setup initial pwm on MOTOR1_ENB
}



void loop() {
}
