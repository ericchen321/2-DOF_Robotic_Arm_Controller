/* included necessary libraries */
#include <TimerOne_v2.h>                // modified based on library found on https://playground.arduino.cc/Code/Timer1
#include <MegaEncoderCounter_v5.h>

/* Define pins */
 #define MOTOR1_ENB 11
 #define MOTOR1_IN3 6
 #define MOTOR1_IN4 7
 #define MOTOR0_ENA 12
 #define MOTOR0_IN1 8
 #define MOTOR0_IN2 9
 #define PWM_PERIOD     100 

void setup() { 
  /* set pin modes */
  pinMode(MOTOR1_ENB, OUTPUT);
  pinMode(MOTOR1_IN3, OUTPUT);
  pinMode(MOTOR1_IN4, OUTPUT);
  pinMode(MOTOR0_ENA, OUTPUT);
  pinMode(MOTOR0_IN1, OUTPUT);
  pinMode(MOTOR0_IN2, OUTPUT);
    
  /* Set initial rotation direction and pwm */
  digitalWrite(MOTOR0_IN1, HIGH);
  digitalWrite(MOTOR0_IN2, LOW);
  Timer1.initialize(PWM_PERIOD);                 // initialize timer1, and set frequency based on period defined
  Timer1.pwm(MOTOR0_ENA, 700);                     // setup initial pwm

  /* Reset pitch decoder */
  YawReset();

  /* Start serial */
  Serial.begin(115200);
}



void loop() {
  Serial.println(YawGetCount());
  delay(100);
}
