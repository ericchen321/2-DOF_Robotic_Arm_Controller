#include "PID_v1_modified_v1.h"

/* included necessary libraries */


/* Define pins */
#define MOTOR0_ENA 9
#define MOTOR0_IN1 6
#define MOTOR0_IN2 7
#define ENCODER0_PINA  2
#define ENCODER0_PINB  3


/* Define variables for the dual channel encoder reading algorithm */
volatile signed long encoder0Pos = 0;


/* Define Variables for the PID algorithm and initialize the algorithm */
double desiredAngle, actualAngle, pwmOutput;
double Kp=2, Ki=5, Kd=1;
PID myPID(&actualAngle, &pwmOutput, &desiredAngle, Kp, Ki, Kd, DIRECT);



void setup() {
  /* set pin modes */
  pinMode(ENCODER0_PINA, INPUT);
  pinMode(ENCODER0_PINB, INPUT);
  pinMode(MOTOR0_ENA, OUTPUT);
  pinMode(MOTOR0_IN1, OUTPUT);
  pinMode(MOTOR0_IN2, OUTPUT);


  /* initialize software interrupts */
  attachInterrupt(0, doEncoder0_A, CHANGE);
  attachInterrupt(1, doEncoder0_B, CHANGE);


  /* Set initial rotation direction */
  digitalWrite(MOTOR0_IN1, HIGH);
  digitalWrite(MOTOR0_IN2, LOW);


  /* initialize the actual angle and desired angle for the PID algorithm */
  encoder();
  desiredAngle = 100;


  /* turn the PID on */
  myPID.SetMode(AUTOMATIC);
}



void loop() {
  encoder();
  myPID.Compute();
  motor();
}



/* calculates the relative angle from the encoder's position count */
void encoder () {
  actualAngle = encoder0Pos * 0.9;
}



/* set the rotation and pwm of the motor */
void motor () {
  if ( myPID.GetDirection() == DIRECT ) {
    digitalWrite(MOTOR0_IN1, HIGH);
    digitalWrite(MOTOR0_IN2, LOW);
  }
  else {
    digitalWrite(MOTOR0_IN1, LOW);
    digitalWrite(MOTOR0_IN2, HIGH);
  }

  analogWrite(MOTOR0_ENA, pwmOutput);
}


/* interrupt channel A */
void doEncoder0_A() {
  // look for a low-to-high on channel A
  if (PORTD & B00000100 == B00000100) { // check if A is high

    // check channel B to see which way encoder is turning
    if (PORTD & B00001000 == B00000000) { // check if B is low
      encoder0Pos = encoder0Pos - 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos + 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (PORTD & B00001000 == B00001000) {  // check if B is high
      encoder0Pos = encoder0Pos - 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos + 1;          // CCW
    }
  }
  // use for debugging - remember to comment out
}


/* interrupt channel B */
void doEncoder0_B() {
  // look for a low-to-high on channel B
  if (PORTD & B00001000 == B00001000) { // if B is high

    // check channel A to see which way encoder is turning
    if (PORTD & B00000100 == B00000100) { // if A is high
      encoder0Pos = encoder0Pos - 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos + 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (PORTD & B00000100 == B00000000) { // if A is low
      encoder0Pos = encoder0Pos - 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos + 1;          // CCW
    }
  }
}
