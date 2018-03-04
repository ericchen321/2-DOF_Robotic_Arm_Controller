/* included necessary libraries */
#include <PID_v1_modified_v1.h>
#include <SetPoint.h>


/* Define pins */
#define MOTOR0_ENA 9
#define MOTOR0_IN1 6
#define MOTOR0_IN2 7
#define ENCODER0_PINA  22
#define ENCODER0_PINB  24

/* Define other macros */
#define SETPOINT_SIZE 126
#define SETPOINT_TIME 0.008
#define RADIUS 0.06
#define HEIGHT 0.1
#define FRAME_PER_SEC 10

/* Define variables for the dual channel encoder reading algorithm */
volatile signed long encoder0Pos = 0;


/* Define x,y coordinate arrays */
double desiredXArray[SETPOINT_SIZE];
double desiredYArray[SETPOINT_SIZE];
double desiredX;
double desiredY;
double desiredYaw;
double desiredPitch;
unsigned char i = 0; // index for traversing desired Y array


/* Define control variables for the PID and initialze all PID related stuff */
double actualPitch, pwmOutput;
double Kp=1.05, Ki=0.012, Kd=0;
SetPoint mySetPoint(HEIGHT, SETPOINT_SIZE, desiredXArray, desiredYArray, &desiredX, &desiredY, &desiredYaw, &desiredPitch);
PID myPID(&actualPitch, &pwmOutput, &desiredPitch, Kp, Ki, Kd, DIRECT);



void setup() {
  /* set pin modes */
  pinMode(ENCODER0_PINA, INPUT);
  pinMode(ENCODER0_PINB, INPUT);
  pinMode(MOTOR0_ENA, OUTPUT);
  pinMode(MOTOR0_IN1, OUTPUT);
  pinMode(MOTOR0_IN2, OUTPUT);


  /* initialize software interrupts */
  attachInterrupt(digitalPinToInterrupt(ENCODER0_PINA), doEncoder0_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER0_PINB), doEncoder0_B, CHANGE);


  /* Set initial rotation direction and pwm */
  digitalWrite(MOTOR0_IN1, HIGH);
  digitalWrite(MOTOR0_IN2, LOW);
  analogWrite(MOTOR0_ENA, 0);


  /* initialize serial communication */
  Serial.begin(115200);


  /* initialize desired x,y coordinates */
  for (i = 0; i < SETPOINT_SIZE; i++){
    desiredYArray[i] = i * SETPOINT_TIME;  
  }
  for (i = 0; i < SETPOINT_SIZE; i++){
    desiredYArray[i] = RADIUS * sin ( 2 * 3.14 * FRAME_PER_SEC * desiredYArray[i] );
  }
  

  /* initialize actual and desired angle for the PID algorithm */
  encoder();
  desiredPitch = 180;


  /* turn the PID on */
  myPID.SetMode(AUTOMATIC);
}



void loop() {
  setPoint();
  encoder();
  myPID.Compute();
  motor();
  // serialStuff() ;
}



/* manipulate set points */
void setPoint () {
  mySetPoint.LoadSetPoint();
  mySetPoint.LoadKinParams();
  mySetPoint.InverseKinY();
}


/* calculates the relative angle from the encoder's position count */
void encoder () {
  actualPitch = encoder0Pos * 0.9;
}



/* set the rotation and pwm of the motor */
void motor () {
  if ( pwmOutput < 0 ) {
    digitalWrite(MOTOR0_IN1, LOW);
    digitalWrite(MOTOR0_IN2, HIGH);
    analogWrite(MOTOR0_ENA, (-1 * pwmOutput));
  }
  else {
    digitalWrite(MOTOR0_IN1, HIGH);
    digitalWrite(MOTOR0_IN2, LOW);
    analogWrite(MOTOR0_ENA, pwmOutput);
  }

}


void serialStuff () {
  Serial.print(desiredPitch);
  Serial.print(",");
  Serial.print(actualPitch);
  Serial.println("");
}


void doEncoder0_A() {
  // look for a low-to-high on channel A
  if (digitalRead(ENCODER0_PINA) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER0_PINB) == LOW) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER0_PINB) == HIGH) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  // use for debugging - remember to comment out
}

void doEncoder0_B() {
  // look for a low-to-high on channel B
  if (digitalRead(ENCODER0_PINB) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(ENCODER0_PINA) == HIGH) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER0_PINA) == LOW) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}
