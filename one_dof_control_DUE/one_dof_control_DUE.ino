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
#define RADIUS 0.1
#define HEIGHT 0.1
#define FRAME_PER_SEC 10

/* Define variables for the dual channel encoder reading algorithm */
volatile signed long encoder0Pos = 0;


/* Define x,y coordinate arrays and yaw, pitch angle arrays */
double desiredXArray[SETPOINT_SIZE];
double desiredYArray[SETPOINT_SIZE];
double desiredYawArray[SETPOINT_SIZE];
double desiredPitchArray[SETPOINT_SIZE];
double actualYawArray[2*SETPOINT_SIZE];
double actualPitchArray[2*SETPOINT_SIZE];
double desiredX;
double desiredY;
double desiredYaw;
double desiredPitch;
unsigned char i = 0; // index for traversing desired Y array

/* For storeOutputs */
unsigned char j = 0; // index for traversing actual pitch array
signed long lastTime = 0;
signed long currentTime = 0;


/* Define control variables for the PID and initialze all PID related stuff */
double actualPitch, pwmOutput;
double Kp=1.5, Ki=0.012, Kd=0.5;
SetPoint mySetPoint(HEIGHT, SETPOINT_SIZE, desiredXArray, desiredYArray, desiredYawArray, desiredPitchArray, &desiredX, &desiredY, &desiredYaw, &desiredPitch);
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
  analogWrite(MOTOR0_ENA, 90);


  /* initialize serial communication */
  Serial.begin(115200);


  /* initialize desired y coordinates array*/
  for (i = 0; i < SETPOINT_SIZE; i++){
    desiredYArray[i] = i * SETPOINT_TIME;  
  }
  for (i = 0; i < SETPOINT_SIZE; i++){
    desiredYArray[i] = RADIUS * sin ( 2 * 3.14 * FRAME_PER_SEC * desiredYArray[i] );
  }

  /* inverse kin - populated desired yaw angles array */
  mySetPoint.InverseKinY();
  

  /* initialize actual angle for the PID algorithm */
  encoder();


  /* turn the PID on */
  myPID.SetMode(AUTOMATIC);
}



void loop() {
  motor();
  setPoint();
  encoder();
  myPID.Compute();
  /*
  storeOutputs();
  serialStuff() ;
  */
}



/* manipulate set points */
void setPoint () {
  mySetPoint.LoadSetPoint();
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


void storeOutputs(){
   currentTime = millis();
   
   if ((currentTime >= lastTime + 4) && j<(2*SETPOINT_SIZE)){
    lastTime = currentTime; 
    actualPitchArray[j] = actualPitch;
    j++;
   }
}


void serialStuff () {
  if (j == 2*SETPOINT_SIZE){
    for (j=0; j < 2*SETPOINT_SIZE; j++){
      Serial.print(desiredPitchArray[j/2]);
      Serial.print(",");
      Serial.print(actualPitchArray[j]);
      Serial.println("");
    }
    while(1);
  }
  
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
