/* included necessary libraries */
#include <PID_v1_modified_v4.h>         // modified based on library found on https://playground.arduino.cc/Code/PIDLibrary
#include <TimerOne_v2.h>                // modified based on library found on https://playground.arduino.cc/Code/Timer1
#include <TimerThree.h>                 // used library found on https://playground.arduino.cc/Code/Timer1
#include <TimerFive.h>                  // used library found on https://playground.arduino.cc/Code/Timer1
#include <MegaEncoderCounter_v5.h>      // modified based on library found on https://www.robogaia.com/two-axis-encoder-counter-mega-shield.html 

/* Define pins */
#define MOTOR1_ENB 12
#define MOTOR1_IN3 8
#define MOTOR1_IN4 9

/* Define other macros */
#define SETPOINT_TIME             40          // Time between two setpoints are loaded in ms
#define SETPOINT_SIZE             25
#define PID_SAMPLE_TIME           500         // PID Sample Time specified in us            
#define PWM_PERIOD                100         // Switching (PWM) period specified in us
#define ANGLE_SAMPLE_ARRAY_SIZE   2000
#define ANGLE_SAMPLE_TIME         4000        // Time between two angles are sampled specified in us
#define SERIAL_BAUD_RATE          115200

/* Declare desired yaw, pitch variables */
float desiredYawArray[SETPOINT_SIZE];
float desiredPitchArray[SETPOINT_SIZE]={0.00,48.37,80.61,92.75,85.50,58.28,12.90,-37.37,-74.44,-91.55,-89.14,-66.99,-25.46,25.46,66.99,89.14,91.55,74.44,37.37,-12.90,-58.28,-85.50,-92.75,-80.61,-48.37};
float desiredYaw;
float desiredPitch;
int i = 0; // index for traversing desired pitch array
int errorClear = 1;  // indicates if the iterm and lastError in PID should be resetted. Should be asserted by loadSetPoint, deasserted by PID.Compute

/* Declare compute flag */
volatile int computeFlag = 1;

/* Define variables for setpoint loading */
signed long newTime = 0;
signed long oldTime = 0;

/* Declare control variables for the PID, initialize PID control object */
float actualPitch = 0;
float pwmOutPitch;
float KiPitch = 0;
float KdPitch = 6;
float KpPitch = 8.518*KdPitch;
PID pitchPID(&actualPitch, &pwmOutPitch, &desiredPitch, KpPitch, KiPitch, KdPitch, &errorClear, PID_SAMPLE_TIME);

/* Declare variables for sampling angles */
char actualPitchSampleArray[ANGLE_SAMPLE_ARRAY_SIZE];
char desiredPitchSampleArray[ANGLE_SAMPLE_ARRAY_SIZE];
volatile int sampleFlag = 1;
int k = 0; // index for traversing angle sample arrays

/* Declare variables for code running time measurement */
unsigned long startTime = 0;
unsigned long stopTime = 0;
unsigned long duration = 0;

void setup() { 
  /* set pin modes */
  pinMode(MOTOR1_ENB, OUTPUT);
  pinMode(MOTOR1_IN3, OUTPUT);
  pinMode(MOTOR1_IN4, OUTPUT);
  
  /* reset the decoders */
  PitchReset();
  YawReset();
  delay(1000);
  
  /* Set initial rotation direction and pwm */
  digitalWrite(MOTOR1_IN3, HIGH);
  digitalWrite(MOTOR1_IN4, LOW);
  Timer1.initialize(PWM_PERIOD);                 // initialize timer1, and set frequency based on period defined
  Timer1.pwm(MOTOR1_ENB, 0);                     // setup initial pwm on MOTOR1_ENB

  /* initialize serial communication */
  Serial.begin(SERIAL_BAUD_RATE);

  /* Set angle reading sampling time*/
  Timer5.initialize(ANGLE_SAMPLE_TIME);
  Timer5.attachInterrupt(setSampleFlag);

  /* Set PID algorithm sampling time */
  Timer3.initialize(PID_SAMPLE_TIME);
  Timer3.attachInterrupt(setComputeFlag);
}



void loop() {
  loadSetPoint();                                             // load in the next desired pitch angle
  
  if (computeFlag == 1) {
    computeFlag = 0;                                            // deassert computeFlag
    actualPitch = PitchGetCount() * 0.9;                        // convert decoder count to acutal pitch angle
    pitchPID.Compute();                                         // compute the next PWM output if computeFlag is set
    pitchMotor();                                               // power up the pitch motor
  }

  if (sampleFlag == 1) {
    sampleFlag = 0;
    angleSampling();                                            // store desired and actual angle in an array
  }

  if (k == ANGLE_SAMPLE_ARRAY_SIZE) {
    serialStuff();
  }
}



inline void loadSetPoint () {
  newTime = millis();
  if (newTime >= oldTime + SETPOINT_TIME) {
    oldTime = newTime;  // update oldTime
    i++;
    if (i >= SETPOINT_SIZE) {
      i = 0;
    }
    desiredYaw = desiredYawArray[i];  // load in new yaw angle
    desiredPitch = desiredPitchArray[i];  // load in new pitch angle
    errorClear = 1;  // tell PID to clear iterm and the last error since a new setpoint is loaded  
  }
}



/* set the direction and pwm of the pitch motor */
inline void pitchMotor () {
  if ( pwmOutPitch < 0 ) {
    digitalWrite(MOTOR1_IN3, LOW);
    digitalWrite(MOTOR1_IN4, HIGH);
    Timer1.setPwmDuty(MOTOR1_ENB, (-1*pwmOutPitch));
  }
  else {
    digitalWrite(MOTOR1_IN3, HIGH);
    digitalWrite(MOTOR1_IN4, LOW);
    Timer1.setPwmDuty(MOTOR1_ENB, pwmOutPitch);
  }
}


/* sample desired vs actual angle*/
inline void angleSampling () {
  desiredPitchSampleArray[k] = (unsigned char)desiredPitch;
  actualPitchSampleArray[k] = (unsigned char)actualPitch;
  k++;
}


/* doing serial print */
inline void serialStuff () {
  for (k= 0; k < ANGLE_SAMPLE_ARRAY_SIZE; k++) {
    Serial.print((int)desiredPitchSampleArray[k]);
    Serial.print(",");
    Serial.print((int)actualPitchSampleArray[k]);
    Serial.println("");
  }
  while (1);
}



/* measureSpeedStart */
inline void measureSpeedStart () {
  startTime = micros();
}



/* measureSpeedStop */
inline void measureSpeedStop () {
  stopTime = micros();
  duration = stopTime - startTime;
  Serial.println(duration);  
}



/* set compute flag which indicates a new PID output should be computed */
void setComputeFlag () {
  computeFlag = 1;  
}



/* set sample flag which indicates new actual and desired should be read */
void setSampleFlag () {
  sampleFlag = 1; 
 }

