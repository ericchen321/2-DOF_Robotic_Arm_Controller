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
#define MOTOR0_ENA 11
#define MOTOR0_IN1 6
#define MOTOR0_IN2 7

/* Define other macros */
#define PID_SAMPLE_TIME           500         // PID Sample Time specified in us            
#define PWM_PERIOD                100         // Switching (PWM) period specified in us
#define ANGLE_SAMPLE_ARRAY_SIZE   2000
#define ANGLE_SAMPLE_TIME         4000        // Time between two angles are sampled specified in us
#define SERIAL_BAUD_RATE          115200

/* Declare desired yaw, pitch variables */
float desiredYaw;
float desiredPitch = 90;
int i = 0; // index for traversing desired pitch array
int errorClear = 1;  // indicates if the iterm and lastError in PID should be resetted. Should be asserted by loadSetPoint, deasserted by PID.Compute

/* Declare compute flag */
volatile int computeFlag = 1;

/* Declare control variables for the PID, initialize PID control object */
float actualPitch = 0;
float pwmOutPitch;
float KiPitch = 0;
float KdPitch = 100;
float KpPitch = 5.127*KdPitch;
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
  pinMode(MOTOR0_ENA, OUTPUT);
  pinMode(MOTOR0_IN1, OUTPUT);
  pinMode(MOTOR0_IN2, OUTPUT);
  
  /* reset the decoders */
  PitchReset();
  YawReset();
  
  /* Set initial rotation direction and pwm */
  digitalWrite(MOTOR0_IN1, HIGH);
  digitalWrite(MOTOR0_IN2, LOW);
  Timer1.initialize(PWM_PERIOD);                 // initialize timer1, and set frequency based on period defined
  Timer1.pwm(MOTOR0_ENA, 512);                     // setup initial pwm on MOTOR1_ENB

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
  if (computeFlag == 1) {
    computeFlag = 0;                                            // deassert computeFlag
    actualPitch = YawGetCount() * 0.9;                        // convert decoder count to acutal pitch angle
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



/* set the direction and pwm of the pitch motor */
inline void pitchMotor () {
  if ( pwmOutPitch < 0 ) {
    digitalWrite(MOTOR0_IN1, LOW);
    digitalWrite(MOTOR0_IN2, HIGH);
    Timer1.setPwmDuty(MOTOR0_ENA, (-1*pwmOutPitch));
  }
  else {
    digitalWrite(MOTOR0_IN1, HIGH);
    digitalWrite(MOTOR0_IN2, LOW);
    Timer1.setPwmDuty(MOTOR0_ENA, pwmOutPitch);
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

