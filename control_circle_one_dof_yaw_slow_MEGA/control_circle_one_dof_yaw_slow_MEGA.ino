/* included necessary libraries */
#include <PID_v1_modified_v4.h>         // modified based on library found on https://playground.arduino.cc/Code/PIDLibrary
#include <TimerOne_v2.h>                // modified based on library found on https://playground.arduino.cc/Code/Timer1
#include <TimerThree.h>                 // used library found on https://playground.arduino.cc/Code/Timer1
#include <TimerFive.h>                  // used library found on https://playground.arduino.cc/Code/Timer1
#include <MegaEncoderCounter_v5.h>      // modified based on library found on https://www.robogaia.com/two-axis-encoder-counter-mega-shield.html 

/* Define pins */
#define MOTOR1_ENB 11
#define MOTOR1_IN3 6
#define MOTOR1_IN4 7
#define MOTOR0_ENA 12
#define MOTOR0_IN1 8
#define MOTOR0_IN2 9

/* Define other macros */
#define SETPOINT_TIME             40          // Time between two setpoints are loaded in ms
#define SETPOINT_SIZE             25
#define PID_SAMPLE_TIME           500         // PID Sample Time specified in us            
#define PWM_PERIOD                100         // Switching (PWM) period specified in us
#define ANGLE_SAMPLE_ARRAY_SIZE   2000
#define ANGLE_SAMPLE_TIME         4000        // Time between two angles are sampled specified in us
#define SERIAL_BAUD_RATE          115200

/* Declare desired yaw, pitch variables */
float desiredYawArray[SETPOINT_SIZE]={0.00,7.98,29.42,57.71,84.94,104.85,113.92,110.86,96.09,71.91,43.22,17.36,2.03,2.03,17.36,43.22,71.91,96.09,110.86,113.92,104.85,84.94,57.71,29.42,7.98};
float desiredPitchArray[SETPOINT_SIZE]={0.00,5.50,9.59,11.29,10.26,6.71,1.44,-4.21,-8.76,-11.12,-10.77,-7.80,-2.85,2.85,7.80,10.77,11.12,8.76,4.21,-1.44,-6.71,-10.26,-11.29,-9.59,-5.50};
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
float actualYaw = 0;
float pwmOutYaw;
float KiYaw = 0;
float KdYaw = 5;
float KpYaw = 5.12*KdYaw;
PID yawPID(&actualYaw, &pwmOutYaw, &desiredYaw, KpYaw, KiYaw, KdYaw, &errorClear, PID_SAMPLE_TIME);

/* Declare variables for sampling angles */
char actualYawSampleArray[ANGLE_SAMPLE_ARRAY_SIZE];
char desiredYawSampleArray[ANGLE_SAMPLE_ARRAY_SIZE];
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
  Timer1.pwm(MOTOR0_ENA, 0);                     // setup initial pwm on MOTOR1_ENB

  /* wait for a while */
  delay(100);

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
    actualYaw = YawGetCount() * 0.9;                        // convert decoder count to acutal pitch angle
    yawPID.Compute();                                         // compute the next PWM output if computeFlag is set
    yawMotor();                                               // power up the pitch motor
  }

  if (sampleFlag == 1) {
    sampleFlag = 0;
    angleSampling();                                            // store desired and actual angle in an array
  }

  if (k == ANGLE_SAMPLE_ARRAY_SIZE) {
    digitalWrite(MOTOR0_IN1, LOW);
    digitalWrite(MOTOR0_IN2, LOW);
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
inline void yawMotor () {
  if ( pwmOutYaw < 0 ) {
    digitalWrite(MOTOR0_IN1, LOW);
    digitalWrite(MOTOR0_IN2, HIGH);
    Timer1.setPwmDuty(MOTOR0_ENA, (-1*pwmOutYaw));
  }
  else {
    digitalWrite(MOTOR0_IN1, HIGH);
    digitalWrite(MOTOR0_IN2, LOW);
    Timer1.setPwmDuty(MOTOR0_ENA, pwmOutYaw);
  }
}


/* sample desired vs actual angle*/
inline void angleSampling () {
  desiredYawSampleArray[k] = (unsigned char)desiredYaw;
  actualYawSampleArray[k] = (unsigned char)actualYaw;
  k++;
}


/* doing serial print */
inline void serialStuff () {
  for (k= 0; k < ANGLE_SAMPLE_ARRAY_SIZE; k++) {
    Serial.print((int)desiredYawSampleArray[k]);
    Serial.print(",");
    Serial.print((int)actualYawSampleArray[k]);
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


