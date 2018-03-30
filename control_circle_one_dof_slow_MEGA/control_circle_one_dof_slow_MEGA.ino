/* included necessary libraries */
#include <PID_v1_modified_v4.h>         // modified based on library found on https://playground.arduino.cc/Code/PIDLibrary
#include <TimerOne_v2.h>                // modified based on library found on https://playground.arduino.cc/Code/Timer1
#include <FlexiTimer2.h>                // used library found on https://playground.arduino.cc/Main/FlexiTimer2
#include <MegaEncoderCounter_v3.h>      // modified based on library found on https://www.robogaia.com/two-axis-encoder-counter-mega-shield.html 

/* Define pins */
#define MOTOR1_ENB 12
#define MOTOR1_IN3 8
#define MOTOR1_IN4 9

/* Define other macros */
#define SETPOINT_TIME             40          // Time between two setpoints are loaded in ms
#define SETPOINT_SIZE             126
#define PID_SAMPLE_TIME           2           // PID Sample Time specified in ms            
#define PWM_PERIOD                200         // Switching (PWM) period specified in us
#define SERIAL_BAUD_RATE          2000000

/* Declare desired yaw, pitch variables */
float desiredYawArray[SETPOINT_SIZE];
float desiredPitchArray[SETPOINT_SIZE]={0.00, 16.12, 26.87, 30.92, 28.50, 19.43, 4.30, -12.46, -24.81, -30.52, -29.71, -22.33, -8.49, 8.49, 22.33, 29.71, 30.52, 24.81, 12.46, -4.30, -19.43, -28.50, -30.92, -26.87, -16.12, -0.00, 16.12, 26.87, 30.92, 28.50, 19.43, 4.30, -12.46, -24.81, -30.52, -29.71, -22.33, -8.49, 8.49, 22.33, 29.71, 30.52, 24.81, 12.46, -4.30, -19.43, -28.50, -30.92, -26.87, -16.12, -0.00, 16.12, 26.87, 30.92, 28.50, 19.43, 4.30, -12.46, -24.81, -30.52, -29.71, -22.33, -8.49, 8.49, 22.33, 29.71, 30.52, 24.81, 12.46, -4.30, -19.43, -28.50, -30.92, -26.87, -16.12, -0.00, 16.12, 26.87, 30.92, 28.50, 19.43, 4.30, -12.46, -24.81, -30.52, -29.71, -22.33, -8.49, 8.49, 22.33, 29.71, 30.52, 24.81, 12.46, -4.30, -19.43, -28.50, -30.92, -26.87, -16.12, -0.00, 16.12, 26.87, 30.92, 28.50, 19.43, 4.30, -12.46, -24.81, -30.52, -29.71, -22.33, -8.49, 8.49, 22.33, 29.71, 30.52, 24.81, 12.46, -4.30, -19.43, -28.50, -30.92, -26.87, -16.12, -0.00};
float desiredYaw;
float desiredPitch;
unsigned char i = 0; // index for traversing desired pitch array
int errorClear = 1;  // indicates if the iterm and lastError in PID should be resetted. Should be asserted by loadSetPoint, deasserted by PID.Compute

/* Declare compute flag */
volatile int computeFlag = 1;

/* Define variables for setpoint loading */
signed long newTime = 0;
signed long oldTime = 0;

/* Initialize decoder reader object */
MEGAEncoderCounter MegaEncoderCounter;

/* Declare control variables for the PID, initialize PID control object */
float actualPitch = 0;
float pwmOutPitch;
float KiPitch = 0;
float KdPitch = 6;
float KpPitch = 8.518*KdPitch;
PID pitchPID(&actualPitch, &pwmOutPitch, &desiredPitch, KpPitch, KiPitch, KdPitch, &errorClear, PID_SAMPLE_TIME);

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
  MegaEncoderCounter.PitchReset();
  delay(1000);
  
  /* Set initial rotation direction and pwm */
  digitalWrite(MOTOR1_IN3, HIGH);
  digitalWrite(MOTOR1_IN4, LOW);
  Timer1.initialize(PWM_PERIOD);                 // initialize timer1, and set frequency based on period defined
  Timer1.pwm(MOTOR1_ENB, 0);                     // setup initial pwm on MOTOR1_ENB

  /* initialize serial communication */
  Serial.begin(SERIAL_BAUD_RATE);

  /* Set PID algorithm sampling time */
  FlexiTimer2::set(PID_SAMPLE_TIME, setComputeFlag);
  FlexiTimer2::start();
}



void loop() {
  loadSetPoint();                                             // load in the next desired pitch angle
  
  if (computeFlag == 1) {
    actualPitch = MegaEncoderCounter.PitchGetCount() * 0.9;     // convert decoder count to acutal pitch angle
    pitchPID.Compute();                                         // compute the next PWM output if computeFlag is set
    pitchMotor();                                               // power up the pitch motor
    serialStuff();                                              // send operating data to laptop
    computeFlag = 0;                                            // deassert computeFlag
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


/* doing serial print */
inline void serialStuff () {
  Serial.print(desiredPitch);
  Serial.print(",");
  Serial.print(actualPitch);
  Serial.println("");
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

