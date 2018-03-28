/* included necessary libraries */
#include <PID_v1_modified_v4.h>         // modified based on library found on https://playground.arduino.cc/Code/PIDLibrary
#include <SetPoint_v3.h>
#include <TimerOne_v2.h>                // modified based on library found on https://playground.arduino.cc/Code/Timer1
#include <FlexiTimer2.h>                // used library found on https://playground.arduino.cc/Main/FlexiTimer2
#include <MegaEncoderCounter_v3.h>      // modified based on library found on https://www.robogaia.com/two-axis-encoder-counter-mega-shield.html 


/* Define pins */
#define MOTOR0_ENA 11
#define MOTOR0_IN1 6
#define MOTOR0_IN2 7

/* Define other macros */
#define PID_SAMPLE_TIME 1           // PID Sample Time specified in ms
#define SETPOINT_SIZE 126
#define SETPOINT_TIME 0.08          // time between two setpoint release times in s
#define RADIUS 0.06                 // radius specified in m
#define HEIGHT 0.1                  // height (laser tip to screen distance) specified in m
#define FRAME_PER_SEC 1             
#define PWM_PERIOD 200              //PWM period specified in us
#define SERIAL_PERIOD 40            // specified in ms
#define SERIAL_BAUD_RATE 2000000

/* Declare desired x,y, yaw, pitch variables */
float desiredXArray[SETPOINT_SIZE];
float desiredYArray[SETPOINT_SIZE];
float desiredYawArray[SETPOINT_SIZE];
float desiredPitchArray[SETPOINT_SIZE];
float desiredYaw;
float desiredPitch;
unsigned char i = 0; // index for traversing desired Y array

/* Declare compute flag */
volatile int computeFlag = 0;

/* Initialize decoder reader object */
MEGAEncoderCounter MegaEncoderCounter;

/* Declare control variables for the PID, initialize setpoint control object, initialize PID control object */
float actualPitch;
float pwmOutput;
float Ki=0, Kd=6;
float Kp=8.518*Kd;
SetPoint mySetPoint(HEIGHT, SETPOINT_SIZE, desiredXArray, desiredYArray, desiredYawArray, desiredPitchArray, &desiredYaw, &desiredPitch);
PID myPID(&actualPitch, &pwmOutput, &desiredPitch, &computeFlag, Kp, Ki, Kd, PID_SAMPLE_TIME);

/* Declare variables for code running time measurement */
unsigned long startTime = 0;
unsigned long stopTime = 0;
unsigned long duration = 0;

/* Define variables for serial */
signed long currentTime = 0;
signed long lastTime = 0;



void setup() {
  /* wait for a while */
  delay(5000);
  
  /* set pin modes */
  pinMode(MOTOR0_ENA, OUTPUT);
  pinMode(MOTOR0_IN1, OUTPUT);
  pinMode(MOTOR0_IN2, OUTPUT);

  /* Set initial rotation direction and pwm */
  digitalWrite(MOTOR0_IN1, HIGH);
  digitalWrite(MOTOR0_IN2, LOW);
  Timer1.initialize(PWM_PERIOD);                 // initialize timer1, and set frequency based on period defined
  Timer1.pwm(MOTOR0_ENA, 0);                     // setup initial pwm on MOTOR0_ENA

  /* initialize serial communication */
  Serial.begin(SERIAL_BAUD_RATE);

  /* initialize desired x,y coordinates, and do inverse kinematics to get desired yaw, ptich values */
  for (i = 0; i < SETPOINT_SIZE; i++){
    desiredYArray[i] = i * SETPOINT_TIME;  
  }
  for (i = 0; i < SETPOINT_SIZE; i++){
    desiredYArray[i] = RADIUS * sin ( 2 * 3.14 * FRAME_PER_SEC * desiredYArray[i] );
  }
  mySetPoint.InverseKinY();

  /* reset the decoder */
  MegaEncoderCounter.YAxisReset();
  
  /* initialize actual and desired angle for the PID algorithm */
  actualPitch = 0;
  desiredPitch = 0;

  /* Set PID algorithm sampling time */
  FlexiTimer2::set(PID_SAMPLE_TIME, setComputeFlag);
  FlexiTimer2::start();
}



void loop() {
  
  mySetPoint.LoadSetPoint();                                  // load in the next desired pitch angle
  
  
  actualPitch = MegaEncoderCounter.YAxisGetCount() * 0.9;     // convert encoder count to acutal ptich angle
  
  measureSpeedStart();
  myPID.Compute();                                            // compute the next PWM output if computeFlag is set
  measureSpeedStop();

  
  motor();                                                    // power up the motor
  
  
  //serialStuff();                                              // send operating data to laptop
}




/* set the rotation and pwm of the motor */
void motor () {
  if ( pwmOutput < 0 ) {
    digitalWrite(MOTOR0_IN1, LOW);
    digitalWrite(MOTOR0_IN2, HIGH);
    Timer1.pwm(MOTOR0_ENA, (-1*pwmOutput));;
  }
  else {
    digitalWrite(MOTOR0_IN1, HIGH);
    digitalWrite(MOTOR0_IN2, LOW);
    Timer1.pwm(MOTOR0_ENA, pwmOutput);;
  }

}


/* doing serial print */
void serialStuff () {
  currentTime = millis();
  
  if (currentTime >= lastTime + SERIAL_PERIOD) {
    Serial.print(desiredPitch);
    Serial.print(",");
    Serial.print(actualPitch);
    Serial.println("");
    lastTime = currentTime;
  }
}



/* measureSpeedStart */
void measureSpeedStart () {
  startTime = micros();
}



/* measureSpeedStop */
void measureSpeedStop () {
  stopTime = micros();
  duration = stopTime - startTime;
  Serial.println(duration);  
}



/* set compute flag which indicates a new PID output should be computed */
void setComputeFlag () {
  computeFlag = 1;  
}



/* ISR for Timer 1 (used for controlling PWM frequency) */
void callback()
{
  digitalWrite(10, digitalRead(10) ^ 1);
}

