/* included necessary libraries */
#include <PID_v1_modified_v3.h>
#include <SetPoint_v1.h>
#include <TimerOne.h>
#include <FlexiTimer2.h>


/* Define pins */
#define MOTOR0_ENA 9
#define MOTOR0_IN1 6
#define MOTOR0_IN2 7
#define ENCODER0_PINA  2
#define ENCODER0_PINB  3

/* Define other macros */
#define PID_SAMPLE_TIME 1 // PID Sample Time defined in ms
#define SETPOINT_SIZE 126
#define SETPOINT_TIME 0.08
#define RADIUS 0.06 // defined in m
#define HEIGHT 0.1 // defined in m
#define FRAME_PER_SEC 1
#define SERIAL_PERIOD 40  // specified in ms

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
double Ki=0, Kd=2.6;
double Kp=13*Kd;
SetPoint mySetPoint(HEIGHT, SETPOINT_SIZE, desiredXArray, desiredYArray, &desiredX, &desiredY, &desiredYaw, &desiredPitch);
PID myPID(&actualPitch, &pwmOutput, &desiredPitch, Kp, Ki, Kd, DIRECT);



/* Define variables for serial */
signed long currentTime = 0;
signed long lastTime = 0;


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


  /* Set initial rotation direction and pwm */
  digitalWrite(MOTOR0_IN1, HIGH);
  digitalWrite(MOTOR0_IN2, LOW);
  Timer1.initialize(200);                 // initialize timer1, and set a 5kHZ frequency
  Timer1.pwm(MOTOR0_ENA, 0);              // setup initial pwm on MOTOR0_ENA
  Timer1.attachInterrupt(callback);       // attaches callback() as a timer overflow interrupt



  /* initialize serial communication */
  Serial.begin(2000000);


  /* wait for a while */
  delay(5000);


  /* initialize desired x,y coordinates */
  for (i = 0; i < SETPOINT_SIZE; i++){
    desiredYArray[i] = i * SETPOINT_TIME;  
  }
  for (i = 0; i < SETPOINT_SIZE; i++){
    desiredYArray[i] = RADIUS * sin ( 2 * 3.14 * FRAME_PER_SEC * desiredYArray[i] );
  }
  

  /* initialize actual and desired angle for the PID algorithm */
  actualPitch = 0;
  desiredPitch = 180;


  /* turn the PID on */
  myPID.SetMode(AUTOMATIC);
  

  /* Set PID algorithm sampling time */
  FlexiTimer2::set(PID_SAMPLE_TIME, encoder);
  FlexiTimer2::start();
}



void loop() {
  setPoint();
  myPID.Compute();  
  motor();
  serialStuff();
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
    Timer1.pwm(MOTOR0_ENA, (-1*6*pwmOutput));;
  }
  else {
    digitalWrite(MOTOR0_IN1, HIGH);
    digitalWrite(MOTOR0_IN2, LOW);
    Timer1.pwm(MOTOR0_ENA, 6*pwmOutput);;
  }

}


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



void callback()
{
  digitalWrite(10, digitalRead(10) ^ 1);
}

