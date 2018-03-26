// encoder
#define encoder0PinA  2
#define encoder0PinB  3


// motor
#define enA 9
#define in1 6
#define in2 7
volatile signed long encoder0Pos = 0;
volatile double angle;
int flag = 0;
volatile long Currenttime = 0;
int count = 0;
double voltage;//

void setup() {
  // encoder
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);

  // motor
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // pinMode(A4, INPUT);
  int pwmOutput = 255;
  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
 
  // first interrupt
  attachInterrupt(0, doEncoderA, CHANGE);
  // second interrupt
  attachInterrupt(1, doEncoderB, CHANGE);
  Serial.begin (2000000);
}

void loop() {
  motor ();
  encoder();
}


// motor function, it outputs pwm
void motor () {
  
  // pwmOutput = 255 correspond to 100% duty cycle
      
 /*
 if(encoder0Pos >= 200) {
 digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  if(encoder0Pos < 0) {

digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  */
}

// calculates the relative angle at which the motor's shaft is
void encoder () {
  if((encoder0Pos % 4) == 0) {
    Serial.print (encoder0Pos);
    Serial.print (",");
    //Currenttime = micros();
    //Serial.print (Currenttime);  
    Serial.println ("");
  }
  
}



void doEncoderA() {
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

void doEncoderB() {
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
