// encoder
#define encoder0PinA  2
#define encoder0PinB  3


// motor
#define enA 9
#define in1 6
#define in2 7


volatile signed long encoder0Pos = 0;
int flag = 0;

void setup() {
  // encoder
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);

  // motor
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
//  pinMode(A4, INPUT);
  int pwmOutput = 255;
  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
 
  // first interrupt
  attachInterrupt(0, doEncoderA, RISING);
  // second interrupt
  Serial.begin (2000000);
}

void loop() {
  motor ();
  encoder();
  serialStuff();
}


// motor function, it outputs pwm
void motor () {
  ;
}

// calculates the relative angle at which the motor's shaft is
void encoder () {
  ;
}


// send data to PC
void serialStuff () {
   if (flag == 1) {
    flag = 0; 
    Serial.println (micros());  
   }
}


void doEncoderA() {
  // look for a low-to-high on channel A
    flag = 1;
}
