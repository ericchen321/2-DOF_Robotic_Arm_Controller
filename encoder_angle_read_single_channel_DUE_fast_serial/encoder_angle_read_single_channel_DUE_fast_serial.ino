// encoder
#define encoder0PinA  22

volatile signed long encoder0Pos = 0;
volatile int flag = 0;

void setup() {
  // encoder
  pinMode(encoder0PinA, INPUT);
   
  // first interrupt
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, RISING);
  
  // Serial
  Serial.begin (2000000);
}

void loop() {
  if (flag == 1){
    flag = 0;
    Serial.print(micros());
    Serial.print(",");
    Serial.print(encoder0Pos);
    Serial.println("");   
  }
}


void doEncoderA() {
  // look for a low-to-high on channel A
  flag = 1;
  encoder0Pos ++;
}
