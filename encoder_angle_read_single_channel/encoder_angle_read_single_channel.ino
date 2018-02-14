// encoder
#define encoder0PinA  2
#define encoder0PinB  3

volatile signed long encoder0Pos = 0;
volatile unsigned char flag = 0;

void setup() {
  // encoder
  pinMode(encoder0PinA, INPUT);
   
  // first interrupt
  attachInterrupt(0, doEncoderA, RISING);
  
  // Serial
  Serial.begin (2000000);
}

void loop() {
  if (flag == 1){
    flag = 0;
    Serial.print (micros());
    Serial.print (",");
    Serial.print (encoder0Pos);
    Serial.println("");  
  }
}


void doEncoderA() {
  // look for a low-to-high on channel A
  flag = 1;
  encoder0Pos ++;
}
