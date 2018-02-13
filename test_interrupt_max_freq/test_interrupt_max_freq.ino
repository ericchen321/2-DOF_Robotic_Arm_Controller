// encoder
#define encoder0PinA  2

volatile signed long count = 0;

void setup() {
  // encoder
  pinMode(encoder0PinA, INPUT);
 
  // first interrupt
  attachInterrupt(0, doEncoderA, RISING);
  Serial.begin (152000);
}

void loop() {
  if (millis() == 1000){
    serialStuff();
    while(1);
  }
}

// send data to PC
void serialStuff () {
  Serial.println (count);
}


void doEncoderA() {
  // look for a low-to-high on channel A
    count ++ ;
}
