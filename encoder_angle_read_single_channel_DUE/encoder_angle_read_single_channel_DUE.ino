// encoder
#define encoder0PinA  22

#define ARRAY_SIZE 3000

volatile signed long encoder0Pos = 0;
signed long encoder0PosArray[ARRAY_SIZE];
signed long timeArray[ARRAY_SIZE];
int i = 0;

void setup() {
  // encoder
  pinMode(encoder0PinA, INPUT);
   
  // first interrupt
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, RISING);
  
  // Serial
  Serial.begin (115200);
}

void loop() {
  delay(2);
  encoder0PosArray[i] = encoder0Pos;
  timeArray[i] = micros();
  i++;

  if (i>ARRAY_SIZE){
    for(i = 0; i<=ARRAY_SIZE; i++){
      Serial.print(timeArray[i]);
      Serial.print(",");
      Serial.print(encoder0PosArray[i]);
      Serial.println("");
    }
    while(1);
  }
}


void doEncoderA() {
  // look for a low-to-high on channel A
  encoder0Pos ++;
}
