// encoder
#define encoder0PinA  2
#define encoder0PinB  3
#define ENCODER0_RES 3.6
#define RAD_PER_DEG 0.01745

// serial
#define SERIAL_PERIOD 1 // serial speed specified in ms

volatile signed long encoder0Pos = 0;
float encoder0Vel = 0;

unsigned long lastTime = 0;
unsigned long currentTime;

void setup() {
  // encoder
  pinMode(encoder0PinA, INPUT);
   
  // first interrupt
  attachInterrupt(0, doEncoderA, RISING);
  
  // Serial
  Serial.begin (2000000);
}

void loop() {
  currentTime = millis();
  
  if (currentTime >= lastTime + SERIAL_PERIOD){
    encoder0Vel = (encoder0Pos * ENCODER0_RES * RAD_PER_DEG)/(SERIAL_PERIOD * 0.001);
    Serial.print (currentTime);
    Serial.print (",");
    Serial.print (encoder0Vel);
    Serial.println(""); 
    encoder0Pos = 0;
    lastTime = currentTime; 
  }
}


void doEncoderA() {
  // look for a low-to-high on channel A
  encoder0Pos ++;
}
