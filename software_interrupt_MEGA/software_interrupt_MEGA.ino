/* Define pins */
#define ENCODER1_PINA  2
#define ENCODER1_PINB  3

/* Define variables for the dual channel encoder reading algorithm */
volatile signed long encoder1Pos = 0;
void setup() {
  /* wait for a while */
  delay(1000);

  /* set pin modes */
  pinMode(ENCODER1_PINA, INPUT);
  pinMode(ENCODER1_PINB, INPUT);

  /* initialize software interrupts */
  attachInterrupt(0, doEncoder1_A, CHANGE);
  attachInterrupt(1, doEncoder1_B, CHANGE);

  /* initialize serial communication */
  Serial.begin(115200);
}



void loop() {
  Serial.println(encoder1Pos);
  delay(10);
}



void doEncoder1_A() {
  // look for a low-to-high on channel A
  if (digitalRead(ENCODER1_PINA) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER1_PINB) == LOW) {
      encoder1Pos = encoder1Pos + 1;         // CW
    }
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER1_PINB) == HIGH) {
      encoder1Pos = encoder1Pos + 1;          // CW
    }
    else {
      encoder1Pos = encoder1Pos - 1;          // CCW
    }
  }
}





void doEncoder1_B() {
  // look for a low-to-high on channel B
  if (digitalRead(ENCODER1_PINB) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(ENCODER1_PINA) == HIGH) {
      encoder1Pos = encoder1Pos + 1;         // CW
    }
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER1_PINA) == LOW) {
      encoder1Pos = encoder1Pos + 1;          // CW
    }
    else {
      encoder1Pos = encoder1Pos - 1;          // CCW
    }
  }
}

