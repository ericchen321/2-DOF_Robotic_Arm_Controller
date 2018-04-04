#define PITCH_CHIP_RESET 47
#define PITCH_CHIP_OE 48
#define YAW_CHIP_RESET 40
#define YAW_CHIP_OE 41

int decoderCount = 0;

void setup()
{ 
  /* Reset decoders */
  PitchReset();
  YawReset();
  
  /* Turn on serial */
  Serial.begin(115200);
}



void loop() 
{  
  PitchGetCount();
  Serial.println(decoderCount);
  delay(100);
}



void PitchReset() {
  // Set modes for data pins, SEL1, SEL2, OE, RST
  DDRC = B00000000;
  DDRL |= B11000110;  

  digitalWrite(PITCH_CHIP_OE, LOW);
  digitalWrite(PITCH_CHIP_RESET, LOW);
  delay(50);
  digitalWrite(PITCH_CHIP_RESET, HIGH);
}



void YawReset() {
  // set modes for data pins, SEL1, SEL2, OE, RST
  DDRA = B00000000;
  DDRG |= B00000111;
  DDRD |= B10000000;

  digitalWrite(YAW_CHIP_OE, LOW);
  digitalWrite(YAW_CHIP_RESET, LOW);
  delay(50);
  digitalWrite(YAW_CHIP_RESET,HIGH);

}



void PitchGetCount() {
  decoderCount = 0; // reset decoderCount

  PORTL &= B01111111; //digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
  PORTL &= B10111111; //digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
  decoderCount += PINC;
  decoderCount <<= 8;

  PORTL |= B10000000; //digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
  PORTL &= B10111111; //digitalWrite(CHIP1_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
  decoderCount += PINC;
}


void YawGetCount() {
  decoderCount = 0; // reset decoderCount

  PORTD &= B01111111; // digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
  PORTG &= B11111011; // digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
  decoderCount += PINA;
  decoderCount <<= 8;

  PORTD |= B10000000; // digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
  PORTG &= B11111011; // digitalWrite(CHIP0_MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
  decoderCount += PINA;
}

