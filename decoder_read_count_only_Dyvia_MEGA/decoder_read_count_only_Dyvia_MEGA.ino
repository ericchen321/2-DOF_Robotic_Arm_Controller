#include <MegaEncoderCounter_v4.h>
#include <TimerFive.h>                  // used library found on https://playground.arduino.cc/Code/Timer1

#define reset 40
#define oe 41


MEGAEncoderCounter MegaEncoderCounter; // Initializes the MEGA Encoder Counter

int count = 0;

void setup()
{

  pinMode(reset, OUTPUT);
  pinMode(oe, OUTPUT);

  digitalWrite(oe, LOW);
  digitalWrite(reset, LOW);
  delay(50);
  digitalWrite(reset,HIGH);

  DDRA = B00000000;
//  DDRL = B11000110;
  DDRG = B00000111;
  DDRD = B10000000;
  DDRA = B00000000; 
  
  
  
  /* Turn on serial */
  Serial.begin(115200);
}

void loop() 
{  
  qd2();
}

void qd2()
{
  int pos = 0;
  PORTD = B10000000;
  PORTG = B00000010;

  pos = PINA;

  PORTD = B00000000;
  PORTG = B00000010;
  
  pos = pos |
        (PINA << 8);

  Serial.println(pos);
}

