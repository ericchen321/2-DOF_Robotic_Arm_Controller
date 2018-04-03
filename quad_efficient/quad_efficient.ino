#include <TimerOne.h>   
#define oePin  48 //active low
#define reset 47
#define sel1Pin  42
#define sel2Pin  43
#define daPin0   37
#define daPin1   36
#define daPin2  35
#define daPin3   34
#define daPin4   33
#define daPin5   32
#define daPin6   31
#define daPin7   30


bool begin=0;
// char pos[8];
bool change = false;
volatile int encoderPos = 0;
volatile int pos;
volatile int start = 0;
volatile int speed = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(oePin, OUTPUT);
  pinMode(reset, OUTPUT);
  digitalWrite(sel1Pin, HIGH);
  digitalWrite(sel2Pin, LOW); //reads lowest 
  digitalWrite(oePin, LOW);

  digitalWrite(reset, LOW);
  delay(100);
  digitalWrite(reset, HIGH);
  
    DDRC = B01100100;
    DDRL = B00011001;
    DDRG = B00000100;
    DDRD = B00000000;
    DDRB = B00000010;
  
}

void loop () {
    Serial.println(qd1());
    delay(10);
}

int qd1()
{
    pos = 0;
    
    // set SEL1 HIGH and SEL2 LOW
    // RESET is always high
    PORTL = B00010001;
    PORTG = B00000000;

    // write lower bits to pos

    pos = (PINL & B00100000) >> 5 |
          (PINC & B00000010) |
          (PINC & B00000001) << 2 |
          (PIND & B10000000) >> 4 |
          (PING & B00000010) << 3 |
          (PING & B00000001) << 5 |
          (PINL & B10000000) >> 1 |
          (PINL & B00000010) << 6; 
    
    // set SEL1 LOW and SEL2 LOW
    // RESET is always high
    PORTL = B00000001;
    PORTG = B00000000;

    // write higher bits to pos
    pos = pos |
          ((PINL & B00100000) >> 5) << 8 |
          ((PINC & B00000010)) << 8 |
          ((PINC & B00000001) << 2) << 8 |
          ((PIND & B10000000) >> 4) << 8 |
          ((PING & B00000010) << 3) << 8 |
          ((PING & B00000001) << 5) << 8 |
          ((PINL & B10000000) >> 1) << 8 |
          ((PINL & B00000010) << 6) << 8; 
          
     return pos;
}



