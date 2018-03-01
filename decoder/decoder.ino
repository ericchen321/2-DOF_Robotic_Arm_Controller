#define OE 1
#define SEL1 4
#define SEL2 5
#define D1 8
#define D2 10
#define D3 11
#define D4 12

long int Pos;
int M_S_B;
int SECOND_BIT;
int THIRD_BIT;
int L_S_B;


void setup() {
  pinMode(OE, OUTPUT);
  pinMode(SEL1, OUTPUT);
  pinMode(SEL2, OUTPUT);
  pinMode(D1, INPUT);
  pinMode(D2, INPUT);
  pinMode(D3, INPUT);
  pinMode(D4, INPUT);

  Serial.begin (9600);
}

void loop() {
  digitalWrite(OE, HIGH);
  delay(25);

  digitalWrite(SEL1, LOW);
  digitalWrite(SEL2, HIGH);
  digitalWrite(OE, LOW);

  MSB_D();

  digitalWrite(SEL1, HIGH);
  digitalWrite(SEL2, HIGH);

  BIT_2();

  digitalWrite(SEL1, LOW);
  digitalWrite(SEL2, LOW);

  BIT_3();

  digitalWrite(SEL1, HIGH);
  digitalWrite(SEL2, LOW);

  LSB_D();

  digitalWrite(OE, HIGH);

  delay(25);

  Pos = M_S_B;
  Pos = Pos << 8;
  Pos = SECOND_BIT;
  Pos = Pos << 8;
  Pos = THIRD_BIT;
  Pos = Pos << 8;
  Pos = L_S_B;

  Serial.println (Pos);

}

void MSB_D() {
  int value;
  int value1;

  value = digitalRead(D4);
  value1 = digitalRead(D4);

  if(value == value1) {
    M_S_B = value1;
  }
  else {
    BIT_2();
  }
}

void BIT_2() {
  int value;
  int value1;

  value = digitalRead(D3);
  value1 = digitalRead(D3);

  if(value == value1) {
    SECOND_BIT = value1;
  }
  else {
   BIT_3();
  }
  
}

void BIT_3() {
  int value;
  int value1;

  value = digitalRead(D2);
  value1 = digitalRead(D2);

  if(value == value1) {
    THIRD_BIT = value1;
  }
  else {
    LSB_D();
  }
  
}

void LSB_D() {
  int value;
  int value1;

  value = digitalRead(D1);
  value1 = digitalRead(D1);

  if(value == value1) {
    L_S_B = value1;
  }
  else {
    LSB_D();
  }

 
}


