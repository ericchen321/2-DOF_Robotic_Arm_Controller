
#define encoder0PinA = 7
#define encoder0PinB = 8
int freq = 0;
double time0 = 0;
double time1 = 0;

void setup() {
  pinMode (encoder0PinA, INPUT);
  pinMode (encoder0PinB, INPUT);
  Serial.begin (9600);
}

void loop() {
  while (digitalRead(encoder0PinA)==0);
  time0 = millis();
  while (digitalRead(encoder0PinA)==1);
  time1 = millis();
  freq = 1/((time1-time0)*0.001);
  Serial.print (encoder0Pos);
  Serial.print ("/");

}
