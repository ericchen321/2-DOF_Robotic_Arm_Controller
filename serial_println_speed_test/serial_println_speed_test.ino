unsigned long int time0 = 0;
unsigned long int time1 = 1;
signed long testVal = 12345;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(4000000);
}

void loop() {
  time0 = micros();
  Serial.println (testVal);
  time1 = micros ();
  Serial.println ((time1-time0));
}
