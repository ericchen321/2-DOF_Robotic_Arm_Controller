#define enA 9
#define in1 6
#define in2 7
#define button 8

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Set initial rotation direction
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void loop() {
  // pwmOutput = 255 correspond to 100% duty cycle
  int pwmOutput = 60;
  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
}
