#define enA 9
#define in1 6
#define in2 7
#define button 8

int pwmOutput = 0;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Set initial rotation direction
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // pwmOutput = 255 correspond to 100% duty cycle
  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
}

void loop() {
  ;
}
