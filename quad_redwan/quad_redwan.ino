//PIN LABELS
const int D0 = 37;
const int D1 = 36;
const int D2 = 35;
const int D3 = 34;
const int D4 = 33;
const int D5 = 32;
const int D6 = 31;
const int D7 = 30;
const int UD = 8;
const int SEL1 = 42;
const int OE = 48;
const int RESET = 47;
const int SEL2 = 43;
const int DIR1 = 51;
const int DIR2 = 53;
const int PWM = 46;

int DATA = 0;
int DATA_0 = 0;
int DATA_1 = 0;
int DATA_2 = 0;
int DATA_3 = 0;
int DATA_4 = 0;
int DATA_5 = 0;
int DATA_6 = 0;
int DATA_7 = 0;

int result_hi = 0;
int result_second = 0;
int result_third = 0;
int result_lo = 0;
int result_count;

int mult = 1;
int temp;
int wait_time = 0;

volatile bool ang_flag;
volatile bool get_ang1_flag = 0;
volatile bool send_pwm_flag = 0;

double Kp = 5;// you can set these constants however you like depending on trial & error
double Ki = 0;
double Kd = 6;

float last_error = 0;
float error = 0;
float changeError = 0;
float totalError = 0;
float pidTerm = 0;
float pidTerm_scaled = 0;// if the total gain we get is not in the PWM range we scale it down so that it's not bigger than |255|

double angle = 0;
double setpoint = 180;

void setup() {
  Serial.begin(115200);
 
//========ISR TIMER CONFIG========//

 cli();//stop interrupts
 
//set timer0 interrupt at 2kHz
  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);   
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  
//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

//set timer2 interrupt at 8kHz
/*  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
*/
//set timer2 interrupt at 2kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 and CS20 bits for 64 prescaler
  TCCR2B |= (1 << CS21) | (1 << CS20);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

sei();//allow interrupts

//=====END ISR CONFIG=====//

  //PIN CONFIGURATION
  
  pinMode(SEL1, OUTPUT);    //SEL1
  pinMode(SEL2, OUTPUT);    //SEL2 
  pinMode(OE, OUTPUT);      //OE (active low)
  pinMode(RESET, OUTPUT);   //Encoder reset (active low)
  pinMode(UD, OUTPUT);      //Up down counter (active low)
  pinMode(DIR1, OUTPUT);    //Motor direction 1
  pinMode(DIR2, OUTPUT);    //Motor direction 2
  pinMode(PWM, OUTPUT);     //PWM output

  pinMode(D0, INPUT);   //
  pinMode(D1, INPUT);   //
  pinMode(D2, INPUT);   //
  pinMode(D3, INPUT);   //
  pinMode(D4, INPUT);   //
  pinMode(D5, INPUT);   //
  pinMode(D6, INPUT);   //
  pinMode(D7, INPUT);   //

  
  //Event based ISRs
//  attachInterrupt(ang_flag,calc_ang,HIGH);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                                                                                                                                                               /////
/////                                                                  Interrupt Service Routines                                                                                                   /////
/////                                                                                                                                                                                               /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ISR0
ISR(TIMER0_COMPA_vect){ 

}  

//ISR1 
ISR(TIMER1_COMPA_vect){

}

//ISR2
//Toggles every x seconds to read quadrature decoder count output
ISR(TIMER2_COMPA_vect){

 // get_ang1_flag = HIGH;
get_ang1();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                                                                                                                                                               /////
/////                                                                  Data stability functions                                                                                                     /////
/////                                                                                                                                                                                               /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This function ensures the MSB is stable before storing the value
void Get_hi() {

 while(1) {
  int hi_old = (digitalRead(D7) << 7 | digitalRead(D6) << 6 | digitalRead(D5) << 5 | digitalRead(D4) << 4 | digitalRead(D3) << 3 | digitalRead(D2) << 2 | digitalRead(D1) << 1 | digitalRead(D0) << 0);
  int hi_new = (digitalRead(D7) << 7 | digitalRead(D6) << 6 | digitalRead(D5) << 5 | digitalRead(D4) << 4 | digitalRead(D3) << 3 | digitalRead(D2) << 2 | digitalRead(D1) << 1 | digitalRead(D0) << 0);
 
  if (hi_new == hi_old){
    result_hi = hi_new;
    return;
  }

 }
}

//This function ensures the 2nd byte is stable before storing the value
void Get_second() {

 while(1) {
 int second_old = (digitalRead(D7) << 7 | digitalRead(D6) << 6 | digitalRead(D5) << 5 | digitalRead(D4) << 4 | digitalRead(D3) << 3 | digitalRead(D2) << 2 | digitalRead(D1) << 1 | digitalRead(D0) << 0);
  int second_new = (digitalRead(D7) << 7 | digitalRead(D6) << 6 | digitalRead(D5) << 5 | digitalRead(D4) << 4 | digitalRead(D3) << 3 | digitalRead(D2) << 2 | digitalRead(D1) << 1 | digitalRead(D0) << 0);
 
 if (second_new == second_old){
  result_second = second_new;
  return;
 }

 }
}

//This function ensures the third byte is stable before storing the value
void Get_third() {

 while(1) {
  int third_old = (digitalRead(D7) << 7 | digitalRead(D6) << 6 | digitalRead(D5) << 5 | digitalRead(D4) << 4 | digitalRead(D3) << 3 | digitalRead(D2) << 2 | digitalRead(D1) << 1 | digitalRead(D0) << 0);
  int third_new = (digitalRead(D7) << 7 | digitalRead(D6) << 6 | digitalRead(D5) << 5 | digitalRead(D4) << 4 | digitalRead(D3) << 3 | digitalRead(D2) << 2 | digitalRead(D1) << 1 | digitalRead(D0) << 0);
 
 if (third_new == third_old){
  result_third = third_new;
  return;
 }

 }
}

//This function ensures the LSB is stable before storing the value
void Get_lo() {

 while(1) {
  int lo_old = (digitalRead(D7) << 7 | digitalRead(D6) << 6 | digitalRead(D5) << 5 | digitalRead(D4) << 4 | digitalRead(D3) << 3 | digitalRead(D2) << 2 | digitalRead(D1) << 1 | digitalRead(D0) << 0);
  int lo_new = (digitalRead(D7) << 7 | digitalRead(D6) << 6 | digitalRead(D5) << 5 | digitalRead(D4) << 4 | digitalRead(D3) << 3 | digitalRead(D2) << 2 | digitalRead(D1) << 1 | digitalRead(D0) << 0);
 
 if (lo_new == lo_old){
  result_lo = lo_new;
  return;
 }

 }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                                                                                                                                                               /////
/////                                                                  Quadrature decoder count conversion                                                                                          /////
/////                                                                                                                                                                                               /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This function combines the bytes acquired from the quadrature decoder into 1, 4 byte variable
void calc_ang() {
  
  mult = 1;
  temp = result_lo*mult;    //Assign LSB
  result_count = temp;

  mult = mult*256;
  temp = result_third*mult;   //Assign 3rd byte
  result_count = result_count + temp;

/*  mult = mult*256;
  temp = result_second*mult;   //Assign 2nd byte
  result_count = result_count + temp;

  mult = mult*256;
  temp = result_hi*mult;   //Assign MSB byte
  result_count = result_count + temp;
 // ang_flag = LOW;*/
  
}

void get_ang1() {
  
  //SET BITS TO ACQUIRE MSB
//  digitalWrite(OE,HIGH);
//  digitalWrite(SEL1,LOW);
 // digitalWrite(SEL2,HIGH);
  digitalWrite(OE,LOW);
//  Get_hi();

  //SET BITS TO ACQUIRE 2ND BYTE
//  digitalWrite(SEL1,HIGH);
//  digitalWrite(SEL2,HIGH);
//  Get_second();

  //SET BITS TO ACQUIRE 3RD BYTE
  digitalWrite(SEL1,LOW);
  digitalWrite(SEL2,LOW);
  Get_third();

  //SET BITS TO ACQUIRE LSB
  digitalWrite(SEL1,HIGH);
  digitalWrite(SEL2,LOW);
  Get_lo();

  digitalWrite(OE,HIGH);
  calc_ang();
 // get_ang1_flag =LOW;  
}

void send_pwm1() {
   
  PIDcalculation();// find PID value
  analogWrite(PWM, pidTerm_scaled);
  
}

void ang1_check() {

  //If setpoint hasnt been reached, keep going forward
  if (angle < (setpoint - 4)) {     
   digitalWrite(DIR1, HIGH);// Forward motion
   digitalWrite(DIR2, LOW);
   //   wait_time = 0;
  } 
  
  //If setpoint has been overshot, come back
  else if (angle > (setpoint + 4)) {
   digitalWrite(DIR1, LOW);// reverse  motion
   digitalWrite(DIR2, HIGH);
//   wait_time = 0;
  }

    else {
    digitalWrite(DIR1, LOW);// stop
    digitalWrite(DIR2, LOW);
 //   wait_time++;
 //   Serial.print(wait_time);
  } 
 
  /*if(wait_time == 3000){
    setpoint = 0 - setpoint;
    wait_time = 0;
   }
*/
}

void PIDcalculation(){
  angle = (0.9 * result_count);//count to angle conversion
  error = setpoint - angle;
  
  changeError = (error - last_error); // derivative term
  totalError += error; //accumalate errors to find integral term
  pidTerm = (Kp * error) + (Ki * totalError) + (Kd * changeError);//total gain
  pidTerm = constrain(pidTerm, -255, 255);//constraining to appropriate value
  pidTerm_scaled = abs(pidTerm);//make sure it's a positive value

  last_error = error;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                                                                                                                                                                               /////
/////                                                                  Main Function                                                                                                                /////
/////                                                                                                                                                                                               /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
   
 digitalWrite(RESET,LOW); //reset at start of program
 delay(1);
 digitalWrite(RESET,HIGH);   //turn reset off

 digitalWrite(UD,LOW);    //NOT SURE WHAT THIS DOES, HAVE TRIED BOTH LOW AND HIGH, SAME COUNT DIRECTION

 
 while(1){

 /* if(get_ang1_flag){
    //get_ang1();
    send_pwm1();
  }*/
  //send_pwm1();
  //ang1_check();
     
  //Serial.print(angle);
  Serial.print(result_count);
  //Serial.print(get_ang1_flag);
  Serial.print('\n');
 }
  
}





