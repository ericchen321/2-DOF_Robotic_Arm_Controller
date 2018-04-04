/* 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Reading decoder HCTL-2022 to count pulses from
an encoder using Arduino Mega 2560
Written by Hanna Hellman, 2016-07-08
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Arduino Pin       HCTL-2022 function
------------------------------------
DIG 9             /RST (active low)
DIG 11            SEL1
DIG 12            SEL2
DIG 13            /OE (active low)
DIG 22->29        Data pins D0->D7
+5V               VDD
GND               VSS
from encoder      CHA
from encoder      CHB
not connected     INDEX
not connected     U/D
not connected     TEST 
External clock    CLK
*/

// Global variable declaration
volatile long countData = 0; // 32-bit int (signed?) the count represents absolute position


void setup() {
  Serial.begin(9600);

  pinMode(9, OUTPUT);
  digitalWrite(9, LOW); // /RST resets the internal counter between runs
  delay(10);
  digitalWrite(9, HIGH); // Stay high for rest of the run
  
  // Set all pins in PORTA (digital pins 22->29 on the Mega) as input pins
  for(int i = 22; i<30; i++) {
    pinMode(i, INPUT);
  }
  // Set pins 5,6,7 in PORT B (digital pins 11,12,13 on the Mega) as output pins
  for(int j = 11; j<14; j++) {
    pinMode(j, OUTPUT);
    digitalWrite(j, LOW);
    
  }

}
/*-------------------------------------------------
  -------------------Functions---------------------
  -------------------------------------------------*/
byte getMSB(){
/*Get stable data for the most significant byte of countData*/

  byte MSBold = PINA;       // read datapins D0->D7 and store in MSBold
  byte MSBnew = PINA;       // read again immediatly after to assure stable data
  if (MSBnew == MSBold){ 
    byte MSBresult = MSBnew;
    return MSBresult;  
  }
  else getMSB();
}

byte getSecond(){
/*Get stable data for the 2nd byte of countData*/
  byte secondOld = PINA;       // read datapins D0->D7 and store in secondOld
  byte secondNew = PINA;       // read again immediatly after to assure stable data
  if (secondNew == secondOld){ 
    byte secondResult = secondNew;
    return secondResult;  
  }
  else getSecond();
}

byte getThird(){
/*Get stable data for the 3rd byte of countData*/
  byte thirdOld = PINA;       // read datapins D0->D7 and store in thirdOld
  byte thirdNew = PINA;       // read again immediatly after to assure stable data
  if (thirdNew == thirdOld){ 
    byte thirdResult = thirdNew;
    return thirdResult;  
  }
  else getThird();
}

byte getLSB(){
/*Get stable data for the least significant byte of countData*/
  byte LSBold = PINA;       // read datapins D0->D7 and store in LSBold
  byte LSBnew = PINA;       // read again immediatly after to assure stable data
  if (LSBnew == LSBold){  
    byte LSBresult = LSBnew;
    return LSBresult;  
  }
  else getLSB();
}

long mergeFunc(byte MSBresult, byte secondResult, byte thirdResult, byte LSBresult){
/*Merges the 4 bytes returning one 32-bit variable called countData*/
  long tempVar = 0;
  tempVar |= ((long) MSBresult << 24) | ((long) secondResult << 16) | ((long) thirdResult << 8) | ((long) LSBresult << 0);
  countData = tempVar;
  return countData;
}



void loop() {

  digitalWrite(13, HIGH); // Set OE to HIGH (disable)
  delay(25);  // need a better way
  
  digitalWrite(11, LOW);
  digitalWrite(12, HIGH); // SEL1 = 0 and SEL2 = 1
  
  digitalWrite(13, LOW); // Set OE to LOW (enable)
  byte MSBresult = getMSB();

  
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH); // SEL1 = 1 and SEL2 = 1
  byte secondResult = getSecond();
  
  digitalWrite(11, LOW);
  digitalWrite(12, LOW); // SEL1 = 0 and SEL2 = 0
  byte thirdResult = getThird();
  
  digitalWrite(11, HIGH);
  digitalWrite(12, LOW); // SEL1 = 1 and SEL2 = 0
  byte LSBresult = getLSB();

  digitalWrite(13, HIGH); // Set OE to HIGH (disable)
  delay(25);
  
  countData = mergeFunc(MSBresult, secondResult, thirdResult, LSBresult);
  Serial.println("Counter: ");
  Serial.println(countData);
  Serial.println("\n");


  }
  
  /*
  Algorithm:
 
  disable OE (high)
  wait 25 ms
  set sel1= 0 and sel2 = 1 to read MSB
  enable OE (low)

  getMSB()
  set sel1 =1 and sel2=1
  get2nd()
  set sel1 = 0 and sel2 = 0
  get3rd()
  set sel1 = 1 and sel2 = 0
  getLSB()

  disable OE (high)
  wait 25 ms
  */

  

  

