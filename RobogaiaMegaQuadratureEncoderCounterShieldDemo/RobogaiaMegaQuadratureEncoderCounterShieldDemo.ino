#include <MegaEncoderCounter.h>

MegaEncoderCounter megaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count Mode

char incomingByte;
unsigned long count;
unsigned long rgcount;
int qualityTestHigh[32];
int qualityTestLow[32];


void setup()
{
   Serial.begin(9600);
   RobogaiaMegaQuadratureEncoderCounterShieldDemoHelp();
   
   int qualityTestIter;
   for(qualityTestIter=0; qualityTestIter<32; qualityTestIter++)
    {
      qualityTestHigh[qualityTestIter]=0;
      qualityTestLow[qualityTestIter]=0;
   }
   Serial.println(" [Power up or Reset of Arduino Occured]");
   Serial.println("Robogaia Mega Quadrature Encoder Counter Shield Demo -> ");
}

void loop() 
{ 
   unsigned char iter;
   if (Serial.available() > 0) 
   {
      incomingByte = Serial.read();
      if(incomingByte == '\r')
      {
         return;
      }
      switch(incomingByte)
      {
         case 'u':
            Serial.println("Resetting the quadrature encoder X counter");
            megaEncoderCounter.XAxisReset( );
            break;
         case 'i':
            Serial.println("Reading the quadrature encoder X counter");
            count = megaEncoderCounter.XAxisGetCount();
            Serial.print("Binary   : ");
            Serial.print(count, BIN);
            Serial.println(" ");
            break;
         case 'o':
            Serial.println("Reading the quadrature encoder X counter extended");
            for( iter = 0 ; iter < 255 ; iter++)
            {
               count = megaEncoderCounter.XAxisGetCount();
               Serial.print("Decimal    : ");
               Serial.print(count);
               Serial.println(" ");
               delay(100);
            }
            break;
         case 'j':
            Serial.println("Resetting the quadrature encoder Y counter");
            megaEncoderCounter.YAxisReset( );
            break;
         case 'k':
            Serial.println("Reading the quadrature encoder Y counter");
            count = megaEncoderCounter.YAxisGetCount();
            Serial.print("Decimal    : ");
            Serial.print(count);
            Serial.println(" ");
            break;
         case 'l':
            Serial.println("Reading the quadrature encoder Y counter extended");
            for( iter = 0 ; iter < 255 ; iter++)
            {
               count = megaEncoderCounter.YAxisGetCount();
               Serial.print("Decimal    : ");
               Serial.print(count);
               Serial.println(" ");
               delay(100);
            }
            break;
         case 'b':
            Serial.println("Changine the quadrature encoder counter to count mode 1X");
            megaEncoderCounter.switchCountMode(1);
            break;
          case 'n':
            Serial.println("Changine the quadrature encoder counter to count mode 2X");
            megaEncoderCounter.switchCountMode(2);
            break;
          case 'm':
            Serial.println("Changine the quadrature encoder counter to count mode 4X");
            megaEncoderCounter.switchCountMode(4);
            break;
          case 'x':
            Serial.println("RG Quality Test X Axis");
            for(;;)
            {
               rgcount = megaEncoderCounter.XAxisGetCount();
               printrgQATest();
            }
            break;
          case 'y':
            Serial.println("RG Quality Test Y Axis");
            for(;;)
            {
               rgcount = megaEncoderCounter.YAxisGetCount();
               printrgQATest();
            }
            break;  
         case 'h':
         default:
            Serial.println("Unknown Entry Event");
            RobogaiaMegaQuadratureEncoderCounterShieldDemoHelp();
            break;
      }
      Serial.println("Robogaia Mega Quadrature Encoder Counter Shield Demo -> ");
   }
   
}

void RobogaiaMegaQuadratureEncoderCounterShieldDemoHelp()
{
   Serial.println("Robogaia Mega Quadrature Encoder Counter Shield Demo"); 
   Serial.println(" u -                    Reset X Axis quadrature encoder counter");
   Serial.println(" i -                    Read X Axis quadrature encoder counter");   
   Serial.println(" o -                    Read X Axis quadrature encoder counter extended, Reads and Prints the count 255 times over 25 seconds"); 
   Serial.println(" j -                    Reset Y Axis quadrature encoder counter");
   Serial.println(" k -                    Read Y Axis quadrature encoder counter");   
   Serial.println(" l -                    Read Y Axis quadrature encoder counter extended, Reads and Prints the count 255 times over 25 seconds");
   Serial.println(" b -                    Change Count Mode to 1X");
   Serial.println(" n -                    Change Count Mode to 2X");
   Serial.println(" m -                    Change Count Mode to 4X"); 
   Serial.println(" x -                    Quality Test X Axis");   
   Serial.println(" y -                    Quality Test Y Axis");   
   Serial.println(" h -                    Shows this Help Screen");   
}

void printrgQATest()
{
   unsigned long iter;

   for(iter = 0 ; iter<32; iter++)
   {
      if(IsBitSet(rgcount, iter) == 1 )
      {
         qualityTestHigh[iter] = 1;
      }
      else
      {
         qualityTestLow[iter] = 1;
      }
      
      if(qualityTestHigh[iter] == 0 && qualityTestLow[iter] == 0)
      {
        Serial.print(" ");
      }
      else if(qualityTestHigh[iter] == 1 && qualityTestLow[iter] == 0)
      {
         Serial.print("H");
      }
      else if( qualityTestHigh[iter] == 0 && qualityTestLow[iter] == 1)
      {
         Serial.print("L");
      }
      else
      {
         Serial.print("P");
      }
   }
   Serial.println(""); 
}

unsigned long IsBitSet(unsigned long b, unsigned long pos)
{
   unsigned long result;
   if((b & (((unsigned long)1) << pos)) != 0)
   {
      return 1;
   }
   else
   {
      return 0;
   }
}

