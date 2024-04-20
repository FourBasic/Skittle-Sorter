#include "ColorSensor.h"
#include <Arduino.h>
#include "GeneralFunctions.h"

// Sensor pins & speed in kHz (12,120,600)
ColorSensor::ColorSensor(uint8_t S0, uint8_t S1, uint8_t S2, uint8_t S3, uint8_t out, unsigned int speed) {
    Pin_S0 = S0;
    Pin_S1 = S1;
    Pin_S2 = S2;
    Pin_S3 = S3;
    Pin_Out = out;
    
    pinMode(Pin_S0, OUTPUT);
    pinMode(Pin_S1, OUTPUT);
    pinMode(Pin_S2, OUTPUT);
    pinMode(Pin_S3, OUTPUT);
    pinMode(Pin_Out, INPUT);

    digitalWrite(Pin_S0, (speed != 12));
    digitalWrite(Pin_S1, (speed != 120));

    sampleLowest[0] = 9999;
    sampleLowest[1] = 9999;
    sampleLowest[2] = 9999;
}

//	
void ColorSensor::scan(unsigned int filterSwitchDelay) {
   // Setting RED (R) filtered photodiodes to be read
   digitalWrite(Pin_S2, LOW); //  Expect that PINS are already in this state
   digitalWrite(Pin_S3, LOW);
   delay(filterSwitchDelay);
   scanResult[0] = pulseIn(2, LOW);

   // Setting GREEN (G) filtered photodiodes to be read
   digitalWrite(Pin_S2, HIGH);//H
   digitalWrite(Pin_S3, HIGH); //H
   delay(filterSwitchDelay);
   scanResult[1] = pulseIn(2, LOW);

   // Setting BLUE (B) filtered photodiodes to be read
   digitalWrite(Pin_S2, LOW); //L
   digitalWrite(Pin_S3, HIGH);//H
   delay(filterSwitchDelay);
   scanResult[2] = pulseIn(2, LOW);

   //  Return to initial filter for next cycle
   digitalWrite(Pin_S2, LOW);
   digitalWrite(Pin_S3, LOW);

 // #define debugScanColor
#ifdef debugScanColor
Serial.print(" ScanColor#");
for (auto rgb : scanResult)
{
Serial.print(rgb);
Serial.print("/");
}
delay(100);
#endif
}

// Checkes for match between any two RGB values
bool ColorSensor::match(unsigned int c1[], unsigned int c2[], unsigned int tol)
{
   bool result = 1;
   for (int i = 0; i < 3; i++)
   {
      if (!withinRange(c1[i], c2[i], tol))
      {
         result = 0;
         break;
      }
   }
   return result;
}

// Saves the highest and lowest scanResult values
// from a series of calls and outputs to Serial for debugging.
//#define debugSampleRange
void ColorSensor::sampleRange()
{
   for (int i = 0; i < 3; i++)
   {
      if (scanResult[i] < sampleLowest[i])
      {
         sampleLowest[i] = scanResult[i];
      }
      if (scanResult[i] > sampleHighest[i])
      {
         sampleHighest[i] = scanResult[i];
      }
      #ifdef debugSampleRange
         Serial.print("Range ");
         Serial.print(i);
         Serial.print(" From ");
         Serial.print(sampleLowest[i]);
         Serial.print(" To ");
         Serial.println(sampleHighest[i]);
      #endif
   }
   #ifdef debugSampleRange
      Serial.println("");
   #endif
   //delay(500);
}

