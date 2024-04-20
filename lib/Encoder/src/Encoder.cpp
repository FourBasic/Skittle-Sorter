#include "Encoder.h"
#include <Arduino.h>

Encoder::Encoder(unsigned int pulsePerRotation, unsigned int indexCount, bool moduloPositioning) {
    ppr = pulsePerRotation;
    ic = indexCount;
    ppi = ppr / ic;
    modeModulo = moduloPositioning;
}

// Updates info based on high speed counter value
// Returns current updated postion
int Encoder::update(long rollingCount) {
    //return cPos;
    if(modeModulo){ cPos = correctOverflow(rollingCount); }
    else { cPos = (int) rollingCount; }
    return cPos;
}

// Correct the postion if the high speed
// counter rolls over ppr between calls.
unsigned int Encoder::correctOverflow(long rollingCount) {
 int result;
 float maxDivisor;
 if (rollingCount >= ppr) {  //  Positive Rollover
   maxDivisor = floor(rollingCount/ppr);
   result = rollingCount - (ppr*maxDivisor);
 } else if (rollingCount < 0) {  //  Negative Rollover
   maxDivisor = floor(abs(rollingCount)/ppr);
   result = rollingCount + (ppr*maxDivisor);
 } else {
   result = rollingCount;
 }
 // #define debugCorrectRollover
 #ifdef debugcorrectRollover
 Serial.print("correctOverflow#");
 Serial.print(result);  
 Serial.print(" + in#");
 Serial.print(rollingCount);
 delay(100);
 #endif
 return result;
}

// Returns the distance required for the actuator to move
// clockwise to the destination
int Encoder::CWDistanceTo(unsigned int actuatorPos, unsigned int destPos, unsigned int zeroWindow) {
 int result;
 result = destPos - actuatorPos;
 if (abs(result) <= zeroWindow) {  //  Distance is negligible (On Top Of)
   result = 0;
 } else if (result < 0) { //  Distance must account for rollover
   result = ppr + result; // Rollover plus negative result
 }
 //#define debugCWDistanceTo
 #ifdef debugCWDistanceTo
 Serial.print(" CWDistanceTo#");
 Serial.print(result);  
 Serial.print(" + actuatorPos#");
 Serial.print(actuatorPos);
 Serial.print(" + DestPos#");
 Serial.print(destPos);
 delay(100);
 #endif
 return result;
}    

// Returns the current absolute position of any index position.
// Index 0 is the leading (positive-most) index number
// and starts at absolute position 0 when homed.
// ex. ppr=20 ic=2 then @home i0=0 & i1=-10 or i1=10 if modeModulo
int Encoder::getCurrentIndexPos(unsigned int index) { 
 int result = cPos - (index * ppi);
 if (result < 0) { result += ppr; }
 //#define debugslotAbsolutePosition
 #ifdef debugslotAbsolutePosition
 Serial.print(" slotAbsolutePosition#");
 Serial.print(result);  
 Serial.print(" + Index#");
 Serial.print(index);
 delay(100);
 #endif
 return result;
}

// Returns static index positions not accounting for current position
// (pulsePerIndex*index) 
int Encoder::getStaticIndexPos(unsigned int index) { 
 return ppi * index;
}

// Return current position count
int Encoder::getPosition() {
    return cPos;
}

unsigned int Encoder::getPulsePerRotation() {
    return ppr;
}

unsigned int Encoder::getPulsePerIndex() {
    return ppi;
}

unsigned int Encoder::getIndexCount() {
  return ic;
}

void Encoder::setPosition(int pos) {
  cPos = pos;
}