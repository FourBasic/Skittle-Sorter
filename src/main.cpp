/* #region INCLUDE */
#include <Arduino.h>
#include <AccelStepper.h>
#include <AFMotor.h>
#include "GeneralFunctions.h"
#include "Encoder.h"
#include "ColorSensor.h"
#include "Debounce.h"
#include "TimerOnDelay.h"
/* #endregion */

/* #region DEFINE */
#define PIN_SCANNER_SCALE_SO 7
#define PIN_SCANNER_SCALE_S1 8
#define PIN_SCANNER_FILTER_S2 9
#define PIN_SCANNER_FILTER_S3 10
#define PIN_SCANNER_OUT 2

#define PIN_HOME_SWITCH A2

#define SCANNER_FREQ_600kHz 2
#define SCANNER_FREQ_120kHz 1
#define SCANNER_FREQ_12kHz 0

#define STEP_ID_DROP 1
#define STEP_ID_COLLECT 2

#define STS_MOVE_TO_WAIT 0
#define STS_MOVE_TO_SCAN 1
#define STS_MOVE_TO_DROP 2
#define STS_MOVE_TO_DROPSCAN 3
/* #endregion */

/* #region STRUCT */
struct SlotData
{
   int num;
   int dist;
};
/* #endregion */

/* #region HARDWARE CONFIG */
const float totalSteps = 400;                // Number of gSteps per full collector/dropper revolution
const int totalQaud = 8;                     // Number of drop qaudrants
const int totalCollectorSlot = 4;            // Number of slots/holes in collector
const int scanQuad = 2;                      // Scanner quadrant number
const int window = 2;                        // +- window where drop/collectors are considered 'in position'. Deals with float calculations with decimals
const unsigned int rgbMatchTol = 14; //10        // +- tolerance when comparing one RGB value to another
const int dropQuadOrder[] = {3, 4, 5, 6, 7}; // Set available drop position quadrants. Newest color will be placed in lowest available element. Qaud 0 is hopper
unsigned int bgColor[] = {139, 159, 101};     // Baseline background color of empty slot {127, 145, 91}
unsigned int bgTol; //= 8; (AUTOSET)           // +- tolerance when comparing RGB value with background color
const unsigned int colorFiltDelay = 25;//10     // Switching delay for color sensor filters (msec) 
/* #endregion */

/* #region CALCULATED */
const float gStepPerQuad = totalSteps / totalQaud;

/* #endregion */

/* #region CONTRUCTOR */
AF_Stepper stepCollect(200, STEP_ID_COLLECT);
AF_Stepper stepDrop(200, STEP_ID_DROP);
void forwardStep1() { stepCollect.onestep(BACKWARD, SINGLE); }
void backwardStep1() { stepCollect.onestep(FORWARD, SINGLE); }
void forwardStep2() { stepDrop.onestep(FORWARD, SINGLE); }
void backwardStep2() { stepDrop.onestep(BACKWARD, SINGLE); }
AccelStepper collect(forwardStep1, backwardStep1); // Pins used 3,4,5,6,7,8,11,12
AccelStepper drop(forwardStep2, backwardStep2);
Encoder encCollector(totalSteps, totalQaud, true);
Encoder encDropper(totalSteps, totalQaud, true);
ColorSensor colorScanner(13,A1,9,10,2,12);
Debounce homeSwitch(1);
/* #endregion */

/* #region GLOBAL */
int gDropPosBuff[totalCollectorSlot]; //  Element is collector slot number  //  Data is Drop Position // 0 = nothing to drop
int gStep;
long gWait;
unsigned int gDropQuadColor[8][3]; // Quadrant x RGB
int gStsCollector, gStsDropper;
int gDebugStep;
int gSkittleCount, gBackgroundCount, gBackgroundCountConsec;
SlotData nextScanSlot;
SlotData nextFullSlot;
/* #endregion */

/* #region FUNCTION */
// #define debugSetStepperSpeed
void setStepperSpeed(float speed, float accel)
{
   collect.setMaxSpeed(speed);
   collect.setAcceleration(accel);
   drop.setMaxSpeed(speed);
   drop.setAcceleration(accel);
}

// returns the appropriate drop quadrant number
// for the current color scanResult
int findDropQuad()
{
#define IS_EMPTY 1
#define IS_MATCH 2
#define IS_LAST 3
   int sts;
   int result;
   for (auto checkQuad : dropQuadOrder)
   {                      //  Check Quadrants in drop order for match or empty
      result = checkQuad; // Load Current Check Quad // Function will break on some kind of match leaving the last loaded as the appropriate drop quad
      if (gDropQuadColor[checkQuad][0] == 0 && gDropQuadColor[checkQuad][1] == 0 && gDropQuadColor[checkQuad][2] == 0)
      { //  Drop quadrant is empty. All elements after this will be empty too.
         gDropQuadColor[checkQuad][0] = colorScanner.scanResult[0];
         gDropQuadColor[checkQuad][1] = colorScanner.scanResult[1];
         gDropQuadColor[checkQuad][2] = colorScanner.scanResult[2];
         sts = IS_EMPTY;
         break;
      }
      else
      {
         unsigned int unpackDropQuadColor[] = {gDropQuadColor[checkQuad][0], gDropQuadColor[checkQuad][1], gDropQuadColor[checkQuad][2]};
         if (colorScanner.match(colorScanner.scanResult, unpackDropQuadColor, rgbMatchTol))
         {
            sts = IS_MATCH;
            break;
         }
      }
   }
   if (sts == 0)
   {
      sts = IS_LAST;
   } //  No match or empty therefore must be last -- Must Drop
// #define debugFindDropQuad
#ifdef debugFindDropQuad
  Serial.print(gDebugStep);
  Serial.print(" FindDropQuad#");
  Serial.print(result);
  if (sts == IS_EMPTY)
  {
     Serial.print("Empty");
  }
  else if (sts == IS_MATCH)
  {
     Serial.print("Match");
  }
  else if (sts == IS_LAST)
  {
     Serial.print("Last");
  }
  else
  {
     Serial.print("Unknown");
  }
  Serial.print(" + dropQuadOrder#");
  for (auto checkQuad : dropQuadOrder)
  {
     Serial.print(checkQuad);
     Serial.print("/");
  }
  Serial.print("END ");
  Serial.println(gDebugStep);
  gDebugStep++;
  delay(500);
#endif
   return result; //  Quad is either: EMPTY, MATCHING or the LAST available quad
}

// Returns the slot data of the nearest clockwise slot to the given position
// slot.num = -1 if nothing found
SlotData findNextSlot(int pos, bool full)
{
   SlotData nextSlot;
   //unsigned int ppi;
   nextSlot.num = -1;
   nextSlot.dist = 9999;
   //ppi = encCollector.getPulsePerIndex();
   for (int i = 0; i < totalCollectorSlot; i++)
   {
      if ((!full && gDropPosBuff[i] == 0) || (full && gDropPosBuff[i] != 0))
      {
         //  Calculate Absolute gStep position of slot accounting for rollover
         int slotAbsolutePos = encCollector.getCurrentIndexPos(i * 2);
         //  Calculate clockwise step distance of current slot to scanner position
         //int dist = encCollector.CWDistanceTo(slotAbsolutePos, ppi * scanQuad, window); 0
         int dist = encCollector.CWDistanceTo(slotAbsolutePos, pos, window);
         // Check against last closest found
         if (dist < nextSlot.dist && dist != 0)
         {
            nextSlot.num = i;
            nextSlot.dist = dist;
         }
      }
   }
   return nextSlot;
}

// Sets current color as background
//#define debugSetBackground
void setBackground() {
   bgTol = 0;
   colorScanner.sampleLowest[0] = 9999;
   colorScanner.sampleLowest[1] = 9999;
   colorScanner.sampleLowest[2] = 9999;
   colorScanner.sampleHighest[0] = 0;
   colorScanner.sampleHighest[1] = 0;
   colorScanner.sampleHighest[2] = 0;
   // Populate sampleLowest/Highest
   for (int i=0; i<10; i++) {
      delay(700);
      colorScanner.scan(colorFiltDelay);
      colorScanner.sampleRange();
   }
   // Use sampleLowest/Highest to determine nominal background color and tolerance
   for (int i=0; i<3; i++) {
      #ifdef debugSetBackground
      Serial.print("Range ");
      Serial.print(i);
      Serial.print(" From ");
      Serial.print(colorScanner.sampleLowest[i]);
      Serial.print(" To ");
      Serial.println(colorScanner.sampleHighest[i]);
      #endif
      bgColor[i] = (colorScanner.sampleHighest[i] + colorScanner.sampleLowest[i]) / 2;
      unsigned int calc;
      calc = (colorScanner.sampleHighest[i] - colorScanner.sampleLowest[i]) / 2;
      if (calc > bgTol) {
         bgTol = calc;
      }
   }
   bgTol += 7; // 3
   #ifdef debugSetBackground
   Serial.print(bgColor[0]);
   Serial.print("-");
   Serial.print(bgColor[1]);
   Serial.print("-");
   Serial.print(bgColor[2]);
   Serial.print("----");
   Serial.println(bgTol);
   
   Serial.print(colorScanner.scanResult[0]);
   Serial.print("-");
   Serial.print(colorScanner.scanResult[1]);
   Serial.print("-");
   Serial.println(colorScanner.scanResult[2]);
   Serial.println("");

   #endif
}
/* #endregion */

void setup()
{
   Serial.begin(9600);  
   pinMode(PIN_HOME_SWITCH, INPUT);
   setStepperSpeed(650.0, 1500.0); // 650 4000$8000 // 650 500 slow //26kwtf 950/36k some skips // solid 750,16k
   colorScanner.scan(colorFiltDelay);
   gStep = 5;
   delay(3000);
}

void loop()
{
   homeSwitch.update(digitalRead(PIN_HOME_SWITCH), 50, 50);
   
   if (!collect.isRunning() && !drop.isRunning())
   {
      switch (gStep)
      {
      case 1: // Test Step
         /* #region TEST */
         //colorScanner.scan(colorFiltDelay);
         //colorScanner.sampleRange();
         setBackground(); 
         /*
         for (int i = 0; i < 3; i++)
         {
            Serial.print(colorScanner.scanResult[i]);
            Serial.print(",");
         }
         Serial.println("");
         */
         
         delay(0);
         /* #endregion */
         break;
      case 5: //  Homing - Tandem jog CCW until home switch not pressed
         /* #region HOME */
         delay(100);
         if (homeSwitch.getState())
         {
            collect.move(-1);
            drop.move(-1);          
         }
         else
         {
            gStep = 7;
         }                 
         break;
      case 7: //  Homing - Jog Collector CW to home switch then back off CCW one index
         delay(100);
         if (!homeSwitch.getState())
         {
            collect.move(1);
         }
         else
         {            
            //collect.move(-encCollector.getPulsePerIndex());
            collect.move(-21);
            gStep = 9;
         } 
         break;
      case 9: //  Homing - Jog Dropper CCW to home switch
         delay(100);
         if (!homeSwitch.getState())
         {
            drop.move(-1);
         }
         else
         {
            collect.setCurrentPosition(encDropper.getStaticIndexPos(scanQuad)-4);
            drop.setCurrentPosition(encDropper.getStaticIndexPos(scanQuad+2)-4);         
            gStep = 10;
         }
         break;
      case 10: // Move to Starting Pos
         collect.moveTo(0);
         drop.moveTo(encDropper.getStaticIndexPos(scanQuad+1));
         gStep = 11;
         break;
      case 11: // Set Background color constant
         delay(500);
         setBackground();
         gStep = 20;
         break;
         /* #endregion */
      case 20: //  Determine next position
         /* #region UPDATE POSITION */
         encCollector.update(collect.currentPosition());
         encDropper.update(drop.currentPosition());
         collect.setCurrentPosition(encCollector.getPosition());
         drop.setCurrentPosition(encDropper.getPosition());
         /* #endregion */

         /* #region NEXT SCAN SLOT */
         // Search for next collector slot to arrive at the scanner
         nextScanSlot = findNextSlot(encCollector.getStaticIndexPos(scanQuad), false);
         /* #endregion */

         /* #region SCAN COLOR */
         // Color scan if a slot is in the scanner position
         #define scanEnable
         #ifdef scanEnable
            if (gStsCollector == STS_MOVE_TO_SCAN || gStsCollector == STS_MOVE_TO_DROPSCAN)
            {
               colorScanner.scan(colorFiltDelay); //  Sets scanResult
               int currentScanSlot;
               currentScanSlot = nextScanSlot.num - 1;
               if (currentScanSlot < 0)
               {
                  currentScanSlot = totalCollectorSlot - 1;
               }
               //Serial.print("Scan Slot - ");
               //Serial.print(currentScanSlot);
               if (!colorScanner.match(colorScanner.scanResult, bgColor, bgTol))
               {                                               // scanResult is not background
                  int dq = findDropQuad();                     // find an appropriate drop quadrant for the current slot and scanner
                  gDropPosBuff[currentScanSlot] = encCollector.getStaticIndexPos(dq); // convert quad to absolute position and insert in buffer
                  gSkittleCount++;
                  gBackgroundCountConsec = 0;
                  /*Serial.print(" - MATCH ");
                  Serial.print(dq);
                  Serial.print(" - ");
                  Serial.print(colorScanner.scanResult[0]);
                  Serial.print(" - ");
                  Serial.print(colorScanner.scanResult[1]);
                  Serial.print(" - ");
                  Serial.println(colorScanner.scanResult[2]);
                  */
               }
               else
               {
                  //Serial.println("BG");
                  gBackgroundCountConsec++;
                  gBackgroundCount++;
               }
            }
         #endif
         /* #endregion */

         /* #region NEXT FULL SLOT */
         // Search for the next FULL collector slot to arrive at droppers current position
         nextFullSlot = findNextSlot(encDropper.getPosition(), true);
         if (nextFullSlot.num != -1) // No full slot found
         {
            // Use the full slot# closest to the dropper to find
            // the distance to that slot's drop position
            int nextFullSlotAbsolutePos;
            nextFullSlotAbsolutePos = encCollector.getCurrentIndexPos(nextFullSlot.num * 2);
            // Negative Result is backwards and scan slot is irrelevant. Positive needs to be compared.
            nextFullSlot.dist = gDropPosBuff[nextFullSlot.num] - nextFullSlotAbsolutePos;
         }
         /* #endregion */

         /* #region MOVEMENT COMMANDS */
         #ifdef debugSlotData
         Serial.print("nextScanSlot/");
         Serial.print(nextScanSlot.num);
         Serial.print(" NextScanSlotDist/");
         Serial.print(nextScanSlot.dist);
         Serial.print(" NextFullSlot/");
         Serial.print(nextFullSlot.num);
         Serial.print(" NextFullSlotDist/");
         Serial.print(nextFullSlot.dist);
         Serial.print(" dropBuffer/");
         Serial.print(gDropPosBuff[0]);
         Serial.print(",");
         Serial.print(gDropPosBuff[1]);
         Serial.print(",");/*  */
         Serial.print(gDropPosBuff[2]);
         Serial.print(",");
         Serial.println(gDropPosBuff[3]);
         #endif
         if (nextFullSlot.num != -1) // No full slot found
         {
            if (nextFullSlot.dist <= nextScanSlot.dist) //  Collector is closest to drop - Move both to drop
            {
               collect.moveTo(encCollector.getPosition() + nextFullSlot.dist);
               drop.moveTo(gDropPosBuff[nextFullSlot.num]);
               if (nextFullSlot.dist < nextScanSlot.dist)
               {
                  gStsCollector = STS_MOVE_TO_DROP;
                  //Serial.print("Both To Drop");
               }
               else
               {
                  gStsCollector = STS_MOVE_TO_DROPSCAN;
                  //Serial.print("Dropscan");
               }
               gStsDropper = STS_MOVE_TO_DROP;
               gDropPosBuff[nextFullSlot.num] = 0;               
            }
            else //  Collector is closest to scan - Move collector to scan & dropper to drop
            { 
               collect.moveTo(encCollector.getPosition() + nextScanSlot.dist);
               drop.moveTo(gDropPosBuff[nextFullSlot.num]);
               gStsCollector = STS_MOVE_TO_SCAN;
               gStsDropper = STS_MOVE_TO_DROP;
               //Serial.println("Dropper To Drop");
            }
         }
         else //  No drop in queue - Move collector to scan & dropper to Wait position
         { 
            collect.moveTo(encCollector.getPosition() + nextScanSlot.dist);
            drop.moveTo(encCollector.getStaticIndexPos(scanQuad + 1));
            gStsCollector = STS_MOVE_TO_SCAN;
            gStsDropper = STS_MOVE_TO_WAIT;
            //Serial.println("Dropper To Wait");
         }
         /* #endregion */
         
         /* #region STOP MONITOR */
         if (gBackgroundCountConsec < 15)
         {
            gWait++;
         }
         else
         {
            Serial.print("Total Count ");
            Serial.print(gSkittleCount);
            Serial.print(" Missed Slots ");
            Serial.println(gBackgroundCount - gBackgroundCountConsec);
            delay(2000);
            // Collect power-down position. Cover hopper hole. 
            //collect.moveTo(encCollector.getPulsePerIndex());
            collect.moveTo(0);
            drop.moveTo(215);
            // Drop power-down position. Can't concisely describe the reason for this position, but there is a reason.
            //drop.moveTo(encCollector.getStaticIndexPos(encCollector.getIndexCount()-3));
            gStep = 99;
         }
         /* #endregion */
         break;
      case 99:
         /* #region De-energize */
         collect.stop();
         drop.stop();
         stepCollect.release();
         stepDrop.release();
         digitalWrite(11, LOW);
         digitalWrite(3, LOW);
         digitalWrite(5, LOW);
         digitalWrite(6, LOW);
         digitalWrite(7, LOW);
         /* #endregion */
         break;
      default:
         // Serial.println("bork");
         break;
      }
   }
   collect.run();
   drop.run();
}
