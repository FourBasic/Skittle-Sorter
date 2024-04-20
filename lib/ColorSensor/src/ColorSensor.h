#ifndef COLORSENSOR_H
#define COLORSENSOR_H
#include <Arduino.h>
#include "GeneralFunctions.h"

class ColorSensor {
  public:		
		ColorSensor(uint8_t S0, uint8_t S1, uint8_t S2, uint8_t S3, uint8_t out, unsigned int speed);
		void scan(unsigned int filterSwitchDelay);
		bool match(unsigned int c1[], unsigned int c2[], unsigned int tol);
		void sampleRange();
		unsigned int scanResult[3];
		unsigned int sampleLowest[3];
		unsigned int sampleHighest[3];
  private:
		uint8_t Pin_S0, Pin_S1, Pin_S2, Pin_S3, Pin_Out;		
		
};
#endif
