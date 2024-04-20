#ifndef TIMERONDELAY_H
#define TIMERONDELAY_H
#include "Arduino.h"
class TimerOnDelay {
  public:		
		TimerOnDelay();
		bool update(bool run, unsigned int PT);
		bool getTimerTiming();
		bool getTimerDone();
  private:
		bool done();
		unsigned long startTime;
		unsigned long doneTime;
		bool running;
		bool dn;
		bool tt;
		bool overflow;
};
#endif