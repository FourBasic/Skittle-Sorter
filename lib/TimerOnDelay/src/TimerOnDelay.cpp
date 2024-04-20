#include "Arduino.h"
#include "TimerOnDelay.h"

TimerOnDelay::TimerOnDelay() {
}

//	Determines if PT has elapsed
bool TimerOnDelay::done() {
	unsigned long ct;
	ct = millis();
	if (ct >= doneTime) {
		if (!overflow) { return true; }
		else if (ct < startTime) { return true; }
		else { return false; } 
	} else {
		return false;
	}
	return false;
}

// 	Returns true after run is true for PT time(msec)
bool TimerOnDelay::update(bool run, unsigned int PT) {	
	if (run) {
		if (!running) {	startTime = millis(); }
		doneTime = startTime + PT;
		overflow = doneTime < startTime;
		dn = done();
		tt = !dn;
	} else { 
		dn = false;
		tt = false;
	}
	running = run;
	return dn;
}

//	Timer is timing but not done
bool TimerOnDelay::getTimerTiming() {
	return tt;
}

//	Timer is done
bool TimerOnDelay::getTimerDone() {
	return dn;
}

