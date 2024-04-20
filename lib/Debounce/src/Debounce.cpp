#include "Debounce.h"
#include "TimerOnDelay.h"

Debounce::Debounce(bool defaultState) {
	out = defaultState;
}
//	Returns state of filtered output
bool Debounce::update(bool in, unsigned int onT, unsigned int offT) {
	raw = in;
	if (timerOn.update(raw, onT)) { out = true; }
	else if (timerOff.update(!raw, offT)) { out = false; }
	return out;
}

bool Debounce::getRaw() {
	return raw;
}

bool Debounce::getState() {
	return out;
}
