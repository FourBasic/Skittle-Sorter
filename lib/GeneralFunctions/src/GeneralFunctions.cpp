#include "GeneralFunctions.h"
#include <Arduino.h>
// Returns true of 'a' in range of 'b' +- tolerance
bool withinRange(int a, int b, int tol) {
 return abs(a-b) < tol;
}



