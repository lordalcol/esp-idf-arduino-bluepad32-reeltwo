#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "../components/U8g2_Arduino/src/U8x8lib.h"
#include "state.h"

void setupDisplay();
bool loopDisplay();

#endif // DISPLAY_H 
