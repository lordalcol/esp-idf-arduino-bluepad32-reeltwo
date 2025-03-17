#include "touch.h"
#include "state.h"
#include "pinout.h"

static bool available = false;

void setupTouch() {
	// https://docs.espressif.com/projects/arduino-esp32/en/latest/api/touch.html
	touchSetCycles( 0x5000, 0x5000 );
	available = true;
}

void loopTouch() {
	if(!available) return;
	touch7 = touchRead(PIN_HEAD_TOUCH);
}
