#include "state.h"
#include "pinout.h"
#include "utils.h"

int gamepadCount = 0;
float neckPositionCurrent = 0;
float neckPositionTarget = 0;
int neckSpeedCurrent = 0;
int neckSpeedTarget = 0;
bool neckZeroFound = false;
bool hallSensor = false;
int sound = 1;
int touch7 = 0;
int lastSentNeckSpeed = 0;
int neckPositionMaxSpeed = 245;
long neckPositionOverflow = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
long servo1value = 0;
long servo2value = 0;
int serialPrintLimiter = 0;
u64_t timestamp_ms = 0;
u64_t lastTickTimestamp_ms = 0;
int deltatime_ms = 0;
Features features = {false, false};

void updateTime() {
	timestamp_ms = millis();
	deltatime_ms = std::min(9999, (int)(timestamp_ms - lastTickTimestamp_ms));
	lastTickTimestamp_ms = timestamp_ms;
	serialPrintLimiter++;
} 

void printStateToSerial(int delay) {
	static auto lastTimestamp = millis();
	if(delay > 0) {
		auto now = millis();
		if(now - lastTimestamp < delay) {
			return;
		}
		lastTimestamp = now;
	}
	Serial.print('$');
	Serial.print(timestamp_ms); Serial.print(',');
	Serial.print(deltatime_ms); Serial.print(',');
	Serial.print(neckZeroFound); Serial.print(',');
	Serial.print(normalizeDegrees(neckPositionCurrent)); Serial.print(',');
	Serial.print(normalizeDegrees(neckPositionTarget)); Serial.print(',');
	Serial.print(servo1value); Serial.print(',');
	Serial.print(servo2value); Serial.print(',');
	Serial.print(touch7); Serial.print(',');
	Serial.print('\n');
}
