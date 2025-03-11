#include "state.h"
#include "utils.h"

int gamepadCount = 0;

// Display state
float neckPositionCurrent = 0;
float neckPositionTarget = 0;
int neckSpeedCurrent = 0;
int neckSpeedTarget = 0;
bool neckZeroFound = false;
bool hallSensor = false;
int sound = 1;

// Motor state
int lastSentNeckSpeed = 0;
int neckPositionMaxSpeed = 245;  // NECK_MAX_SPEED
long neckPositionOverflow = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

long servo1value = 0;
long servo2value = 0;

// Timing state
int serialPrintLimiter = 0;
u64_t timestamp_ms = 0;
u64_t lastTickTimestamp_ms = 0;
int deltatime_ms = 0;

void updateTime() {
	timestamp_ms = millis();
	deltatime_ms = std::min(9999, (int)(timestamp_ms - lastTickTimestamp_ms));
	lastTickTimestamp_ms = timestamp_ms;
	serialPrintLimiter++;
} 

void printStateToSerial() {
	Serial.print('$');
	Serial.print(deltatime_ms); Serial.print(',');
	Serial.print(neckZeroFound); Serial.print(',');
	Serial.print(normalizeDegrees(neckPositionCurrent)); Serial.print(',');
	Serial.print(normalizeDegrees(neckPositionTarget)); Serial.print(',');
	Serial.print(servo1value); Serial.print(',');
	Serial.print(servo2value); Serial.print(',');
	Serial.print('\n');
}
