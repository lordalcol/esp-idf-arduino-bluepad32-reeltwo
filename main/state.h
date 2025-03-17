#ifndef STATE_H
#define STATE_H

#include <Arduino.h>

extern int gamepadCount;
constexpr const size_t BUFFER_SIZE = 200;
extern float neckPositionCurrent;
extern float neckPositionTarget;
extern int neckSpeedCurrent;
extern int neckSpeedTarget;
extern bool neckZeroFound;
extern bool hallSensor;
extern int sound;
extern int lastSentNeckSpeed;
extern int neckPositionMaxSpeed;  // NECK_MAX_SPEED
extern long neckPositionOverflow;
extern long prevT;
extern float eprev;
extern float eintegral;
extern long servo1value;
extern long servo2value;
extern int serialPrintLimiter;
extern u64_t timestamp_ms;
extern u64_t lastTickTimestamp_ms;
extern int deltatime_ms;
extern int touch7;
extern bool printBufferUpdated;
extern char buffer[BUFFER_SIZE];

typedef struct
{
	bool sound;
	bool CAN;
} Features;

extern Features features;

void updateTime();
void printStateToSerial(int delay);

#endif // STATE_H 
