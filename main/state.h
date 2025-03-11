#ifndef STATE_H
#define STATE_H

#include <Arduino.h>

extern int gamepadCount;

// Display buffer size
constexpr const size_t BUFFER_SIZE = 200;

// Display state
extern float neckPositionCurrent;
extern float neckPositionTarget;
extern int neckSpeedCurrent;
extern int neckSpeedTarget;
extern bool neckZeroFound;
extern bool hallSensor;
extern int sound;

// Motor state
extern int lastSentNeckSpeed;
extern int neckPositionMaxSpeed;  // NECK_MAX_SPEED
extern long neckPositionOverflow;
extern long prevT;
extern float eprev;
extern float eintegral;

// Servo state
extern long servo1value;
extern long servo2value;

// Timing state
extern int serialPrintLimiter;
extern u64_t timestamp_ms;
extern u64_t lastTickTimestamp_ms;
extern int deltatime_ms;

// Sound state
extern bool soundAvailable;

// Display buffer state
extern bool printBufferUpdated;
extern char buffer[BUFFER_SIZE];

// Function declarations
void updateTime();
void printStateToSerial();

#endif // STATE_H 
