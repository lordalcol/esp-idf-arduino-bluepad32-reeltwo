#include "servos.h"
#include "state.h"
#include "pinout.h"
#include <ESP32Servo.h>

static Servo myservo;  // create servo object to control a servo
static Servo myservo2;  // create servo object to control a servo

void setServo(Servo &servo, long val, int centerVal, int tolerance) {
	auto v2 = val - centerVal;
	if(v2 < tolerance && v2 > - tolerance) v2 = 0;
	v2 = v2 + centerVal;
	//if(printSerialLimited) Serial.printf(";Writing %ld [%d] to servo\n", v2, centerVal);
	servo.write(v2);
}

void setupServos() {
	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);// Standard 50hz servo
	myservo.attach(PIN_SERVO1, 500, 2400);   // attaches the servo on pin 18 to the servo object
	// using SG90 servo min/max of 500us and 2400us
	// for MG995 large servo, use 1000us and 2000us,
	// which are the defaults, so this line could be
	// "myservo.attach(servoPin);"
	myservo2.attach(PIN_SERVO2, 1000, 2000);
}

void processServos(ControllerPtr ctl) {
	servo1value = map(ctl->axisX(), -511, 512, 0, 180);
	setServo(myservo, servo1value, 90, 3);
	servo2value = map(ctl->throttle(), 0, 1023, 0, 90);
	setServo(myservo2, servo2value, 45, 2);
}
