#include "servos.h"
#include "state.h"
#include "pinout.h"
#include <ESP32Servo.h>

static Servo myservo1;
static Servo myservo2;
static Servo myservo3;
static Servo myservo4;

static bool available = false;

void setServo(Servo &servo, long val, int centerVal, int tolerance) {
	auto v2 = val - centerVal;
	if(v2 < tolerance && v2 > - tolerance) v2 = 0;
	v2 = v2 + centerVal;
	//if(printSerialLimited) Serial.printf(";Writing %ld [%d] to servo\n", v2, centerVal);
	servo.write(v2);
}

long mapServo(Servo &servo, long in, long in_min, long in_max, long out_min, long out_max, int tolerance) {
	auto value = map(in, in_min, in_max, out_min, out_max);
	setServo(servo, value, (out_max - out_min) / 2, 2);
	return value;
}

void standby() {
	myservo1.detach();
	digitalWrite(PIN_SERVO1, LOW);
	myservo2.detach();
	digitalWrite(PIN_SERVO2, LOW);
	myservo3.detach();
	digitalWrite(PIN_SERVO3, LOW);
	myservo4.detach();
	digitalWrite(PIN_SERVO4, LOW);
}

void resume() {
	// using SG90 servo min/max of 500us and 2400us
	// for MG995 large servo, use 1000us and 2000us,
	// which are the defaults, so this line could be
	// "myservo.attach(servoPin);"
	myservo1.setPeriodHertz(50);// Standard 50hz servo
	myservo1.attach(PIN_SERVO1, 500, 2400);
	myservo2.attach(PIN_SERVO2, 1000, 2000);
	myservo3.attach(PIN_SERVO3, 1000, 2000);
	myservo4.attach(PIN_SERVO4, 1000, 2000);
}

void setupServos() {
	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	resume();
	available = true;
}

void processServos(ControllerPtr ctl) {
	if(!available) return;
	if(myservo1.attached()) {
		servo1value = mapServo(myservo1, ctl->axisX(), -511, 512, 0, 180, 3);
		servo2value = mapServo(myservo2, ctl->throttle(), 0, 1023, 0, 90, 2);
		mapServo(myservo3, ctl->axisX(), -511, 512, 0, 180, 3);
		mapServo(myservo4, ctl->axisX(), -511, 512, 0, 180, 3);
	}
}
