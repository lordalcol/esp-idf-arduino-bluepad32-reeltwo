// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"

#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include "driver/pulse_cnt.h"
#include "../components/arduino-CAN/src/CAN.h"
#include "../components/Adafruit_Soundboard_library/Adafruit_Soundboard.h"

#define ENC_TICK_PER_NECK_ROTATION (19687 / 2)
#define ENC_TICK_LOW_LIMIT -ENC_TICK_PER_NECK_ROTATION / 2
#define ENC_TICK_HIGH_LIMIT ENC_TICK_PER_NECK_ROTATION / 2

bool soundAvailable = false;

#define PIN_CLK   0
#define PIN_SDD   0
#define PIN_SDI   0

#define PIN_AUDIO_RESET   15
#define PIN_CAN_TX   4
#define PIN_CAN_RX   16
#define PIN_SENSOR0_UNUSED 33
#define PIN_SENSOR1_UNUSED 32
#define PIN_SENSOR2_UNUSED 35
#define PIN_HALL_SENSOR 34
int motor1Pin1 = 11;
int motor1Pin2 = 10;
int motor1pwm = 9;
#define TXD2 17
#define RXD2 5
HardwareSerial dfSD(1);
// Possible PWM GPIO pins on the ESP32: 0(used by on-board button),2,4,5(used by on-board LED),12-19,21-23,25-27,32-33
// Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42
// Possible PWM GPIO pins on the ESP32-S3: 0(used by on-board button),1-21,35-45,47,48(used by on-board LED)
// Possible PWM GPIO pins on the ESP32-C3: 0(used by on-board button),1-7,8(used by on-board LED),9-10,18-21
#define PIN_SERVO1 18      // GPIO pin used to connect the servo control (digital out)
#define PIN_SERVO2 19      // GPIO pin used to connect the servo control (digital out)
// Possible ADC pins on the ESP32: 0,2,4,12-15,32-39; 34-39 are recommended for analog input
// Possible ADC pins on the ESP32-S2: 1-20 are recommended for analog input
#define EXAMPLE_EC11_GPIO_A 32
#define EXAMPLE_EC11_GPIO_B 33

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 4;
const int resolution = 8;
pcnt_unit_handle_t pcnt_unit = NULL;

void playNextSound();

//
// README FIRST, README FIRST, README FIRST
//
// Bluepad32 has a built-in interactive console.
// By default, it is enabled (hey, this is a great feature!).
// But it is incompatible with Arduino "Serial" class.
//
// Instead of using "Serial" you can use Bluepad32 "Console" class instead.
// It is somewhat similar to Serial but not exactly the same.
//
// Should you want to still use "Serial", you have to disable the Bluepad32's console
// from "sdkconfig.defaults" with:
//    CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=n

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
Servo myservo;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo

Adafruit_Soundboard sfx = Adafruit_Soundboard(&dfSD, NULL, PIN_AUDIO_RESET);

void setMotor(int pwmVal, int pwm, int in1, int in2, int TOLERANCE_ZERO){
	analogWrite(pwm,abs(pwmVal));
	if(pwmVal > TOLERANCE_ZERO){
		digitalWrite(in1,HIGH);
		digitalWrite(in2,LOW);
	}
	else if(pwmVal < -TOLERANCE_ZERO){
		digitalWrite(in1,LOW);
		digitalWrite(in2,HIGH);
	}
	else{
		digitalWrite(in1,LOW);
		digitalWrite(in2,LOW);
	}
}

int serialPrintLimiter = 0;
bool printSerialLimited = false;
u64_t lastTickTimestamp_ms = 0;
u64_t deltatime_ms = 0;

#define NECK_MAX_SPEED 245
#define NECK_ACC 1100
int neckSpeedCurrent = 0;
int lastSentNeckSpeed = 0;
int neckSpeedTarget = 0;
float neckPositionTarget = 0;
int neckPositionMaxSpeed = NECK_MAX_SPEED;
bool neckZeroFound = false;
long neckPositionOverflow = 0;

volatile bool hallSensor = false;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void updateNeckSpeed() {
	if (neckSpeedTarget != neckSpeedCurrent) {
		int accelerationStep = (NECK_ACC * deltatime_ms) / 1000; // Scale acceleration by delta time

		// Determine the direction of acceleration
		int speedDiff = neckSpeedTarget - neckSpeedCurrent;
		neckSpeedCurrent += min(abs(speedDiff), accelerationStep) * (speedDiff > 0 ? 1 : -1);
	}

	// Clamp neck speed within the allowed range
	neckSpeedCurrent = max(-NECK_MAX_SPEED, min(NECK_MAX_SPEED, neckSpeedCurrent));

	// Only send update if speed has changed
	if (neckSpeedCurrent != lastSentNeckSpeed) {
		Serial.printf("Neck speed target: %d current %d\n", neckSpeedTarget, neckSpeedCurrent);
		setMotor(neckSpeedCurrent, motor1pwm, motor1Pin1, motor1Pin2, 5);
		lastSentNeckSpeed = neckSpeedCurrent;
	}
}

void clearNeckPosition() {
	neckZeroFound = true;
	Serial.printf("clear pcnt unit\n");
	ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
	neckPositionOverflow = 0;
	neckPositionTarget = 0;
}

void zeroFound() {
	clearNeckPosition();
	neckSpeedTarget = 0;
	updateNeckSpeed();
	playNextSound();
}

void updateTime() {
	auto timestamp = millis();
	deltatime_ms = timestamp - lastTickTimestamp_ms;
	lastTickTimestamp_ms = timestamp;
	serialPrintLimiter++;
	printSerialLimited = (serialPrintLimiter % 10 == 0);
}

float shortestAngleDifference(float angle1, float angle2) {
	float delta = fmod((angle2 - angle1 + 180), 360) - 180;
	return (delta < -180) ? delta + 360 : delta;
}

void updateNeckPosition(float currentPos) {
	int e = shortestAngleDifference(neckPositionTarget, currentPos);
	int dir = e >= 0 ? 1 : -1;
	int delta = abs(e);
	if(delta <= 1) {
		neckSpeedTarget = 0;
	} else {
		neckSpeedTarget = dir * map(delta, 0, 180, 25, NECK_MAX_SPEED);
	}
	//if(printSerialLimited) Serial.printf("dir %d, delta %d, neckSpeedTarget %d\n", dir, delta, neckSpeedTarget);
}

void updateNeckPositionPID(float currentPos) {

	// PID constants
	float kp = 2;
	float kd = 0.025; //0.025;
	float ki = 0.0;

	// time difference
	long currT = micros();
	float deltaT = ((float) (currT - prevT))/( 1.0e6f );
	prevT = currT;

	// error
	int e = shortestAngleDifference(neckPositionTarget, currentPos);

	// derivative
	float dedt = (e-eprev)/(deltaT);

	// integral
	eintegral = eintegral + e*deltaT;

	// control signal
	float u = kp*e + kd*dedt + ki*eintegral;

	// motor power
	float pwr = fabs(u);
	if( pwr > neckPositionMaxSpeed ){
		pwr = neckPositionMaxSpeed;
	}
	if(pwr < 30) {
		pwr = 70;
	}
	Serial.printf("Pwr %.2f\n", pwr);

	// motor direction
	int dir = 1;
	if(u<0){
		dir = -1;
	}

	neckSpeedTarget = pwr * dir;

	// store previous error
	eprev = e;
}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Console.printf("CALLBACK: Controller is connected, index=%d\n", i);
			playNextSound();
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Console.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Console.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Console.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void setServo(Servo &servo, long val, int centerVal, int tolerance) {
	auto v2 = val - centerVal;
	if(v2 < tolerance && v2 > - tolerance) v2 = 0;
	v2 = v2 + centerVal;
	if(printSerialLimited) Serial.printf("Writing %ld [%d] to servo\n", v2, centerVal);
	servo.write(v2);
}

int sound = 1;

void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...

    //auto val = map(ctl->axisX(), -511, 512, 0, 180);
    auto val = map(ctl->axisX(), -511, 512, 0, 180);
	setServo(myservo, val, 90, 3);

    auto vel = map(ctl->throttle(), 0, 1023, 0, 90);
	setServo(myservo2, vel, 45, 2);

	if(neckZeroFound) {
		//neckSpeedTarget = map(ctl->axisRX(), -511, 512, -NECK_MAX_SPEED, NECK_MAX_SPEED);
		auto x = ctl->axisRX();
		auto y = ctl->axisRY();
		float mag = sqrt((float)(x * x + y * y));
		if(mag > 300) {
			neckPositionMaxSpeed = map(mag, 0, 720, 50, NECK_MAX_SPEED);
			neckPositionTarget = atan2(x, -y) * 180 / PI;
			Serial.printf("Angle: %.2f speed %d\n", neckPositionTarget, neckPositionMaxSpeed);
		}
	}
    if(ctl->buttons() & BUTTON_A) {
		playNextSound();
	}

    if(ctl->x()) {
        Console.printf("Sending CAN packet\n");
        CAN.beginPacket(0x12);
        CAN.write('h');
        CAN.write('e');
        CAN.write('l');
        CAN.write('l');
        CAN.write('o');
        CAN.endPacket();
    }

    if(ctl->y()) {
        Console.printf("Sending ext CAN packet\n");
        CAN.beginExtendedPacket(0xabcdef);
        CAN.write('w');
        CAN.write('o');
        CAN.write('r');
        CAN.write('l');
        CAN.write('d');
        CAN.endPacket();
    }

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    //dumpGamepad(ctl);
}

void playNextSound() {
	if(soundAvailable) {
//		uint8_t files = sfx.listFiles();
//
//		Serial.println("File Listing");
//		Serial.println("========================");
//		Serial.println();
//		Serial.print("Found "); Serial.print(files); Serial.println(" Files");
//		for (uint8_t f=0; f<files; f++) {
//			Serial.print(f);
//			Serial.print("\tname: "); Serial.print(sfx.fileName(f));
//			Serial.print("\tsize: "); Serial.println(sfx.fileSize(f));
//		}
//		Serial.println("========================");

		Console.printf("Playing sound %d\n", sound);
//		if (! sfx.playTrack((uint8_t)sound) ) {
//			Serial.println("Failed to play track?");
//		}
		sfx.playTrackAsync((uint8_t)sound);
		sound++;
	}
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            }
        }
    }
}

void IRAM_ATTR hallSensorRising() {
	hallSensor = true;
}

void setupSound() {
	Serial.begin(115200);
	dfSD.begin(9600, SERIAL_8N1, RXD2, TXD2);
	if (!sfx.reset()) {
		Serial.println("SFX Not found");
		delay(3000);
		return;
	}
	Serial.println("SFX board found");
	soundAvailable = true;

	uint16_t v = 0;
	while(v < 200) {
		if (!(v = sfx.volUp())) {
			Serial.println("Failed to adjust");
		} else {
			Serial.print("Volume: ");
			Serial.println(v);
		}
	}
}

static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
	BaseType_t high_task_wakeup;
	QueueHandle_t queue = (QueueHandle_t)user_ctx;

	// Increment overflow counter on overflow
	if (edata->watch_point_value == ENC_TICK_HIGH_LIMIT) {
		neckPositionOverflow += ENC_TICK_HIGH_LIMIT;
	}
	if (edata->watch_point_value == ENC_TICK_LOW_LIMIT) {
		neckPositionOverflow += ENC_TICK_LOW_LIMIT;
	}
//	if (edata->watch_point_value == 0) {
//		if (edata->zero_cross_mode == PCNT_UNIT_ZERO_CROSS_NEG_POS) {
//			neckPositionOverflow += 1;
//		} else if (edata->zero_cross_mode == PCNT_UNIT_ZERO_CROSS_POS_NEG) {
//			neckPositionOverflow -= 1;
//		} if (edata->zero_cross_mode == PCNT_UNIT_ZERO_CROSS_NEG_ZERO) {
//			neckPositionOverflow += 1;
//		} if (edata->zero_cross_mode == PCNT_UNIT_ZERO_CROSS_POS_ZERO) {
//			//neckPositionOverflow -= 1;
//		}
//	}

	// send event data to queue, from this interrupt callback
	xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
	return (high_task_wakeup == pdTRUE);
}
QueueHandle_t queue = xQueueCreate(10, sizeof(int));

void setupNeckEncoder() {
	pcnt_unit_config_t unit_config = {
			.low_limit = ENC_TICK_LOW_LIMIT,
			.high_limit = ENC_TICK_HIGH_LIMIT,
	};
	unit_config.flags.accum_count = 0;
	ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

	pcnt_glitch_filter_config_t filter_config = {
			.max_glitch_ns = 1000,
	};
	ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

	pcnt_chan_config_t chan_a_config = {
			.edge_gpio_num = EXAMPLE_EC11_GPIO_A,
			.level_gpio_num = EXAMPLE_EC11_GPIO_B,
	};
	pcnt_channel_handle_t pcnt_chan_a = NULL;
	ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
	pcnt_chan_config_t chan_b_config = {
			.edge_gpio_num = EXAMPLE_EC11_GPIO_B,
			.level_gpio_num = EXAMPLE_EC11_GPIO_A,
	};
	pcnt_channel_handle_t pcnt_chan_b = NULL;
	ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
	ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
	ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
	ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, unit_config.high_limit));
	ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, unit_config.low_limit));
	//ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, 0));
	pcnt_event_callbacks_t cbs = {
			.on_reach = example_pcnt_on_reach,
	};
	ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

	ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
	ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
	ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

	neckSpeedTarget = 100;
	updateNeckSpeed();
}

void setupBluetooth() {
	const uint8_t* addr = BP32.localBdAddress();
	Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

	// Setup the Bluepad32 callbacks, and the default behavior for scanning or not.
	// By default, if the "startScanning" parameter is not passed, it will do the "start scanning".
	// Notice that "Start scanning" will try to auto-connect to devices that are compatible with Bluepad32.
	// E.g: if a Gamepad, keyboard or mouse are detected, it will try to auto connect to them.
	bool startScanning = true;
	BP32.setup(&onConnectedController, &onDisconnectedController, startScanning);

	// Notice that scanning can be stopped / started at any time by calling:
	// BP32.enableNewBluetoothConnections(enabled);

	// "forgetBluetoothKeys()" should be called when the user performs
	// a "device factory reset", or similar.
	// Calling "forgetBluetoothKeys" in setup() just as an example.
	// Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
	// But it might also fix some connection / re-connection issues.
	BP32.forgetBluetoothKeys();

	// Enables mouse / touchpad support for gamepads that support them.
	// When enabled, controllers like DualSense and DualShock4 generate two connected devices:
	// - First one: the gamepad
	// - Second one, which is a "virtual device", is a mouse.
	// By default, it is disabled.
	BP32.enableVirtualDevice(false);

	// Enables the BLE Service in Bluepad32.
	// This service allows clients, like a mobile app, to setup and see the state of Bluepad32.
	// By default, it is disabled.
	BP32.enableBLEService(false);
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

void setupCAN() {
	CAN.setPins (PIN_CAN_RX, PIN_CAN_TX);
	if (!CAN.begin(100E3)) {
		Console.println("Starting CAN failed!");
		delay(2000);
	}
}

void setupNeckZeroSensor() {
	pinMode(PIN_HALL_SENSOR, INPUT);
#if USE_HALL_INTERRUPT
	attachInterrupt(digitalPinToInterrupt(PIN_HALL_SENSOR), hallSensorRising, CHANGE);
#endif
}

void setupNeckMotorDrive() {
	// sets the pins as outputs:
	pinMode(motor1Pin1, OUTPUT);
	pinMode(motor1Pin2, OUTPUT);
	pinMode(motor1pwm, OUTPUT);
	// configure LEDC PWM
	ledcAttachChannel(motor1pwm, freq, resolution, pwmChannel);
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
	setupBluetooth();
	setupServos();
//	setupSound();
//	setupCAN();
//	setupNeckZeroSensor();
//	setupNeckMotorDrive();
//	setupNeckEncoder();
	updateTime();
}

// Arduino loop function. Runs in CPU 1.
void loop() {
	updateTime();
	// This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);

//	// Checks state of player, if playing enable amp
//	// We now do this in hardware
//    auto state = myDFPlayer.readState();
//    if(state == 1) {
//        pinMode(21, INPUT);
//    } else {
//        pinMode(21, OUTPUT);
//        digitalWrite(21, LOW);
//    }

//	updateNeckSpeed();
//
//	if(!neckZeroFound) {
//		if(hallSensor) {
//			hallSensor = false;
//			detachInterrupt(digitalPinToInterrupt(PIN_HALL_SENSOR));
//			zeroFound();
//		}
//	}
//
//	//if(printSerialLimited) {
//		int pulse_count = 0;
//		ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
//		auto tot = neckPositionOverflow + pulse_count;
//		auto rot = tot / ENC_TICK_PER_NECK_ROTATION;
//		auto deg = 360.0f * (float) (tot % ENC_TICK_PER_NECK_ROTATION) / (float) (ENC_TICK_PER_NECK_ROTATION);
//		//if(printSerialLimited) Serial.printf("Pulse count: ticks:%d overflowed:%ld total:%ld rot: %ld deg: %.2f (target %.2f)\n", pulse_count, neckPositionOverflow, neckPositionOverflow + pulse_count, rot, deg, neckPositionTarget);
//		//}
//	if(neckZeroFound) {
//		updateNeckPosition(deg);
//	}
	delay(15);
}
