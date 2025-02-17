// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"

#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <DFRobotDFPlayerMini.h>
#include <CAN.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>

const char *ssid = "NETGEAR69";
const char *password = "newbreeze553";

#define TX_GPIO_NUM   5
#define RX_GPIO_NUM   4
#define LED_BUILTIN   2
#define HALL_SENSOR 22

int motor1Pin1 = 27;
int motor1Pin2 = 26;
int enable1Pin = 14;
// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 4;
const int resolution = 8;

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

void updateTime();

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
Servo myservo;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo

HardwareSerial dfSD(1);
DFRobotDFPlayerMini myDFPlayer;

// Possible PWM GPIO pins on the ESP32: 0(used by on-board button),2,4,5(used by on-board LED),12-19,21-23,25-27,32-33
// Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42
// Possible PWM GPIO pins on the ESP32-S3: 0(used by on-board button),1-21,35-45,47,48(used by on-board LED)
// Possible PWM GPIO pins on the ESP32-C3: 0(used by on-board button),1-7,8(used by on-board LED),9-10,18-21
int servoPin = 18;      // GPIO pin used to connect the servo control (digital out)
int servoPin2 = 19;      // GPIO pin used to connect the servo control (digital out)
// Possible ADC pins on the ESP32: 0,2,4,12-15,32-39; 34-39 are recommended for analog input
// Possible ADC pins on the ESP32-S2: 1-20 are recommended for analog input

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
#define NECK_ACC 1400
int neckSpeedCurrent = 0;
int lastSentNeckSpeed = 0;
int neckSpeedTarget = 0;

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
		if (printSerialLimited) Serial.printf("Neck speed target: %d current %d\n", neckSpeedTarget, neckSpeedCurrent);
		setMotor(neckSpeedCurrent, enable1Pin, motor1Pin1, motor1Pin2, 10);
		lastSentNeckSpeed = neckSpeedCurrent;
	}
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
	Serial.printf("Writing %d to servo\n", v2);
	servo.write(v2 + centerVal);
}

int sound = 0;

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
//    if (ctl->a()) {
//		digitalWrite(LED_BUILTIN, LOW);
//	} else {
//		digitalWrite(LED_BUILTIN, HIGH);
//	}

    //auto val = map(ctl->axisX(), -511, 512, 0, 180);
    auto val = map(ctl->axisX(), -511, 512, 0, 180);
	setServo(myservo, val, 90, 3);

    auto vel = map(ctl->throttle(), 0, 1023, 0, 90);
	setServo(myservo2, vel, 45, 2);

	neckSpeedTarget = map(ctl->axisRX(), -511, 512, -NECK_MAX_SPEED, NECK_MAX_SPEED);

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
	Console.printf("Playing sound %d\n", sound);
	pinMode(21, INPUT);
	myDFPlayer.volume(30);  // Set volume value. From 0 to 30
	myDFPlayer.play(sound++);     // Play the first mp3
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
#define RXD2 16
#define TXD2 17

#define USE_HALL_INTERRUPT 0

volatile bool hallSensor = false;

int hallState = 0;

void IRAM_ATTR hallSensorRising() {
	hallSensor = true;
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

//	WiFi.mode(WIFI_STA);
//	WiFi.begin(ssid, password);
//	while (WiFi.waitForConnectResult() != WL_CONNECTED) {
//		Serial.println("Connection Failed! Rebooting...");
//		delay(5000);
//		ESP.restart();
//	}
	Serial.printf("Connected to %s\n", ssid);

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

    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    myservo.setPeriodHertz(50);// Standard 50hz servo
    myservo.attach(servoPin, 500, 2400);   // attaches the servo on pin 18 to the servo object
                                          // using SG90 servo min/max of 500us and 2400us
                                          // for MG995 large servo, use 1000us and 2000us,
                                          // which are the defaults, so this line could be
                                          // "myservo.attach(servoPin);"
    myservo2.attach(servoPin2, 1000, 2000);

    pinMode(21, OUTPUT);
    digitalWrite(21, LOW);

    Serial.begin(115200);
    dfSD.begin(9600, SERIAL_8N1, RXD2, TXD2);
    Console.println(F("DFRobot DFPlayer Mini Demo"));
    Console.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

    if (!myDFPlayer.begin(dfSD, true, true)) {
        Console.println(F("Unable to begin:"));
        Console.println(F("1.Please recheck the connection!"));
        Console.println(F("2.Please insert the SD card!"));
    } else {
        Console.println(F("DFPlayer Mini online."));
        delay(15);
		playNextSound();
    }

    CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);
    if (!CAN.begin(100E3)) {
        Console.println("Starting CAN failed!");
        delay(2000);
    }

	pinMode(LED_BUILTIN, OUTPUT);

	pinMode(HALL_SENSOR, INPUT);
#if USE_HALL_INTERRUPT
	attachInterrupt(digitalPinToInterrupt(HALL_SENSOR), hallSensorRising, RISING);
#endif

	// sets the pins as outputs:
	pinMode(motor1Pin1, OUTPUT);
	pinMode(motor1Pin2, OUTPUT);
	pinMode(enable1Pin, OUTPUT);
	// configure LEDC PWM
	ledcAttachChannel(enable1Pin, freq, resolution, pwmChannel);

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
    auto state = myDFPlayer.readState();
    if(state == 1) {
        pinMode(21, INPUT);
    } else {
        pinMode(21, OUTPUT);
        digitalWrite(21, LOW);
    }

	updateNeckSpeed();

#if USE_HALL_INTERRUPT
	if(hallSensor) {
		hallSensor = false;
		Serial.println("Hall sensor RISING");
		myservo.write(180);
		playNextSound();
	} else {
		myservo.write(0);
	}
#else
	auto newHallState = digitalRead(HALL_SENSOR);
	if(newHallState != hallState) {
		Serial.printf("Hall %d", newHallState);
		if(newHallState == 0) {
			myservo.write(180);
			playNextSound();
		} else {
			myservo.write(0);
		}
	}
	hallState = newHallState;
#endif
	delay(15);
}

void updateTime() {
	auto timestamp = millis();
	deltatime_ms = timestamp - lastTickTimestamp_ms;
	lastTickTimestamp_ms = timestamp;
	serialPrintLimiter++;
	printSerialLimited = (serialPrintLimiter % 10 == 0);
}
