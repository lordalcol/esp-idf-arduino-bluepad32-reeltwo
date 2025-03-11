#include "sdkconfig.h"
#include "display.h"
#include "pinout.h"

#include <Arduino.h>
#include <Bluepad32.h>
#include "../components/arduino-CAN/src/CAN.h"
#include "../components/Adafruit_Soundboard_library/Adafruit_Soundboard.h"
#include "neck.h"
#include "sound.h"
#include "servos.h"

static bool enableDisplay = true;
static bool enableBluetooth = true;
static bool enableServos = true;
static bool enableNeck = true;
static bool enableSound = false;
static bool enableCAN = false;

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
			gamepadCount++;
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
			gamepadCount--;
            break;
        }
    }

    if (!foundController) {
        Console.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

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

    processServos(ctl);

	if(neckZeroFound) {
		//neckSpeedTarget = map(ctl->axisRX(), -511, 512, -NECK_MAX_SPEED, NECK_MAX_SPEED);
		auto x = ctl->axisRX();
		auto y = ctl->axisRY();
		updateNeckPositionFromXY(x, y);
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

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            }
        }
    }
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

void setupCAN() {
	CAN.setPins(PIN_CAN_RX, PIN_CAN_TX);
	if (!CAN.begin(100E3)) {
		Console.println("Starting CAN failed!");
		delay(2000);
	}
}

// Arduino setup function. Runs in CPU 1
void setup() {
	Serial.begin(115200);
	Console.printf("Firmware: %s\n", BP32.firmwareVersion());
	if(enableDisplay) setupDisplay();
	if(enableBluetooth) setupBluetooth();
	if(enableServos) setupServos();
	if(enableNeck) setupNeck();
	if(enableSound) setupSound();
	if(enableCAN) setupCAN();
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

	loopNeck();
	printStateToSerial();
	if(!loopDisplay()) {
		delay(15);
	}
}
