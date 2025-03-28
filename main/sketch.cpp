#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>

#include "display.h"
#include "neck.h"
#include "sound.h"
#include "servos.h"
#include "touch.h"
#include "can.h"
#include "pinout.h"

#define USE_DEBUG
#include "../components/Reeltwo/src/ReelTwo.h"
//#define USE_VERBOSE_SERVO_DEBUG
#define USE_SERVO_DEBUG
#include "../components/Reeltwo/src/ServoDispatchDirect.h"
#include "../components/Reeltwo/src/ServoSequencer.h"
#include "../components/Reeltwo/src/core/Animation.h"
#include "../components/Reeltwo/src/core/Marcduino.h"

int32_t strtol(const char *cmd, const char **endptr)
{
    bool sign = false;
    int32_t result = 0;
    if (*cmd == '-')
    {
        cmd++;
        sign = true;
    }
    while (isdigit(*cmd))
    {
        result = result * 10L + (*cmd - '0');
        cmd++;
    }
    *endptr = cmd;
    return (sign) ? -result : result;
}

bool numberparams(const char *cmd, uint8_t &argcount, int32_t *args, uint8_t maxcount)
{
    for (argcount = 0; argcount < maxcount; argcount++)
    {
        args[argcount] = strtol(cmd, &cmd);
        if (*cmd == '\0')
        {
            argcount++;
            return true;
        }
        else if (*cmd != ',')
        {
            return false;
        }
        cmd++;
    }
    return true;
}

static bool enableCAN = false;
static bool enableBluetooth = true;
static bool enableTouch = true;
static bool enableNeck = true;
static bool enableServos = false;
static bool enableSound = true;
static bool enableDisplay = true;

const ServoSettings servoSettings[] PROGMEM = {
        { PIN_SERVO2, 500, 2400, 0x1000 },
};

ServoDispatchDirect<SizeOfArray(servoSettings)> servoDispatch(servoSettings);
ServoSequencer servoSequencer(servoDispatch);
AnimationPlayer player(servoSequencer);

////////////////

MARCDUINO_ACTION(CloseAllPanels, :CL01, ({
    SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllClose, 0x1000);
}))

////////////////

MARCDUINO_ACTION(OpenAllPanels, :OP01, ({
    SEQUENCE_PLAY_ONCE(servoSequencer, SeqPanelAllOpen, 0x1000);
}))

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
    if(ctl->buttons() & BUTTON_B) {
        Marcduino::processCommand(player, ":CL01");
	}
    if(ctl->buttons() & BUTTON_Y) {
        Marcduino::processCommand(player, ":OP01");
	}
    if(ctl->buttons() & BUTTON_X) {
        Marcduino::processCommand(player, ":XX99");
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
	 // BP32.forgetBluetoothKeys();

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

void loopBluetooth() {
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
}

// Arduino setup function. Runs in CPU 1
void setup() {
    REELTWO_READY();
    SetupEvent::ready();
	//Serial.begin(115200);
	Console.printf("Firmware: %s\n", BP32.firmwareVersion());
	if(enableDisplay) setupDisplay();
	loopDisplay(0);
	if(enableCAN) setupCAN();
	if(enableBluetooth) setupBluetooth();
	if(enableTouch) setupTouch();
	if(enableNeck) setupNeck();
	if(enableServos) setupServos();
	if(enableSound) setupSound();
	loopDisplay(0);
	updateTime();
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    AnimatedEvent::process();
	updateTime();
	loopCAN();
	loopBluetooth();
	loopTouch();
	loopNeck();
	printStateToSerial(1000);
	if(!loopDisplay(900)) {
		delay(1);
	}
}
