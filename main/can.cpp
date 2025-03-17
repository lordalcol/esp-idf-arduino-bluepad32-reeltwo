#include <Arduino.h>
#include <Bluepad32.h>
#include "pinout.h"
#include "state.h"
#include "../components/arduino-CAN/src/CAN.h"

void setupCAN() {
	Console.println("Starting CAN...");
	CAN.setPins(PIN_CAN_RX, PIN_CAN_TX);
	int ret = CAN.begin(500E3);
	if (!ret) {
		Console.printf("Starting CAN failed: %d\n", ret);
		delay(2000);
	}
	features.CAN = true;
}

void loopCAN() {
	if(!features.CAN) return;
	static u64_t lastUpdate = 0;
	if(timestamp_ms - lastUpdate > 1000) {
		lastUpdate = timestamp_ms;
		Console.printf("Sending CAN packet\n");
		CAN.setTimeout(10);
		CAN.beginPacket(0x12);
		CAN.write('h');
		CAN.write('e');
		CAN.write('l');
		CAN.write('l');
		CAN.write('o');
		CAN.endPacket();
	}

	if(false) {
		Console.printf("Sending ext CAN packet\n");
		CAN.beginExtendedPacket(0xabcdef);
		CAN.write('w');
		CAN.write('o');
		CAN.write('r');
		CAN.write('l');
		CAN.write('d');
		CAN.endPacket();
	}
}
