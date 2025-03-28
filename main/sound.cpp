#include "sound.h"
#include "pinout.h"
#include "state.h"
#define MAXFILES 128
#include "../components/Adafruit_Soundboard_library/Adafruit_Soundboard.h"

HardwareSerial dfSD(1);
Adafruit_Soundboard sfx = Adafruit_Soundboard(&dfSD, NULL, PIN_AUDIO_RESET);

void setupSound() {
    dfSD.begin(9600, SERIAL_8N1, PIN_AUDIO_RX, PIN_AUDIO_TX);
    if (!sfx.reset()) {
        Serial.println("SFX Not found");
        //delay(3000);
        return;
    }
    Serial.println("SFX board found");

    uint16_t v = 0;
    while(v < 200) {
        if (!(v = sfx.volUp())) {
            Serial.println("Failed to adjust");
			return;
        } else {
            Serial.print("Volume: ");
            Serial.println(v);
        }
    }
	features.sound = true;
}

void playNextSound() {
    if(!features.sound) return;
	sfx.playTrackAsync((uint8_t)sound);
	sound++;
}
