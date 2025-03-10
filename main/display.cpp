#include "display.h"

// Create a display instance
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 22, /* data=*/ 21, /* reset=*/ U8X8_PIN_NONE);

void setupDisplay() {
    u8x8.begin();
    u8x8.setFont(u8x8_font_chroma48medium8_r);
}

void loopDisplay() {
    if(printOled) {
        u8x8.clearLine(0);
        u8x8.setCursor(0,0);
        u8x8.print("Neck:");
        u8x8.print(neckPositionCurrent);
        u8x8.print(" Target:");
        u8x8.print(neckPositionTarget);
        u8x8.print(" Speed:");
        u8x8.print(neckSpeedCurrent);
        u8x8.print(" Target:");
        u8x8.print(neckSpeedTarget);
        u8x8.print(" Zero:");
        u8x8.print(neckZeroFound);
        u8x8.print(" Hall:");
        u8x8.print(hallSensor);
        u8x8.print(" Sound:");
        u8x8.print(sound);
        printOled = false;
    }
} 