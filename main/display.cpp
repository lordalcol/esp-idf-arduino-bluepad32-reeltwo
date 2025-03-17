#include "display.h"
#include "state.h"
#include "pinout.h"
#include "utils.h"

static bool available = false;

// Create a display instance
static U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ PIN_DISPLAY_CLK, /* data=*/ PIN_DISPLAY_DATA, /* reset=*/ U8X8_PIN_NONE);

void setupDisplay() {
	u8x8.setBusClock(800000);
    u8x8.begin();
	u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
	int r = 0;
	u8x8.drawString(11, r++, "gp");
	u8x8.drawString(0, r++, "dt");
	u8x8.drawString(0, r++, "ax cx");
	u8x8.drawString(0, r++, "s1 xxx s2 xxx");
	available = true;
}

bool loopDisplay(int delay) {
	if(!available) return false;
	static u64_t lastOledUpdate = 0;
	if(timestamp_ms - lastOledUpdate > delay) {
		lastOledUpdate = timestamp_ms;
		static char b0[BUFFER_SIZE];
		int time = (int)(timestamp_ms / 1000);
		static int lastDisplayTime = 0;
		int r = 0;

		format_to_buffer(b0, BUFFER_SIZE, "%08d", time);
		u8x8.drawString(0, r, b0);
		format_to_buffer(b0, BUFFER_SIZE, "%01d", (int)gamepadCount);
		u8x8.drawString(14, r, b0);
		r++;
		format_to_buffer(b0, BUFFER_SIZE, "%03d-%03d", (int) deltatime_ms, (int)lastDisplayTime);
		u8x8.drawString(8, r, b0);
		r++;
		format_to_buffer(b0, BUFFER_SIZE, "%01d", features.sound);
		u8x8.drawString(1, r, b0);
		format_to_buffer(b0, BUFFER_SIZE, "%01d", features.CAN);
		u8x8.drawString(4, r, b0);
		r++;
		format_to_buffer(b0, BUFFER_SIZE, "%03d", (int)servo1value);
		u8x8.drawString(3, r, b0);
		format_to_buffer(b0, BUFFER_SIZE, "%03d", (int)servo2value);
		u8x8.drawString(10, r, b0);
		r++;

		lastDisplayTime = millis() - timestamp_ms;
		return true;
	}
	return false;
} 
