#include "utils.h"
#include <cmath>

float normalizeDegrees(float angle) {
	angle = std::fmod(angle, 360.0f);  // Keep within -360 to 360
	if (angle < 0) {
		angle += 360.0f;  // Ensure it's positive
	}
	return angle;
}

void format_to_buffer(char* buf, size_t size, const char* format, ...) {
    va_list args;
    va_start(args, format);
    vsnprintf(buf, size, format, args);
    va_end(args);
}
