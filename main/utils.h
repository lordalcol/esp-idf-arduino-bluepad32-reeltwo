#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <stdarg.h>

float normalizeDegrees(float angle);
void format_to_buffer(char* buf, size_t size, const char* format, ...);

#endif // UTILS_H 
