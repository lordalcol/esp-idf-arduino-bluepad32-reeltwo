#ifndef SERVOS_H
#define SERVOS_H
#include <Arduino.h>
#include <Bluepad32.h>

void setupServos();
void processServos(ControllerPtr ctl);

#endif //SERVOS_H
