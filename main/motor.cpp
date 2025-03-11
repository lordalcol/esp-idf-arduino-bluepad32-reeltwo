#include "motor.h"

void setMotor(int pwmVal, int pwm, int in1, int in2, int TOLERANCE_ZERO) {
    analogWrite(pwm, abs(pwmVal));
    if(pwmVal > TOLERANCE_ZERO) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
    else if(pwmVal < -TOLERANCE_ZERO) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
    else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
} 