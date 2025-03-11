#ifndef PINOUT_H
#define PINOUT_H

#define PIN(name, value, ...) \
    constexpr int name = value; \
    __VA_OPT__(constexpr int __VA_ARGS__ = value;)

// https://www.upesy.com/blogs/tutorials/esp32-pinout-reference-gpio-pins-ultimate-guide#

PIN(PIN_GPIO06_SPICLK,   6)
PIN(PIN_GPIO07_SPID,     7)
PIN(PIN_GPIO08_SPID,     8)
PIN(PIN_GPIO15,          15, PIN_AUDIO_RESET)
PIN(PIN_GPIO02,          2)  // LED
PIN(PIN_GPIO00,          0)  // External 5k pullout?
PIN(PIN_GPIO04,          4,  PIN_CAN_TX)
PIN(PIN_GPIO16_U2_RX,    16, PIN_AUDIO_RX)
PIN(PIN_GPIO17_U2_TX,    17, PIN_AUDIO_TX)
PIN(PIN_GPIO05,          5,  PIN_CAN_RX)
PIN(PIN_GPIO18,          18, PIN_SERVO1)
PIN(PIN_GPIO19,          19, PIN_SERVO2)
PIN(PIN_GND1,            -1)
PIN(PIN_GPIO21,          21, PIN_DISPLAY_DATA)
PIN(PIN_GPIO03_U0RX,     3)  // USB
PIN(PIN_GPIO01_U0TX,     1)  // USB
PIN(PIN_GPIO22,          22, PIN_DISPLAY_CLK)
PIN(PIN_GPIO23,          23)
PIN(PIN_GND0,            -1)

PIN(PIN_5v,              -1)
PIN(PIN_GPIO11_CMD,      -1)
PIN(PIN_GPIO10_U1TXD,    -1)
PIN(PIN_GPIO09_U1RXD,    -1)
PIN(PIN_GPIO13,          13, PIN_MOTOR1_IN1)
PIN(PIN_GND2,            -1)
PIN(PIN_GPIO12,          12, PIN_MOTOR1_IN2)
PIN(PIN_GPIO14,          14, PIN_MOTOR1_PWM)
PIN(PIN_GPIO27,          27)
PIN(PIN_GPIO26,          26)
PIN(PIN_GPIO25,          25)
PIN(PIN_GPIO33,          33, PIN_ENCODER_B)
PIN(PIN_GPIO32,          32, PIN_ENCODER_A)
PIN(PIN_GPIO35_IN,       35)
PIN(PIN_GPIO34_IN,       34, PIN_NECK_ZERO_SENSOR)
PIN(PIN_SYN,             -1)
PIN(PIN_SYP,             -1)
PIN(PIN_EN,              -1)
PIN(PIN_3V3,             -1)

#endif // PINOUT_H 
