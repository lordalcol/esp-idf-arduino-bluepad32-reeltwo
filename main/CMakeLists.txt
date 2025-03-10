set(srcs
        "main.c"
        "sketch.cpp")

set(requires "bluepad32" "bluepad32_arduino" "arduino" "btstack")

idf_component_register(SRCS "${srcs}"
                    PRIV_REQUIRES esp_driver_pcnt esp_driver_gpio
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}" "ESP32Servo" "arduino-CAN" "Adafruit_Soundboard_library" "esp8266-oled-ssd1306" "U8g2_Arduino")
