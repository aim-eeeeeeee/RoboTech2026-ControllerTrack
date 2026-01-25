#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "ServoConfig.h"

// ============================================================
// Servo Controller Class
// ------------------------------------------------------------
// Manages 8 servos via PCA9685 with smooth interpolation
// ============================================================

#define SERVO_FREQ 50        // Standard servo frequency (Hz)
#define SERVO_MIN_US 500     // Minimum pulse width (microseconds)
#define SERVO_MAX_US 2500    // Maximum pulse width (microseconds)

class ServoController {
public:
    explicit ServoController(uint8_t addr = 0x40);
    void begin();
    void handleSerialOffsetTuning();

    void setAngleImmediate(uint8_t index, float angle, bool print_details = false);

    void standStill(bool print_details = false);


private:
    Adafruit_PWMServoDriver pwm;
    uint16_t angleToUs(float angle_deg, uint8_t index) const;

    static void removeSpacesInPlace(char* s) {
        char* w = s;
        for (char* r = s; *r; ++r) {
            if (!isspace((unsigned char)*r)) {
                *w++ = *r;
            }
        }
        *w = '\0';
    }

    // us -> PCA9685 counts (0..4095)
    uint16_t usToCounts(uint16_t us) const;
};

#endif