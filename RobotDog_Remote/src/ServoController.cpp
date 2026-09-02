#include "ServoController.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h> 
#include <string.h> 

ServoController::ServoController(uint8_t addr) : pwm(addr) {}

void ServoController::begin() {
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
    delay(10);
    Serial.println("ServoController initialized");
}

uint16_t ServoController::usToCounts(uint16_t us) const {
    return (uint16_t)((uint32_t)us * 4096UL / 20000UL);
}

uint16_t ServoController::angleToUs(float angle_deg, uint8_t index) const {
    if (index >= NUM_SERVOS) return 1500;

    float a = angle_deg;
    a = constrain(a, servoAngleLimit[index][0], servoAngleLimit[index][1]);
    a += servoOffset[index];

    a = fmodf(a, 360.0f);
    if (a < 0.0f) a += 360.0f;

    float t = a / 360.0f;
    float us_f = (float)SERVO_MIN_US + t * (float)(SERVO_MAX_US - SERVO_MIN_US);

    int us = (int)lroundf(us_f);
    us = constrain(us, (int)SERVO_MIN_US, (int)SERVO_MAX_US);
    return (uint16_t)us;
}

void ServoController::setAngleImmediate(uint8_t index, float angle, bool print_details) {
    if (index >= NUM_SERVOS) return;

    uint8_t ch = servoChannel[index];
    uint16_t us = angleToUs(angle, index);

    if (print_details) {
        Serial.print("idx="); Serial.print(index);
        Serial.print(" ch="); Serial.print(ch);
        Serial.print(" angle="); Serial.print(angle);
        Serial.print(" -> "); Serial.print(us);
        Serial.println("us");
    }

    uint16_t counts = usToCounts(us);
    pwm.setPWM(ch, 0, counts);}

void ServoController::standStill(bool print_details) {
    if (print_details) {
        Serial.println("StandStill -> set all to 172deg mapping");
    }

    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        setAngleImmediate(i, 172.0f, print_details);
    }

    delay(800);
    if (print_details) {
        Serial.println("StandStill done");
    }
}

void ServoController::handleSerialOffsetTuning() {
    static char line[64];
    static size_t len = 0;

    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r') continue;

        if (c != '\n') {
            if (len < sizeof(line) - 1) {
                line[len++] = c;
                line[len] = '\0';
            } else {
                Serial.println("[OffsetTune] Input too long, cleared.");
                len = 0;
                line[0] = '\0';
            }
            continue;
        }

        if (len == 0) return;

        removeSpacesInPlace(line);

        int opPos = -1;
        char op = 0;
        for (size_t i = 1; i < len; ++i) {
            if (line[i] == '+' || line[i] == '-') {
                opPos = (int)i;
                op = line[i];
                break;
            }
        }

        if (opPos <= 0 || opPos >= (int)len - 1) {
            Serial.print("[OffsetTune] Invalid cmd: \"");
            Serial.print(line);
            Serial.println("\". Use: <num>+<delta> or <num>-<delta> (e.g., 3+5, 7-0.25).");
            len = 0; line[0] = '\0';
            return;
        }

        line[opPos] = '\0';
        const char* numStr = line;
        const char* deltaStr = line + opPos + 1;

        char* endNum = nullptr;
        long numL = strtol(numStr, &endNum, 10);
        if (endNum == numStr || *endNum != '\0') {
            Serial.println("[OffsetTune] Invalid servo index.");
            len = 0; line[0] = '\0';
            return;
        }

        if (numL < 0 || numL >= NUM_SERVOS) {
            Serial.print("[OffsetTune] servo index out of range: ");
            Serial.println(numL);
            len = 0; line[0] = '\0';
            return;
        }

        char* endDelta = nullptr;
        double delta = strtod(deltaStr, &endDelta);
        if (endDelta == deltaStr || *endDelta != '\0') {
            Serial.println("[OffsetTune] Invalid delta.");
            len = 0; line[0] = '\0';
            return;
        }

        int num = (int)numL;
        if (op == '+') servoOffset[num] += delta;
        else           servoOffset[num] -= delta;

        Serial.print("[OffsetTune] servoIndex=");
        Serial.print(num);
        Serial.print(" (pcaChannel=");
        Serial.print(servoChannel[num]);
        Serial.print(") newOffset=");
        Serial.println(servoOffset[num], 6);

        standStill();

        len = 0;
        line[0] = '\0';
        return;
    }
}
