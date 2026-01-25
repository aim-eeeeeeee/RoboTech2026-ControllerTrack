#include <Arduino.h>
#include <Wire.h>
#include "wit_c_sdk.h"
#include "REG.h"

// Default I2C address for WT901CM
#define WIT_I2C_ADDRESS 0x50

static float g_roll = 0, g_pitch = 0, g_yaw = 0;

// Forward declarations
int32_t IICwriteBytes(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len);
int32_t IICreadBytes(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len);
void Delayms(uint16_t nms);
void CopeSensorData(uint32_t reg, uint32_t regNum);

void initGyro() {
    WitInit(WIT_PROTOCOL_I2C, WIT_I2C_ADDRESS);
    WitI2cFuncRegister(IICwriteBytes, IICreadBytes);
    WitRegisterCallBack(CopeSensorData);
    WitDelayMsRegister(Delayms);
}

void updateGyro() {
    // Read Roll, Pitch, Yaw
    WitReadReg(Roll, 3);
}

void getAngles(float &roll, float &pitch, float &yaw) {
    if (g_roll < 0) g_roll += 360.0f;
    roll = g_roll;
    pitch = g_pitch;
    yaw = g_yaw;
}

int32_t IICwriteBytes(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len) {
    Wire.beginTransmission(addr >> 1);
    Wire.write(reg);
    for (uint32_t i = 0; i < len; i++) {
        Wire.write(data[i]);
    }
    return (Wire.endTransmission() == 0) ? 1 : 0;
}

int32_t IICreadBytes(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len) {
    Wire.beginTransmission(addr >> 1);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0;
    Wire.requestFrom((uint8_t)(addr >> 1), (uint8_t)len);
    for (uint32_t i = 0; i < len && Wire.available(); i++) {
        data[i] = Wire.read();
    }
    return 1;
}

void Delayms(uint16_t nms) {
    delay(nms);
}

void CopeSensorData(uint32_t reg, uint32_t regNum) {
    if (reg == Roll && regNum >= 3) {
        g_roll  = sReg[Roll]  / 32768.0f * 180.0f;
        g_pitch = sReg[Pitch] / 32768.0f * 180.0f;
        g_yaw   = sReg[Yaw]   / 32768.0f * 180.0f;
    }
}
