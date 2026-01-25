#ifndef _GYRO_READER_H_
#define _GYRO_READER_H_

#include <Arduino.h>

void initGyro();
void updateGyro();
void getAngles(float &roll, float &pitch, float &yaw);

#endif
