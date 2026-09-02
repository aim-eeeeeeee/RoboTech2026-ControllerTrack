#include "ServoConfig.h"

int servoChannel[NUM_SERVOS] = {
  0,  // Channel 0 for left_front_bottom
  1,  // Channel 1 for left_front_top
  2,  // Channel 2 for left_back_bottom
  3,  // Channel 3 for left_back_top
  8,   // Channel 8 for right_front_bottom
  9,   // Channel 9 for right_front_top
  10,   // Channel 10 for right_back_bottom
  11    // Channel 11 for right_back_top
};

double servoOffset[NUM_SERVOS] = {
  15,    // left_front_bottom
  5,  // left_front_top
  15,   // left_back_bottom
  10,   // left_back_top
  3,   // right_front_bottom
  -5,    // right_front_top
  3,  // right_back_bottom
  8    // right_back_top
};

int servoAngleLimit[NUM_SERVOS][2] = {
  {140, 220},    // left_front_bottom
  {150, 210},    // left_front_top
  {140, 220},    // left_back_bottom
  {150, 210},    // left_back_top
  {140, 220},    // right_front_bottom
  {150, 210},    // right_front_top
  {140, 220},    // right_back_bottom
  {150, 210}     // right_back_top
};

double servoThighAngles_1[NUM_ANGLES/2] = {
  164.99, 167.58, 170.03, 172.33, 174.47, 176.46, 178.28, 179.93, 181.4, 182.65,
  183.69, 184.46, 184.97, 185.16, 185.03, 184.58, 183.83, 182.78, 181.48, 179.95
};

double servoThighAngles_2[NUM_ANGLES/2] = {
  179.95, 179.19, 178.42, 177.64, 176.85, 176.05, 175.25, 174.44, 173.62, 172.79,
  171.95, 171.1, 170.24, 169.39, 168.52, 167.65, 166.77, 165.89, 165.0, 164.09
};

double servoCalfAngles_1[NUM_ANGLES/2] = {
  166.18, 163.9, 161.84, 160.04, 158.56, 157.43, 156.7, 156.39, 156.52, 157.07,
  158.06, 159.44, 161.18, 163.23, 165.56, 168.12, 170.88, 173.77, 176.79, 179.9
};

double servoCalfAngles_2[NUM_ANGLES/2] = {
  179.89, 179.15, 178.41, 177.68, 176.97, 176.26, 175.55, 174.88, 174.2, 173.54,
  172.89, 172.26, 171.63, 171.02, 170.42, 169.84, 169.26, 168.71, 168.17, 167.64
};

/*
Right Leg (Servo 4=knee, 5=hip):
Knee (Servo 4):
  90° → 110° (+20) = 抬起/压缩（腿弯曲）
  90° → 70° (-20)  = 伸长（腿伸直）

Hip (Servo 5):
  90° → 110° (+20) = 向下摆（身体升高）
  90° → 70° (-20)  = 向上抬（压缩）

Left Leg (Servo 0=knee, 1=hip):
Knee (Servo 0):
  90° → 110° (+20) = 伸长（腿伸直）
  90° → 70° (-20)  = 抬起/压缩（腿弯曲）

Hip (Servo 1):
  90° → 110° (+20) = 向上抬（压缩）
  90° → 70° (-20)  = 向下摆（身体升高）
*/