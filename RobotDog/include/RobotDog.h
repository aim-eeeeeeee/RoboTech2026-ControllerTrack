#ifndef ROBOT_DOG_H
#define ROBOT_DOG_H

#include <Arduino.h>
#include <Servo.h>
#include "ServoController.h"
#include "GyroReader.h"
#include "ServoConfig.h" 

class RobotDog {
public:
    RobotDog();
    void begin();
    
    /**
     * @brief Main Control Interface
     * @param force_stand  true: Force robot to stand pose (ignore walking phase); false: Allow walking
     * @param direction    1: Forward, -1: Backward, 0: Stop phase update
     * @param h_lf         Left Front Height:  1(High), 0(Normal), -1(Low)
     * @param h_lb         Left Back Height:   1(High), 0(Normal), -1(Low)
     * @param h_rf         Right Front Height: 1(High), 0(Normal), -1(Low)
     * @param h_rb         Right Back Height:  1(High), 0(Normal), -1(Low)
    */

    void setControl(bool force_stand, int direction, int h_lf, int h_lb, int h_rf, int h_rb);
    void setSpeed(float frequency);
    void run();
    void LiftLeftFront();
    void Roll();

private:
    ServoController servoCtrl;
    float _gait_freq;      
    float _phase;          
    unsigned long _last_time;
    
    // PID State
    float _prev_pitch_err;
    float _prev_roll_err;
    float _offset_pitch;
    float _offset_roll;

    // Control Inputs
    bool _force_stand;
    int _dir;
    int _trim_lf;
    int _trim_lb;
    int _trim_rf;
    int _trim_rb;

    float _filter_pitch_err = 0.0;
    float _filter_roll_err = 0.0;

    // Internal methods
    void updateGait(float dt);
    void getInterpolatedAngles(float p, float &thigh, float &calf);
    void getInterpolatedStep(float idx, float &t1, float &c1, float &t2, float &c2);
    void calculatePID(float &p_out, float &r_out);
    float getTableAngle(int idx);
    float getMirroredAngle(float input_angle);
};

#endif