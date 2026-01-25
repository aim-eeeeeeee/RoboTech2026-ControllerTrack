

#include "RobotDog.h"


// ================= TUNING PARAMETERS =================
const float Kp_pitch = 0;
const float Kd_pitch = 0;
const float Kp_roll  = 0;
const float Kd_roll  = 0;
const float MAX_PID  = 10.0; 

const float GAIN = 1; 

const int STAND_IDX = 10;      

const float HEIGHT_TRIM_DEG = 5.0;

const float R_Target = 180.0;
const float P_Target = 0;

const int STEP_FRAMES = 20;

RobotDog::RobotDog() : servoCtrl(0x40) {
    _gait_freq = 1; 
    _phase = 0.0;
    _last_time = 0;
    
    // Default: Stand still, no movement, normal height
    _force_stand = true; 
    _dir = 0; 
    _trim_lf = 0; _trim_lb = 0; _trim_rf = 0; _trim_rb = 0;
    
    _offset_pitch = 0; 
    _offset_roll = 0;
}

void RobotDog::begin() {
    Wire.begin();
    Wire.setClock(400000);
    servoCtrl.begin();
    
    Serial.println("Init: Standing up...");
    servoCtrl.standStill(); 
    delay(2000); 
    initGyro();
}

void RobotDog::setControl(bool force_stand, int direction, int h_lf, int h_lb, int h_rf, int h_rb) {
    _force_stand = force_stand;
    _dir = direction;
    _trim_lf = h_lf;
    _trim_lb = h_lb;
    _trim_rf = h_rf;
    _trim_rb = h_rb;
}

void RobotDog::setSpeed(float frequency) {
    _gait_freq = frequency;
}

void RobotDog::run() {
    unsigned long now = millis();

    /*
    // ===== Print IMU Data =====
    static unsigned long last_print = 0;
    if (now - last_print > 20) {
        last_print = now;
        
        float r, p, y;
        getAngles(r, p, y);
        
        Serial.print("Gyro [R, P, Y]: ");
        Serial.print(r); Serial.print(", ");
        Serial.print(p); Serial.print(", ");
        Serial.println(y);
    }
    */


    if (now - _last_time < 10) return; // 100Hz
    float dt = (now - _last_time) / 1000.0f;
    _last_time = now;
    updateGait(dt);
}

// Helper: Get angle from servoThighAngles_2
float RobotDog::getTableAngle(int idx) {
    if (idx < 0) idx = 0;
    if (idx > 19) idx = 19;
    return servoThighAngles_2[idx];
}
/*
void RobotDog::updateGait(float dt) {
    // We define two diagonal groups for the Trot gait:
    // Group A: Left Front (LF) & Right Back (RB) -> Move together
    // Group B: Right Front (RF) & Left Back (LB) -> Move together (180 degree phase shift)
    
    float hip_A, knee_A; // Base angles for Group A
    float hip_B, knee_B; // Base angles for Group B

    // 1. Gait Generation (Trajectory Calculation)
    if (_force_stand || _dir == 0) {
        // --- STAND STILL ---
        // Load the standard standing angles
        float val_h = getTableAngle(STAND_IDX); 
        
        // Assuming calf uses the same index or a fixed value for standing
        float val_k = 172.0;
        
        hip_A = val_h; knee_A = val_k;
        hip_B = val_h; knee_B = val_k;

    } else {
        // --- WALKING (TROT) ---
        // Update phase based on frequency and time step
        float phase_change = _gait_freq * dt;

        if (_dir == 1) {
            _phase -= phase_change; // Move Forward
        } else {
            _phase += phase_change; // Move Backward
        }
        
        // Wrap phase between 0.0 and 1.0
        if (_phase >= 1.0f) _phase -= 1.0f;
        if (_phase < 0.0f)  _phase += 1.0f;

        // Group A (LF + RB) follows the current phase
        getInterpolatedAngles(_phase, hip_A, knee_A);

        // Group B (RF + LB) follows phase + 0.5 (Opposite timing)
        float phase_B = _phase + 0.5f;
        if (phase_B >= 1.0f) phase_B -= 1.0f;
        getInterpolatedAngles(phase_B, hip_B, knee_B);
    }

    // 2. PID Stabilization (Balance)
    float pid_pitch, pid_roll;
    calculatePID(pid_pitch, pid_roll);

    // Convert height trim settings to degrees
    float h_lf = _trim_lf * HEIGHT_TRIM_DEG;
    float h_lb = _trim_lb * HEIGHT_TRIM_DEG;
    float h_rf = _trim_rf * HEIGHT_TRIM_DEG;
    float h_rb = _trim_rb * HEIGHT_TRIM_DEG;

    // Mixing PID and Trim:
    // Pitch positive: Nose goes up -> Front legs extend (+), Back legs retract (-)
    // Roll positive: Right side goes up -> Right legs extend (+), Left legs retract (-)
    float ext_LF =  pid_pitch - pid_roll + h_lf; 
    float ext_LB = -pid_pitch - pid_roll + h_lb;
    float ext_RF =  pid_pitch + pid_roll + h_rf;
    float ext_RB = -pid_pitch + pid_roll + h_rb;

    // 3. Motor Mixing & Inversion
    // Note: 'GAIN' determines if positive 'ext' makes the leg longer or shorter.
    // If the robot moves down when it should go up, flip the sign of GAIN.
    
    // --- LEFT SIDE (Standard) ---
    // LF is in Group A
    float out_LF_Hip  = hip_A - (ext_LF * GAIN); 
    float out_LF_Knee = knee_A + (ext_LF * GAIN);

    // LB is in Group B
    float out_LB_Hip  = hip_B - (ext_LB * GAIN);
    float out_LB_Knee = knee_B + (ext_LB * GAIN);

    // --- RIGHT SIDE (Mirrored) ---
    // IMPORTANT: Right side hip servos usually need to rotate in the opposite 
    // direction to achieve the same forward/backward motion.
    
    // RF is in Group B (Same timing as LB)
    float out_RF_Hip = hip_B + (ext_RF * GAIN); // PID sign might need adjustment here
//    float out_RF_Hip = getMirroredAngle(raw_RF_Hip); // <-- MIRRORING APPLIED HERE
    float out_RF_Knee = knee_B + (ext_RF * GAIN);    // Knees usually don't need mirroring, just offset

    // RB is in Group A (Same timing as LF)
    float out_RB_Hip = hip_A + (ext_RB * GAIN);
//    float out_RB_Hip = getMirroredAngle(raw_RB_Hip); // <-- MIRRORING APPLIED HERE
    float out_RB_Knee = knee_A + (ext_RB * GAIN);

    // 4. Send to Servos
    // LF
    servoCtrl.setAngleImmediate(0, out_LF_Knee);
    servoCtrl.setAngleImmediate(1, out_LF_Hip);
    
    // LB
    servoCtrl.setAngleImmediate(2, out_LB_Knee);
    servoCtrl.setAngleImmediate(3, out_LB_Hip);
    
    // RF (Ensure IDs match your wiring)
    servoCtrl.setAngleImmediate(4, out_RF_Knee);
    servoCtrl.setAngleImmediate(5, out_RF_Hip);
    
    // RB (Ensure IDs match your wiring)
    servoCtrl.setAngleImmediate(6, out_RB_Knee);
    servoCtrl.setAngleImmediate(7, out_RB_Hip);
}
*/
/*
void RobotDog::updateGait(float dt) {
    // 1. 计算当前处于哪个“帧” (模拟 walk 函数的循环索引 sample)
    // 我们把整个步态周期看作 0.0 -> 2.0 (前半段 0-1, 后半段 1-2)
    float phase_change = _gait_freq * dt * (float)STEP_FRAMES * 2.0f; // 缩放速度
    
    if (_dir == 1) _phase += phase_change; // 前进
    else if (_dir == -1) _phase -= phase_change; // 后退
    
    // 循环 _phase 在 0 到 2*STEP_FRAMES 之间
    float max_idx = (float)STEP_FRAMES * 2.0f;
    if (_phase >= max_idx) _phase -= max_idx;
    if (_phase < 0.0f) _phase += max_idx;

    // 2. 查表获取基础角度 (Base Angles)
    float base_LF_Thigh, base_LF_Calf;
    float base_RB_Thigh, base_RB_Calf;
    float base_RF_Thigh, base_RF_Calf;
    float base_LB_Thigh, base_LB_Calf;

    // 获取当前的 float 索引和两个插值点
    float current_pos = _phase;
    // 如果大于 STEP_FRAMES (20)，说明在第二个循环
    bool second_half = (current_pos >= (float)STEP_FRAMES);
    
    // 归一化到 0-19.99 之间用于查表
    float local_idx_float = second_half ? (current_pos - STEP_FRAMES) : current_pos;
    
    // 插值获取当前帧的角度
    float val_1_T, val_1_C; // Angle_1 (Lift/抬腿)
    float val_2_T, val_2_C; // Angle_2 (Pullback/后拉)
    getInterpolatedStep(local_idx_float, val_1_T, val_1_C, val_2_T, val_2_C);

    if (!second_half) {
        // --- 前半周期 (First Half) ---
        // LF & RB: Pullback (Angle_2)
        base_LF_Thigh = val_2_T; base_LF_Calf = val_2_C;
        base_RB_Thigh = val_2_T; base_RB_Calf = val_2_C;
        
        // RF & LB: Lift (Angle_1)
        base_RF_Thigh = val_1_T; base_RF_Calf = val_1_C;
        base_LB_Thigh = val_1_T; base_LB_Calf = val_1_C;
    } else {
        // --- 后半周期 (Second Half) ---
        // LF & RB: Lift (Angle_1)
        base_LF_Thigh = val_1_T; base_LF_Calf = val_1_C;
        base_RB_Thigh = val_1_T; base_RB_Calf = val_1_C;
        
        // RF & LB: Pullback (Angle_2)
        base_RF_Thigh = val_2_T; base_RF_Calf = val_2_C;
        base_LB_Thigh = val_2_T; base_LB_Calf = val_2_C;
    }

    // 3. 计算 PID (陀螺仪控制)
    float pid_pitch, pid_roll;
    calculatePID(pid_pitch, pid_roll);
    

    // 混合 PID 到扩展量 (Extension)
    //roll+: 前腿伸长(+), 后腿缩短(-)
    //pitch+: 右腿伸长(+), 左腿缩短(-)

    float ext_LFU = (pid_pitch + pid_roll) * GAIN;
    float ext_LFD = (-pid_pitch - pid_roll) * GAIN;

    float ext_LBU = (pid_pitch - pid_roll) * GAIN;
    float ext_LBD = (-pid_pitch + pid_roll) * GAIN;

    float ext_RFU = (pid_pitch - pid_roll) * GAIN;
    float ext_RFD = (-pid_pitch + pid_roll) * GAIN;

    float ext_RBU = (pid_pitch + pid_roll) * GAIN;
    float ext_RBD = (-pid_pitch - pid_roll) * GAIN;



    // 4. 应用 PID 并处理反转 (360 - angle)
    // 逻辑： 先算 (基础角度 + PID)，如果是右后/左后，再用 360 减

    // --- LF (Left Front) ---
    // walk: setAngle(6, angle);
    float out_LFU = base_LF_Thigh + ext_LFU;
    float out_LFD = base_LF_Calf + ext_LFD;

    // --- RB (Right Back) ---
    // walk: setAngle(0, 360 - angle);
    // 注意：我们将 PID 加在 angle 上，然后整体被 360 减，这样符合几何逻辑
    float out_RBU = 360.0f - (base_RB_Thigh + ext_RBU);
    float out_RBD = 360.0f - (base_RB_Calf + ext_RBD);

    // --- RF (Right Front) ---
    // walk: setAngle(4, angle);
    float out_RFU = base_RF_Thigh + ext_RFU;
    float out_RFD = base_RF_Calf + ext_RFD;

    // --- LB (Left Back) ---
    // walk: setAngle(2, 360 - angle);
    float out_LBU = 360.0f - (base_LB_Thigh + ext_LBU);
    float out_LBD = 360.0f - (base_LB_Calf + ext_LBD);

    // 5. 写入舵机 (ID 严格按照 walk 函数)
    
    if (_force_stand) {
        // 站立逻辑略，或者设为定值
        servoCtrl.standStill();
    } else {
        // LF
        servoCtrl.setAngleImmediate(1, out_LFU);
        servoCtrl.setAngleImmediate(0, out_LFD);
        // RB
        servoCtrl.setAngleImmediate(7, out_RBU);
        servoCtrl.setAngleImmediate(6, out_RBD);
        // RF
        servoCtrl.setAngleImmediate(5, out_RFU);
        servoCtrl.setAngleImmediate(4, out_RFD);
        // LB
        servoCtrl.setAngleImmediate(3, out_LBU);
        servoCtrl.setAngleImmediate(2, out_LBD);
    }
}
*/

void RobotDog::updateGait(float dt) {
    // 1. 计算当前处于哪个“帧” (模拟 walk 函数的循环索引 sample)
    // 我们把整个步态周期看作 0.0 -> 2.0 (前半段 0-1, 后半段 1-2)
    float phase_change = _gait_freq * dt * (float)STEP_FRAMES * 2.0f; // 缩放速度
    
    if (_dir == 1) _phase += phase_change; // 前进
    else if (_dir == -1) _phase -= phase_change; // 后退
    
    // 循环 _phase 在 0 到 2*STEP_FRAMES 之间
    float max_idx = (float)STEP_FRAMES * 2.0f;
    if (_phase >= max_idx) _phase -= max_idx;
    if (_phase < 0.0f) _phase += max_idx;

    // 2. 查表获取基础角度 (Base Angles)
    float base_LF_Thigh, base_LF_Calf;
    float base_RB_Thigh, base_RB_Calf;
    float base_RF_Thigh, base_RF_Calf;
    float base_LB_Thigh, base_LB_Calf;

    // 获取当前的 float 索引和两个插值点
    float current_pos = _phase;
    // 如果大于 STEP_FRAMES (20)，说明在第二个循环
    bool second_half = (current_pos >= (float)STEP_FRAMES);
    
    // 归一化到 0-19.99 之间用于查表
    float local_idx_float = second_half ? (current_pos - STEP_FRAMES) : current_pos;
    
    // 获取当前时刻的插值角度
    float val_1_T, val_1_C; // 对应 Angles_1 (Lift)
    float val_2_T, val_2_C; // 对应 Angles_2 (Pullback)
    
    getInterpolatedStep(local_idx_float, val_1_T, val_1_C, val_2_T, val_2_C);

    if (!second_half) {
        // === 对应 walk 函数的第一个 for 循环 ===
        // LF & RB pull back (Angle_2), RF & LB lift (Angle_1)
        
        // Group A (LF, RB) -> Pullback
        base_LF_Thigh = val_2_T; base_LF_Calf = val_2_C;
        base_RB_Thigh = val_2_T; base_RB_Calf = val_2_C;
        
        // Group B (RF, LB) -> Lift
        base_RF_Thigh = val_1_T; base_RF_Calf = val_1_C;
        base_LB_Thigh = val_1_T; base_LB_Calf = val_1_C;
        
    } else {
        // === 对应 walk 函数的第二个 for 循环 ===
        // LF & RB lift (Angle_1), RF & LB pull back (Angle_2)
        
        // Group A (LF, RB) -> Lift
        base_LF_Thigh = val_1_T; base_LF_Calf = val_1_C;
        base_RB_Thigh = val_1_T; base_RB_Calf = val_1_C;
        
        // Group B (RF, LB) -> Pullback
        base_RF_Thigh = val_2_T; base_RF_Calf = val_2_C;
        base_LB_Thigh = val_2_T; base_LB_Calf = val_2_C;
    }

    // 3. 计算 PID (陀螺仪控制)
    float pid_pitch, pid_roll;
    calculatePID(pid_pitch, pid_roll);

    // 混合 PID 到扩展量 (Extension)
    // Pitch+: 车头抬起 -> 前腿伸长(+), 后腿缩短(-)
    // Roll+:  右侧抬起 -> 右腿伸长(+), 左腿缩短(-)
    float ext_LF = (pid_pitch - pid_roll) * GAIN;
    float ext_LB = (-pid_pitch - pid_roll) * GAIN;
    float ext_RF = (pid_pitch + pid_roll) * GAIN;
    float ext_RB = (-pid_pitch + pid_roll) * GAIN;

    // 4. 应用 PID 并处理反转 (360 - angle)
    // 逻辑： 先算 (基础角度 + PID)，如果是右后/左后，再用 360 减

    // --- LF (Left Front) ---
    // walk: setAngle(6, angle);
    float out_LF_T = base_LF_Thigh + ext_LF;
    float out_LF_C = base_LF_Calf + ext_LF;

    // --- RB (Right Back) ---
    // walk: setAngle(0, 360 - angle);
    // 注意：我们将 PID 加在 angle 上，然后整体被 360 减，这样符合几何逻辑
    float out_RB_T = 360.0f - (base_RB_Thigh + ext_RB);
    float out_RB_C = 360.0f - (base_RB_Calf + ext_RB);

    // --- RF (Right Front) ---
    // walk: setAngle(4, angle);
    float out_RF_T = base_RF_Thigh + ext_RF;
    float out_RF_C = base_RF_Calf + ext_RF;

    // --- LB (Left Back) ---
    // walk: setAngle(2, 360 - angle);
    float out_LB_T = 360.0f - (base_LB_Thigh + ext_LB);
    float out_LB_C = 360.0f - (base_LB_Calf + ext_LB);

    // 5. 写入舵机 (ID 严格按照 walk 函数)
    
    if (_force_stand) {
        // 站立逻辑略，或者设为定值
        servoCtrl.standStill();
    } else {
        // LF
        servoCtrl.setAngleImmediate(6, out_LF_T);
        servoCtrl.setAngleImmediate(7, out_LF_C);
        // RB
        servoCtrl.setAngleImmediate(0, out_RB_T);
        servoCtrl.setAngleImmediate(1, out_RB_C);
        // RF
        servoCtrl.setAngleImmediate(4, out_RF_T);
        servoCtrl.setAngleImmediate(5, out_RF_C);
        // LB
        servoCtrl.setAngleImmediate(2, out_LB_T);
        servoCtrl.setAngleImmediate(3, out_LB_C);
    }
}

void RobotDog::getInterpolatedStep(float idx, float &t1, float &c1, float &t2, float &c2) {
    int i = (int)idx;
    int next_i = i + 1;
    if (next_i >= STEP_FRAMES) next_i = 0; // 这里的循环无所谓，因为外面控制了阶段切换
    // 如果不希望在数组尾部和头部之间插值（因为是两个动作的切换点），可以限制 next_i = i;

    float alpha = idx - i; // 小数部分

    // 读 Array 1
    t1 = servoThighAngles_1[i] * (1.0f - alpha) + servoThighAngles_1[next_i] * alpha;
    c1 = servoCalfAngles_1[i] * (1.0f - alpha) + servoCalfAngles_1[next_i] * alpha;

    // 读 Array 2
    t2 = servoThighAngles_2[i] * (1.0f - alpha) + servoThighAngles_2[next_i] * alpha;
    c2 = servoCalfAngles_2[i] * (1.0f - alpha) + servoCalfAngles_2[next_i] * alpha;
}

void RobotDog::getInterpolatedAngles(float p, float &thigh, float &calf) {
    p = p - floor(p); 
    float pos = p * NUM_ANGLES;
    int idx = (int)pos;
    int next = (idx + 1) % NUM_ANGLES;
    float alpha = pos - idx;

    auto getT = [&](int i) { return (i < HALF_FRAMES) ? servoThighAngles_1[i] : servoThighAngles_2[i-HALF_FRAMES]; };
    auto getC = [&](int i) { return (i < HALF_FRAMES) ? servoCalfAngles_1[i] : servoCalfAngles_2[i-HALF_FRAMES]; };

    thigh = getT(idx) * (1.0f - alpha) + getT(next) * alpha;
    calf  = getC(idx) * (1.0f - alpha) + getC(next) * alpha;
}

void RobotDog::calculatePID(float &p_out, float &r_out) {
    updateGyro(); 
    float r, p, y;
    getAngles(r, p, y);

    // Subtract tare offset
    float robot_pitch = p;
    float robot_roll  = r; 

    float current_pitch_err = P_Target - robot_pitch;
    float current_roll_err  = R_Target - robot_roll;

    unsigned long now = millis();

    // ===== Print IMU Data =====
    static unsigned long last_print = 0;
    if (now - last_print > 20) {
        last_print = now;
        
        Serial.print("Error R: ");
        Serial.print(current_roll_err); Serial.print(", ");
        Serial.print("Current R: ");
        Serial.println(robot_roll); Serial.print(" ");
        Serial.print("Error P: ");
        Serial.print(current_pitch_err); Serial.print(", ");
        Serial.print("Current P: ");
        Serial.println(robot_pitch); Serial.print(" ");
        Serial.println("");
    }

    r_out = Kp_roll  * current_roll_err;
    p_out = Kp_pitch * current_pitch_err;
    // 限制最大修正幅度，防止舵机打死
    p_out = constrain(p_out, -MAX_PID, MAX_PID);
    r_out = constrain(r_out, -MAX_PID, MAX_PID);
}

float RobotDog::getMirroredAngle(float input_angle) {
    const float CENTER_ANGLE = 172.0;
    return CENTER_ANGLE + (CENTER_ANGLE - input_angle);
}

void RobotDog::LiftLeftFront() {
    servoCtrl.standStill();
    delay(500);
    servoCtrl.setAngleImmediate(1, 200); // Thigh
    servoCtrl.setAngleImmediate(0, 160); // Calf
    delay(1000);
    servoCtrl.standStill();
    delay(5000);
}

void RobotDog::Roll() {
    digitalWrite(5, HIGH);
    digitalWrite(7, LOW);
    analogWrite(6, 127); //50%
    delay(15000);

    digitalWrite(5, HIGH);
    digitalWrite(7, HIGH);

    delay(5000);

    digitalWrite(5, LOW);
    digitalWrite(7, HIGH);
    analogWrite(6, 127); //50%
    delay(5000);

    
}


