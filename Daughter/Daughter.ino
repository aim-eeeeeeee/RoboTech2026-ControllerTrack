/*******************************************************
 * Kid.ino  (Arduino UNO R3 + nRF24 + DRV8833 + Servo + MPU6050)
 * Role: Receive CMD packets from Controller via nRF24, apply motor control, send TEL
 * Implements failsafe timeout (200ms) for safety
 *******************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>        // For I2C (MPU6050)
#include <Servo.h>      // For servo steering
#include <RF24.h>       // nRF24 library (RF24 by TMRh20)
#include "radio_protocol.h"  // Shared protocol definitions

/******************** CONSTANTS & TUNABLES ********************/
// Servo configuration
static const int SERVO_CENTER_ANGLE = 90;  // degrees
static const int SERVO_ANGLE_LIMIT = 45;    // ±30 degrees limit from center

// Motor deadband
static const int16_t MOTOR_DEADBAND = 30;  // ±3% of 1000 range

// Task periods
static const uint32_t CONTROL_PERIOD_MS = 20;  // 50 Hz
static const uint32_t IMU_PERIOD_MS = 10;      // 100 Hz
static const uint32_t TEL_PERIOD_MS = 50;      // 20 Hz

// IMU calibration
static const int IMU_CALIBRATION_SAMPLES = 100;

// MPU6050 I2C address
static const uint8_t MPU6050_ADDR = 0x68;  // AD0 = LOW

/******************** PIN MAP (UNO R3) ********************/
// nRF24L01 (SPI - fixed pins on UNO)
static const int PIN_NRF_CE  = 9;   // Updated: was 8
static const int PIN_NRF_CSN = 10;
// SPI pins: MOSI=11, MISO=12, SCK=13 (fixed on UNO)

// DRV8833 Motor Driver
static const int PIN_MOTOR_AIN1 = 3;  // Updated: was 5, PWM speed
static const int PIN_MOTOR_AIN2 = 4;  // Direction/reverse PWM
static const int PIN_MOTOR_STBY = 2;  // Updated: was 7, Enable (HIGH = drive)

// Servo Steering
static const int PIN_SERVO = 5;  // Updated: was 6, PWM pin

// MPU6050 IMU (I2C - fixed pins on UNO)
// SDA = A4, SCL = A5 (fixed on UNO)

/******************** GLOBALS ********************/
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
Servo servo;
CmdPacket cmd{};
TelPacket tel{};

// Cached command values
static int16_t cachedKidPower = 0;
static int16_t cachedKidSteer = 0;
static uint8_t cachedCmdSeq = 0;
static uint32_t lastCmdMs = 0;

// Failsafe tracking
static bool failsafeActive = false;

// Motor output state (for debug)
static int16_t appliedKidPower = 0;
static int16_t appliedKidSteer = 0;

// IMU data
static ImuData latestImuData{};
static float gyroZBias = 0.0f;  // Gyro Z bias for calibration
static uint32_t lastImuMs = 0;
static float integratedYaw = 0.0f;  // Integrated yaw angle (degrees)
static uint32_t lastImuMicros = 0;

// TEL transmission
static uint8_t telSeq = 0;
static uint32_t lastTelSendMs = 0;
static uint32_t lastControlMs = 0;

/******************** RADIO FUNCTIONS ********************/
/**
 * Initialize nRF24L01 radio module.
 * Configures radio settings, pipes, and starts listening mode.
 */
bool radioInit() {
  // #region agent log
  Serial.print(F("RInit:CE="));
  Serial.print(PIN_NRF_CE);
  Serial.print(F(",CSN="));
  Serial.println(PIN_NRF_CSN);
  // #endregion
  
  // Initialize radio
  if (!radio.begin()) {
    // #region agent log
    Serial.println(F("RInit:FAIL"));
    // #endregion
    Serial.println(F("[Kid Radio] ERROR: Radio hardware not responding!"));
    return false;
  }
  
  // #region agent log
  Serial.println(F("RInit:OK"));
  // #endregion

  // Configure radio settings
  // Use protocol-defined data rate to match Controller
  radio.setDataRate(RADIO_DATA_RATE);  // Must match Controller (250KBPS)
  radio.setChannel(RADIO_CHANNEL);     // Channel 90
  radio.setPALevel(RADIO_PA_LEVEL);   // PA level (LOW for short range)
  
  // Enable auto-acknowledge for reliability
  radio.setAutoAck(true);
  radio.setAutoAck(0, true);  // Enable auto-ack on pipe 0
  
  // Set retry configuration
  radio.setRetries(5, 15);  // 5 retries with 15*250us delay = ~3.75ms max delay
  
  // Set payload size (matching Controller - uses TelPacket size to accommodate both)
  radio.setPayloadSize(sizeof(TelPacket));  // Fixed payload size (accommodates both packets)
  
  // Open pipes
  radio.openReadingPipe(0, ADDR_C2K);    // Listen on ADDR_C2K (where Controller sends CMD)
  radio.openWritingPipe(ADDR_K2C);      // Writing pipe for TEL transmission to Controller
  
  // Start in listening mode (RX)
  radio.startListening();
  
  // #region agent log
  Serial.print(F("RConfig:CH="));
  Serial.print(RADIO_CHANNEL);
  Serial.print(F(",PS="));
  Serial.println(sizeof(TelPacket));
  // #endregion
  
  Serial.println(F("[Kid Radio] OK"));
  
  return true;
}

/**
 * Read command packet from Controller if available.
 * Validates target and caches command values.
 * Returns true if packet was received and valid, false otherwise.
 */
bool readCommandIfAvailable() {
  // Poll for available data
  if (radio.available()) {
    // Read command packet
    radio.read(&cmd, sizeof(cmd));
    
    // #region agent log
    Serial.print(F("PKT:T="));
    Serial.print(cmd.target);
    Serial.print(F(",S="));
    Serial.println(cmd.seq);
    // #endregion
    
    // Validate target - only process if for Kid
    if (cmd.target != TARGET_KID) {
      // CMD is for Mother, not Kid - ignore
      return false;
    }
    
    // Update cached values
    cachedKidPower = cmd.kidPower;
    cachedKidSteer = cmd.kidSteer;
    cachedCmdSeq = cmd.seq;
    lastCmdMs = millis();
    
    return true;
  }
  return false;
}

/******************** MOTOR CONTROL FUNCTIONS ********************/
/**
 * Initialize DRV8833 motor driver.
 * Sets pin modes and initializes to stopped state.
 */
void motorInit() {
  pinMode(PIN_MOTOR_AIN1, OUTPUT);
  pinMode(PIN_MOTOR_AIN2, OUTPUT);
  pinMode(PIN_MOTOR_STBY, OUTPUT);
  
  // Initialize to stopped state
  digitalWrite(PIN_MOTOR_AIN1, LOW);
  digitalWrite(PIN_MOTOR_AIN2, LOW);
  digitalWrite(PIN_MOTOR_STBY, HIGH);  // Enable driver (but motor stopped)
  
  Serial.println(F("[Kid Motor] OK"));
}

/**
 * Set motor power using DRV8833.
 * Maps kidPower (-1000..1000) to PWM control with deadband.
 * 
 * @param power Motor power in range -1000..1000
 */
void motorSet(int16_t power) {
  // Apply deadband (±3% of range)
  if (abs(power) < MOTOR_DEADBAND) {
    power = 0;
  }
  
  // Clamp to valid range
  if (power > 1000) power = 1000;
  if (power < -1000) power = -1000;
  
  // Ensure STBY is HIGH (enabled)
  digitalWrite(PIN_MOTOR_STBY, HIGH);
  
  if (power == 0) {
    // Stop motor (coast mode)
    analogWrite(PIN_MOTOR_AIN1, 0);
    analogWrite(PIN_MOTOR_AIN2, 0);
  } else if (power > 0) {
    // Forward: AIN1=PWM, AIN2=LOW
    uint8_t pwm = map(abs(power), 0, 1000, 0, 255);
    analogWrite(PIN_MOTOR_AIN1, pwm);
    analogWrite(PIN_MOTOR_AIN2, 0);
  } else {
    // Reverse: AIN1=LOW, AIN2=PWM
    uint8_t pwm = map(abs(power), 0, 1000, 0, 255);
    analogWrite(PIN_MOTOR_AIN1, 0);
    analogWrite(PIN_MOTOR_AIN2, pwm);
  }
}

/**
 * Stop motor (coast mode).
 * Sets both AIN1 and AIN2 to LOW.
 */
void motorStop() {
  analogWrite(PIN_MOTOR_AIN1, 0);
  analogWrite(PIN_MOTOR_AIN2, 0);
  // STBY remains HIGH (driver enabled but motor stopped)
}

/******************** SERVO CONTROL FUNCTIONS ********************/
/**
 * Initialize servo steering.
 * Attaches servo to pin and sets center position.
 */
void steerInit() {
  servo.attach(PIN_SERVO);
  servo.write(SERVO_CENTER_ANGLE);  // Center position
  delay(500);  // Allow servo to reach position
  Serial.println(F("[Kid Servo] OK"));
}

/**
 * Set steering angle based on kidSteer value.
 * Interprets kidSteer as absolute angle in degrees with ±30° limit.
 * 
 * @param steer Absolute angle in degrees (interpreted from kidSteer value)
 */
void steerSet(int16_t steer) {
  // 添加死区（±5%）
  if (abs(steer) < 70) {
    steer = 0;
  }
  
  // 映射 -1000~1000 到 -30~+30 度
  int16_t angle_offset = map(steer, -1000, 1000, -SERVO_ANGLE_LIMIT, SERVO_ANGLE_LIMIT);
  int angle = SERVO_CENTER_ANGLE + angle_offset;
  
  // 保护性限制（理论上map后已经在范围内）
  angle = constrain(angle, 45, 135);
  
  // 调试输出（可选）
  Serial.print(F("Steer: "));
  Serial.print(steer);
  Serial.print(F(" → Offset: "));
  Serial.print(angle_offset);
  Serial.print(F(" → Angle: "));
  Serial.println(angle);
  
  servo.write(angle);
}

/******************** IMU FUNCTIONS ********************/
/**
 * Initialize MPU6050 IMU.
 * Configures I2C, wakes up MPU6050, and performs gyro bias calibration.
 */
void imuInit() {
  Wire.begin();
  
  // Wake up MPU6050 (exit sleep mode)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Set to 0 to wake up
  Wire.endTransmission();
  
  delay(100);  // Wait for MPU6050 to stabilize
  
  // Configure MPU6050
  // Set accelerometer range to ±2g
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x00);  // ±2g range
  Wire.endTransmission();
  
  // Set gyro range to ±250 deg/s
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x00);  // ±250 deg/s range
  Wire.endTransmission();
  
  // Gyro bias calibration
  Serial.println(F("[Kid IMU] Calibrating..."));
  float sumGz = 0.0f;
  for (int i = 0; i < IMU_CALIBRATION_SAMPLES; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);  // GYRO_XOUT_H register
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)6, (uint8_t)true);
    
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();
    
    // Convert to deg/s (±250 deg/s range, 16-bit = 32768 counts)
    float gz_dps = (gz / 32768.0f) * 250.0f;
    sumGz += gz_dps;
    delay(10);
  }
  gyroZBias = sumGz / IMU_CALIBRATION_SAMPLES;
  
  Serial.println(F("[Kid IMU] OK"));
  
  lastImuMicros = micros();
}

/**
 * Read MPU6050 IMU data and compute attitude.
 * Updates latestImuData with accel, gyro, and computed angles.
 */
void updateImu() {
  // Read accelerometer data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)6, (uint8_t)true);
  
  int16_t ax_raw = (Wire.read() << 8) | Wire.read();
  int16_t ay_raw = (Wire.read() << 8) | Wire.read();
  int16_t az_raw = (Wire.read() << 8) | Wire.read();
  
  // Convert to m/s² (±2g range, 16-bit = 32768 counts, 1g = 9.81 m/s²)
  latestImuData.ax = (ax_raw / 32768.0f) * 2.0f * 9.81f;
  latestImuData.ay = (ay_raw / 32768.0f) * 2.0f * 9.81f;
  latestImuData.az = (az_raw / 32768.0f) * 2.0f * 9.81f;
  
  // Read gyroscope data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43);  // GYRO_XOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)6, (uint8_t)true);
  
  int16_t gx_raw = (Wire.read() << 8) | Wire.read();
  int16_t gy_raw = (Wire.read() << 8) | Wire.read();
  int16_t gz_raw = (Wire.read() << 8) | Wire.read();
  
  // Convert to deg/s (±250 deg/s range)
  float gx_dps = (gx_raw / 32768.0f) * 250.0f;
  float gy_dps = (gy_raw / 32768.0f) * 250.0f;
  float gz_dps = (gz_raw / 32768.0f) * 250.0f;
  
  // Apply bias correction
  gz_dps -= gyroZBias;
  
  // Compute roll and pitch from accelerometer (simple method)
  latestImuData.roll = atan2(latestImuData.ay, latestImuData.az) * 180.0f / PI;
  latestImuData.pitch = atan2(-latestImuData.ax, sqrt(latestImuData.ay * latestImuData.ay + latestImuData.az * latestImuData.az)) * 180.0f / PI;
  
  // Integrate yaw from gyro Z
  uint32_t nowMicros = micros();
  float dt = (nowMicros - lastImuMicros) / 1000000.0f;  // Convert to seconds
  if (dt > 0.0f && dt < 1.0f) {  // Sanity check (avoid huge dt on first call)
    integratedYaw += gz_dps * dt;
    // Keep yaw in reasonable range (optional: wrap to -180..180)
    if (integratedYaw > 180.0f) integratedYaw -= 360.0f;
    if (integratedYaw < -180.0f) integratedYaw += 360.0f;
  }
  lastImuMicros = nowMicros;
  
  latestImuData.yaw = integratedYaw;
}

/******************** ACTUATOR CONTROL FUNCTIONS ********************/
/**
 * Update actuators from cached command values.
 * Applies motor and steering control, respecting failsafe state.
 * Keeps last state when failsafe is not active (packet loss handling).
 */
void updateActuatorsFromCommand() {
  // Check failsafe first (only active on initial startup)
  if (failsafeActive) {
    // Failsafe active - stop motor and center steering (initial state only)
    motorStop();
    steerSet(0);  // Center steering (0 degrees offset)
    appliedKidPower = 0;
    appliedKidSteer = 0;
    return;
  }
  
  // Apply motor control (uses cached values, keeps last state if no new packet)
  motorSet(cachedKidPower);
  appliedKidPower = cachedKidPower;
  
  // Apply steering control (uses cached values, keeps last state if no new packet)
  // kidSteer is interpreted as absolute angle offset in degrees
  steerSet(cachedKidSteer);
  appliedKidSteer = cachedKidSteer;
}

/******************** FAILSAFE FUNCTIONS ********************/
/**
 * Check failsafe condition and update failsafe state.
 * Only activates failsafe on initial startup (no CMD received yet).
 * Once a CMD is received, keeps last state even if packets are lost.
 */
void checkFailsafe() {
  uint32_t now = millis();
  
  if (lastCmdMs > 0) {
    // Already received at least one CMD - keep last state, don't activate failsafe on packet loss
    // Clear failsafe if it was active
    if (failsafeActive) {
      failsafeActive = false;
      Serial.println(F("[Kid] FAILSAFE OFF"));
    }
  } else {
    // Haven't received any CMD yet - failsafe active (initial state)
    if (!failsafeActive) {
      failsafeActive = true;
      Serial.println(F("[Kid] FAILSAFE ON (initial)"));
      motorStop();
      steerSet(0);  // Center steering (0 degrees offset)
    }
  }
}

/******************** TELEMETRY FUNCTIONS ********************/
/**
 * Send telemetry packet to Controller if due.
 * Populates TelPacket with IMU data and link status.
 */
void sendTelemetryIfDue() {
  // Build telemetry packet
  tel.seq = telSeq++;
  
  // Set linkFlags
  tel.linkFlags = 0x01;  // bit0 = kidOnline
  if (failsafeActive) {
    tel.linkFlags |= 0x02;  // bit1 = controllerTimeout
  }
  
  // Populate Kid IMU data
  tel.imuK = latestImuData;
  
  // Optional fields
  tel.distK = 0.0f;  // Distance not used
  
  // Clear Mother data (not used by Kid)
  tel.imuM.ax = 0.0f;
  tel.imuM.ay = 0.0f;
  tel.imuM.az = 0.0f;
  tel.imuM.roll = 0.0f;
  tel.imuM.pitch = 0.0f;
  tel.imuM.yaw = 0.0f;
  tel.distM = 0.0f;
  tel.speedM = 0.0f;
  
  // Stop listening to enable TX
  radio.stopListening();
  
  // Send packet
  bool success = radio.write(&tel, sizeof(tel));
  
  // Return to listening mode
  radio.startListening();
  
  // Optional: track TX success for linkFlags (not implemented for now)
  (void)success;  // Suppress unused variable warning
}

/******************** DEBUG FUNCTIONS ********************/
// Removed printCmdPacket() to save memory - can be re-enabled if needed

/******************** SETUP ********************/
void setup() {
  Serial.begin(115200);
  delay(1000);  // Give Serial time to initialize
  
  Serial.println();
  Serial.println(F("[Kid] Starting..."));
  
  // #region agent log
  Serial.print(F("Pins:CE="));
  Serial.print(PIN_NRF_CE);
  Serial.print(F(",CSN="));
  Serial.print(PIN_NRF_CSN);
  Serial.print(F(",A1="));
  Serial.print(PIN_MOTOR_AIN1);
  Serial.print(F(",A2="));
  Serial.print(PIN_MOTOR_AIN2);
  Serial.print(F(",STBY="));
  Serial.print(PIN_MOTOR_STBY);
  Serial.print(F(",SERVO="));
  Serial.println(PIN_SERVO);
  // #endregion
  
  // SPI init for nRF (UNO uses fixed SPI pins)
  SPI.begin();
  
  // Initialize hardware modules
  motorInit();
  steerInit();
  imuInit();
  
  // nRF24 radio init
  if (!radioInit()) {
    Serial.println(F("[Kid] ERROR: Radio init failed!"));
    while (1) {
      delay(1000);  // Halt on error
    }
  }
  
  // Initialize timestamps
  lastCmdMs = 0;
  lastControlMs = 0;
  lastImuMs = 0;
  lastTelSendMs = 0;
  
  Serial.println(F("[Kid] Ready"));
}

/******************** LOOP ********************/
void loop() {
  uint32_t now = millis();
  
  // 1) Radio RX (always, every loop - highest priority)
  if (readCommandIfAvailable()) {
    // Packet received - print to Serial for debugging (optional, can be throttled)
    // printCmdPacket();  // Uncomment for debug
  }
  
  // 2) Failsafe check
  checkFailsafe();
  
  // 3) Control update (periodic, 50 Hz)
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    updateActuatorsFromCommand();
    lastControlMs = now;
  }
  
  // 4) IMU update (periodic, 100 Hz)
  if (now - lastImuMs >= IMU_PERIOD_MS) {
    updateImu();
    lastImuMs = now;
  }
  
  // 5) Telemetry TX (periodic, 20 Hz)
  if (now - lastTelSendMs >= TEL_PERIOD_MS) {
    sendTelemetryIfDue();
    lastTelSendMs = now;
  }
  
  // No delay needed - non-blocking continuous polling
}
