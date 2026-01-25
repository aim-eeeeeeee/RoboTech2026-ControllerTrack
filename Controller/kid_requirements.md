# Kid Car Firmware Requirements (UNO R3 + nRF24 + DRV8833 + Servo + MPU6050)

## 0) Purpose / Big Picture
The Kid Car firmware runs on an Arduino UNO R3. It must:
1. Receive control telepackets (commands) via nRF24L01 from a Controller node.
2. Convert the received command data into actuator outputs:
   - TT DC motor driven through DRV8833 (according to middle/for-kid joystick x-axis)
   - Servo steering (front steering bridge) (according to mid/for-kid joystick y-axis)
3. Read IMU data from MPU6050 (accel + gyro; compute roll/pitch/yaw or at least raw + integrated yaw).
4. Transmit telemetry packets back to Controller via nRF24L01.
5. All transmitted and received payload formats MUST follow `radio_protocol.h` (struct layouts, packing, addresses, flags, sequences).

---

## 1) Hardware & Wiring Requirements

### 1.1 Microcontroller
- Arduino UNO R3 (ATmega328P, 16 MHz, 5V logic)

### 1.2 Radio: nRF24L01 (SPI, 3.3V only)
- nRF24L01 MUST be powered by **stable 3.3V** (prefer external LDO ≥150mA).
- SPI pins are fixed on UNO:
  - MOSI = D11
  - MISO = D12
  - SCK  = D13
- Required control pins:
  - CE  = configurable digital pin (recommend D8 if servo uses D6/D9)
  - CSN = configurable digital pin (recommend D10)

### 1.3 Motor Driver: DRV8833
- Use only **A channel** for 1 TT motor:
  - AO1 / AO2 → TT motor terminals
- Motor power:
  - VM → motor battery positive (TT typically 3–6V; do not exceed driver spec)
  - GND → motor battery ground
- MUST share common ground with UNO + radio + servo + IMU.

**Recommended control pin mapping (non-conflicting):**
- AIN1 = D5 (PWM speed)
- AIN2 = D4 (direction or reverse PWM)
- STBY = D7 (enable; must be HIGH to drive)

### 1.4 Steering: Servo
- Servo signal pin uses PWM-capable pin (recommend D6 or D9).
- Servo power MUST be external 5V (≥2A recommended).
- Servo power ground MUST connect to UNO GND (common ground).
- Add bulk capacitor on servo supply rails (≥470µF recommended) to reduce brownouts.

### 1.5 IMU: MPU6050 (I2C)
- I2C pins on UNO:
  - SDA = A4
  - SCL = A5
- VCC to 5V (or 3.3V if module requires; many support 3–5V)
- GND to common ground
- AD0 default low → address 0x68 (unless protocol requires otherwise)
- INT optional (not required unless using DMP/interupts)

---

## 2) Protocol & Data Requirements (radio_protocol.h)

### 2.1 Mandatory Compliance
- All packet structs, enums, flags, target IDs, pipe addresses, and timing constants MUST match `radio_protocol.h`.
- Payload sizes MUST match struct sizes exactly.
- Use the same endian assumptions as controller (UNO = little-endian).
- Avoid dynamic allocation; use fixed-size structs.

### 2.2 Expected Message Types
The system uses star topology:
- Controller → Kid: **Command packet** (CmdPacket)
- Kid → Controller: **Telemetry packet** (TelPacket)

Kid firmware MUST:
- Read only packets intended for the Kid target (e.g., `cmd.target == TARGET_KID`)
- Ignore packets for other targets (e.g., cart/camera/arm, etc.)

### 2.3 Pipe / Addressing
Kid radio MUST configure:
- **Reading pipe**: Controller-to-Kid address (e.g., `ADDR_C2K`)
- **Writing pipe**: Kid-to-Controller address (e.g., `ADDR_K2C`)

> Exact address strings MUST be taken from `radio_protocol.h`.

### 2.4 Sequence Numbers
- CmdPacket includes `seq` (monotonic or wrapping).
- TelPacket includes `seq` (monotonic or wrapping).
Kid MUST:
- Track last received cmd seq (for diagnostics and optional duplicate drop).
- Increment tel seq on each telemetry send.

### 2.5 Flags & Link State
- CmdPacket may contain flags (e.g., arming, mode, e-stop).
- TelPacket includes `linkFlags` (e.g., rx ok, tx ok, failsafe active).
Kid MUST:
- Set a "failsafe/timeout" flag when command stream is missing.
- Optionally set flags indicating radio rx/tx success.

> Exact bit meanings MUST match `radio_protocol.h`.

---

## 3) Runtime Behavior Requirements

### 3.1 Main Loop Must Be Non-Blocking
- No long `delay()` in the main loop.
- Use `millis()` / `micros()` scheduling for periodic tasks:
  - Radio RX: as fast as possible (every loop)
  - Control update: 50–100 Hz
  - IMU update: 100–200 Hz if feasible
  - Telemetry TX: 20–50 Hz

### 3.2 Command Reception & Validation
On each loop iteration:
1. If `radio.available()`:
   - Read CmdPacket
   - Validate:
     - `cmd.target` matches Kid target
     - (optional) check packet version/magic if defined
   - Update:
     - `lastCmdMs = millis()`
     - cached `kidPower`, `kidSteer` (and other relevant cmd fields)
     - cached `cmdSeq`

### 3.3 Failsafe
- Use failsafe timeout constant from protocol (e.g., `FAILSAFE_MS = 200`).
- If `(millis() - lastCmdMs) > FAILSAFE_MS`:
  - Motor output MUST go to STOP (PWM=0)
  - Steering MUST go to CENTER angle (configurable, default 90°)
  - Telemetry MUST indicate failsafe state via linkFlags

Failsafe behavior MUST override all control.

### 3.4 Actuator Control Mapping

#### 3.4.1 TT Motor via DRV8833
Inputs:
- `kidPower` from CmdPacket (range defined by protocol)

Behavior:
- Map `kidPower` to:
  - direction (forward/reverse)
  - PWM magnitude (0–255)
- Implement a deadband around zero to prevent jitter (e.g., ±3% of range).
- Ensure STBY is HIGH when driving.
- Provide `motorStop()` that coasts or brakes (choose one consistent behavior):
  - Coast: PWM=0 on both pins
  - Brake: both pins HIGH (if desired and safe)

#### 3.4.2 Servo Steering
Inputs:
- `kidSteer` from CmdPacket (range defined by protocol)

Behavior:
- Map `kidSteer` to servo angle:
  - center angle default = 90°
  - clamp to [minAngle, maxAngle] to avoid mechanical stall
- Optionally apply smoothing (low-pass or rate limit) to reduce twitch.

### 3.5 IMU Sampling & Output
Kid MUST read:
- Raw accel: ax, ay, az
- Raw gyro: gx, gy, gz

Kid SHOULD compute (minimum viable):
- gyroZ_dps
- integrated yaw (degrees) using dt from `micros()`

Kid MAY compute:
- roll/pitch using accel + complementary filter
- full 3D attitude estimate

Telemetry MUST include:
- `imuK` data per protocol:
  - at minimum: accel + gyro fields
  - if protocol includes roll/pitch/yaw, fill them (or set to 0 if not used, but MUST be consistent)

> The exact `imuK` struct layout MUST match `radio_protocol.h`.

### 3.6 Telemetry Transmission
At `TEL_PERIOD_MS` (or chosen cadence):
1. Populate TelPacket:
   - `tel.seq++`
   - `tel.linkFlags` updated:
     - rx recent vs timeout
     - failsafe active or not
     - optional tx success/fail status
   - `tel.imuK = latest imu reading`
   - optional `tel.distK`, `tel.speedK` if defined by protocol (else omit)
2. Transmit:
   - radio.stopListening()
   - `radio.write(&tel, sizeof(tel))`
   - radio.startListening()
3. Record tx success to set/clear a tx status flag if defined.

---

## 4) Software Architecture Requirements

### 4.1 Modules / Functions
KidCar.ino MUST implement these logical modules (can be functions):
- `radioInit()`
- `imuInit()`
- `motorInit()`, `motorSet(pwm, dir)`, `motorStop()`
- `steerInit()`, `steerSet(angle)`
- `readCommandIfAvailable()` → updates cached command + timestamps
- `updateActuatorsFromCommand()` → uses cached values or failsafe
- `updateImu()` → reads sensor + updates computed yaw/attitude
- `sendTelemetryIfDue()`

### 4.2 Constants & Tunables
Expose tunables at the top of the file:
- Servo center/min/max angles
- Motor deadband
- Motor PWM cap (optionally)
- IMU gyro bias calibration samples count
- Task periods (if not in protocol)

### 4.3 Logging (Optional)
- Support Serial debug compile-time toggle:
  - packet rx count, tx count
  - failsafe on/off transitions
  - imu values (throttled)

---

## 5) Reliability / Safety Requirements

### 5.1 Power & Brownout Robustness
- Servo and motor power transients MUST NOT reset UNO.
- Enforce common ground.
- If resets happen, reduce servo update rate, add caps, and use external 5V buck.

### 5.2 Radio Robustness
- Set data rate and PA level to stable defaults (per controller):
  - Data rate: 1Mbps recommended for stability
  - PA: LOW recommended unless range demands higher
- Implement failsafe strictly.

### 5.3 Timing Guarantees
- No task may block >5ms repeatedly.
- IMU + telemetry should not starve radio RX.

---

## 6) Acceptance Tests (Must Pass)

### 6.1 Radio RX
- When controller sends CmdPacket at ~20ms period:
  - Kid receives packets and updates `lastCmdMs`.
  - Failsafe flag remains cleared.

### 6.2 Motor Control
- Increase `kidPower` positive → motor forward speeds up.
- Negative → motor reverses.
- Near zero → motor stops (deadband works).

### 6.3 Steering
- Vary `kidSteer` left/right → servo angles change smoothly.
- Servo stays within clamp range, no binding.

### 6.4 IMU
- Serial debug shows gyro/accel changing with movement.
- Yaw integrates and changes when rotating car.

### 6.5 Telemetry TX
- Controller receives TelPacket continuously at expected rate.
- Telemetry contains IMU fields and correct seq increments.

### 6.6 Failsafe
- Turn off controller or stop sending:
  - Within FAILSAFE_MS, motor stops, steering centers.
  - Telemetry indicates failsafe active (linkFlags).
  - When command resumes, exits failsafe.

---

## 7) Implementation Notes / Constraints
- Use `RF24` library (TMRh20).
- Use `Wire` for I2C.
- Use `Servo` library.
- Avoid floating point heavy math unless necessary (UNO is slow).
- If protocol uses packed structs, include `#pragma pack(push, 1)` or `__attribute__((packed))` exactly as in `radio_protocol.h`.

---

## 8) TODO When radio_protocol.h is available
When `radio_protocol.h` is re-provided, finalize:
- Exact pipe addresses `ADDR_C2K`, `ADDR_K2C`
- Exact packet struct fields (CmdPacket, TelPacket, imuK)
- Exact flag bit definitions and target IDs
- Exact timing constants (CMD_PERIOD_MS, FAILSAFE_MS, etc.)
- Any CRC/version fields or magic bytes


Block diagram:
                ┌──────────────────────────────────────────┐
                │              START / BOOT                │
                └──────────────────────────────────────────┘
                                  |
                                  v
┌────────────────────────────────────────────────────────────────────┐
│ INIT HARDWARE                                                      │
│  - GPIO: DRV8833 (AIN1/AIN2/STBY), Servo PWM                        │
│  - I2C: MPU6050 init + optional gyro bias calibrate                 │
│  - nRF24: begin(), setChannel(RADIO_CHANNEL), setDataRate(...),     │
│          setPALevel(...), openReadingPipe(C2K), openWritingPipe(K2C)│
│  - State vars: lastCmdMs, cmdSeqSeen, telSeq, failsafe default      │
└────────────────────────────────────────────────────────────────────┘
                                  |
                                  v
                ┌──────────────────────────────────────────┐
                │                 MAIN LOOP                 │
                └──────────────────────────────────────────┘
                                  |
          ┌───────────────────────┴────────────────────────┐
          |                                                |
          v                                                v
┌──────────────────────────────┐                ┌──────────────────────────────┐
│ RADIO RX TASK (fast, always)  │                │ FAILSAFE / CONTROL TASK      │
│  if radio.available():        │                │  if (now - lastCmdMs) >      │
│    read CmdPacket             │                │      FAILSAFE_MS:             │
│    if cmd.target==TARGET_KID: │                │     motor=0, steer=center     │
│      lastCmdMs = now          │                │     set linkFlag timeout bit  │
│      cache kidPower,kidSteer  │                │  else:                        │
│      cache cmd.seq            │                │     map kidPower → motor PWM  │
│      clear timeout flag       │                │     map kidSteer → servo angle│
└──────────────────────────────┘                └──────────────────────────────┘
          |                                                |
          v                                                v
┌──────────────────────────────┐                ┌──────────────────────────────┐
│ IMU SAMPLE TASK (periodic)    │                │ TELEMETRY TX TASK (periodic) │
│  read accel ax,ay,az          │                │  build TelPacket:             │
│  compute roll/pitch/yaw       │                │   tel.seq++                   │
│   (complementary filter or    │                │   tel.linkFlags bits          │
│    integrated gyro yaw)       │                │   tel.imuK = imu              │
│  store to imuK                │                │   tel.distK/speedK optional   │
└──────────────────────────────┘                │  radio.stopListening()        │
          |                                     │  radio.write(&tel,sizeof)     │
          v                                     │  radio.startListening()       │
┌──────────────────────────────┐                └──────────────────────────────┘
│ (optional) DEBUG / LED        │
│  heartbeat, packet counters   │
└──────────────────────────────┘

