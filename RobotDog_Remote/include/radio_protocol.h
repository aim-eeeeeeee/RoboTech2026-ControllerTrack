/*******************************************************
 * radio_protocol.h
 * Shared radio protocol definitions for Controller, Mother, and Kid
 * 
 * This header contains:
 * - Packet structures (CmdPacket, TelPacket, ImuData)
 * - Target enum
 * - Radio addresses (4 addresses for direct star topology)
 * - Radio configuration constants
 *******************************************************/

#ifndef RADIO_PROTOCOL_H
#define RADIO_PROTOCOL_H

#include <Arduino.h>
#include <RF24.h>

/******************** TARGET ENUM ********************/
enum Target : uint8_t { 
  TARGET_MOTHER = 0, 
  TARGET_KID = 1 
};

/******************** PACKET STRUCTS ********************/
// Keep structs fixed-size and identical across nodes.

struct ImuData {
  float ax, ay, az;    // acceleration
  float roll, pitch, yaw; // angles (or attitude)
};

struct CmdPacket {
  uint8_t  seq;        // increments every send
  uint8_t  target;     // TARGET_MOTHER / TARGET_KID
  int16_t  leftPower;  // e.g. -1000..1000 (Mother: left wheel, Kid: 0)
  int16_t  rightPower; // e.g. -1000..1000 (Mother: right wheel, Kid: 0)
  int16_t  kidPower;   // -1000..1000 (Kid: forward/back, Mother: 0)
  int16_t  kidSteer;   // -1000..1000 (Kid: turn left/right, Mother: 0)
  int8_t   winch;      // -1 rev, 0 stop, +1 fwd
  uint8_t  flags;      // spare bits for future
};

struct TelPacket {
  uint8_t seq;
  uint8_t linkFlags;   // bit0=kidOnline, bit1=controllerTimeout, etc.
  ImuData imuM;
  float   distM;
  float   speedM;      // optional, can be 0 if unused
  ImuData imuK;        // optional, can be 0 if unused
  float   distK;       // optional
};

/******************** RADIO ADDRESSES ********************/
// 5-byte addresses for direct star topology (Plan A)
// Controller communicates directly with both Mother and Kid

static const uint8_t ADDR_C2M[6] = "C2M01"; // Controller -> Mother (CMD)
static const uint8_t ADDR_M2C[6] = "M2C01"; // Mother -> Controller (TEL)
static const uint8_t ADDR_C2K[6] = "C2K01"; // Controller -> Kid (CMD)
static const uint8_t ADDR_K2C[6] = "K2C01"; // Kid -> Controller (TEL)

/******************** RADIO CONFIGURATION CONSTANTS ********************/
// Shared radio settings across all nodes

static const uint8_t RADIO_CHANNEL = 90;
static const rf24_datarate_e RADIO_DATA_RATE = RF24_250KBPS;
static const rf24_pa_dbm_e RADIO_PA_LEVEL = RF24_PA_LOW;
static const uint32_t FAILSAFE_MS = 200;  // Failsafe timeout (200ms)

#endif // RADIO_PROTOCOL_H
