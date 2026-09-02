/*******************************************************
 * controller.ino  (ESP32 + nRF24 + OLED + Joysticks)
 * Role: Read inputs -> Send CMD to Mother -> Receive TEL -> Update OLED UI
 *******************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>        // For OLED I2C
#include <U8g2lib.h>     // OLED library
#include <math.h>
#include <RF24.h>        // nRF24 library (RF24 by TMRh20 for ESP32)
#include "radio_protocol.h"  // Shared protocol definitions

/******************** PIN MAP (Controller) ********************/
// nRF24L01 (VSPI)
static const int PIN_NRF_CE  = 4;
static const int PIN_NRF_CSN = 5;
static const int PIN_SPI_SCK  = 18;
static const int PIN_SPI_MOSI = 23;
static const int PIN_SPI_MISO = 19;

// Joysticks (ADC1)
static const int PIN_J1X = 36; // ADC1_CH0
static const int PIN_J1Y = 39; // ADC1_CH3
static const int PIN_J2X = 34; // ADC1_CH6
static const int PIN_J2Y = 35; // ADC1_CH7
static const int PIN_J3X = 32; // ADC1_CH4
static const int PIN_J3Y = 33; // ADC1_CH5

// Joystick Z-axis buttons (internal pull-up)
static const int PIN_J1Z = 25; // Left joystick Z (winch extend: +1)
static const int PIN_J2Z = 26; // Right joystick Z (winch retract: -1)
static const int PIN_J3Z = 27; // Middle joystick Z (target toggle: Mother/Kid)

// OLED I2C
static const int PIN_OLED_SDA = 21;
static const int PIN_OLED_SCL = 22;

/******************** RADIO ADDRESSES ********************/
// Radio addresses are now defined in radio_protocol.h
// ADDR_C2M, ADDR_M2C, ADDR_C2K, ADDR_K2C

/******************** PACKET STRUCTS ********************/
// Packet structures are now defined in radio_protocol.h
// CmdPacket, TelPacket, ImuData, Target enum

/******************** GLOBALS ********************/
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
CmdPacket cmd{};
TelPacket telMother{};  // Telemetry from Mother
TelPacket telKid{};     // Telemetry from Kid
uint32_t lastCmdSendMs = 0;
uint32_t lastUiMs = 0;
uint8_t cmdSeq = 0;

// Scheduling
static const uint32_t CMD_PERIOD_MS = 20;  // 50 Hz
static const uint32_t UI_PERIOD_MS  = 50;  // 20 Hz UI refresh

// Radio configuration constants are now in radio_protocol.h
static const uint32_t TEL_TIMEOUT_MS = FAILSAFE_MS;  // Link lost if no TEL for 200ms

// Link status tracking (separate for Mother and Kid)
static uint32_t lastTelMotherMs = 0;
static uint32_t lastTelKidMs = 0;
static bool linkStableMother = false;
static bool linkStableKid = false;

// Joystick Z-button state tracking (for edge detection and toggle)
static bool prevLeftZ = HIGH;    // Previous state of left joystick Z
static bool prevRightZ = HIGH;   // Previous state of right joystick Z
static bool prevMiddleZ = HIGH; // Previous state of middle joystick Z
static uint8_t currentTarget = TARGET_MOTHER; // Current target state (persists)

// OLED Display (Dual OLED setup)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oledLeft(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // 0x3C - Control screen
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oledRight(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  // 0x3D - Telemetry screen

// TX success/fail tracking for debug
static uint32_t cmdTxSuccessCount = 0;
static uint32_t cmdTxFailCount = 0;

/******************** UTILITIES ********************/
// Map ADC (0..4095) -> (-1..1) with deadzone
float adcToNorm(int raw, int center = 2048, int deadzone = 80) {
  int v = raw - center;
  if (abs(v) < deadzone) return 0.0f;
  float norm = (float)v / 2048.0f;
  if (norm > 1) norm = 1;
  if (norm < -1) norm = -1;
  return norm;
}

int16_t normToPower(float x) {
  // scale to -1000..1000 (you can change)
  return (int16_t)(x * 1000.0f);
}

/******************** OLED UI HELPERS ********************/
static inline int clampInt(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
static inline float clampFloat(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
static inline int absInt(int v) { return (v < 0) ? -v : v; }
static inline float deg2rad(float d) { return d * (float)M_PI / 180.0f; }

// Convert power (-1000..1000) to percentage (0..100)
// Apply deadband: if power is within ±7%, return 0% (handles joystick drift)
static int powerToPct(int16_t power) {
  // Apply deadband: ±70 units (±7%) maps to 0%
  if (absInt(power) <= 70) {
    return 0;
  }
  return clampInt(absInt(power) / 10, 0, 100);
}

// Draw horizontal progress bar (filled + unfilled portion with dotted pattern)
void drawProgressBar(U8G2 &oled, int x, int y, int w, int h, int pct) {
  pct = clampInt(pct, 0, 100);
  int fillW = (w * pct) / 100;
  int unfillW = w - fillW;
  
  // Draw filled portion (solid box)
  if (fillW > 0) {
    oled.drawBox(x, y, fillW, h);
  }
  
  // Draw unfilled portion (dotted pattern - vertical lines spaced)
  if (unfillW > 0) {
    for (int i = 0; i < unfillW; i += 3) {
      oled.drawVLine(x + fillW + i, y, h);
    }
  }
}

// Draw upward arrow (for winch PULLING)
void drawUpArrow(U8G2 &oled, int x, int y) {
  // Small upward triangle
  oled.drawLine(x, y, x - 2, y - 3);
  oled.drawLine(x, y, x + 2, y - 3);
  oled.drawLine(x - 2, y - 3, x + 2, y - 3);
}

// Draw status dot (indicator)
void drawStatusDot(U8G2 &oled, int x, int y, bool active) {
  if (active) {
    oled.drawDisc(x, y, 2, U8G2_DRAW_ALL);
  } else {
    oled.drawCircle(x, y, 2, U8G2_DRAW_ALL);
  }
}

// Draw speed gauge (horizontal bar, 0-60 km/h)
void drawSpeedGauge(U8G2 &oled, int x, int y, int w, int h, float speed, float maxSpeed) {
  speed = clampFloat(speed, 0, maxSpeed);
  
  // Draw frame
  oled.drawFrame(x, y, w, h);
  
  // Draw scale labels
  oled.setFont(u8g2_font_4x6_tf);
  oled.drawStr(x + 2, y + 7, "0");
  char buf[8];
  snprintf(buf, sizeof(buf), "%.0f", maxSpeed);
  oled.drawStr(x + w - 12, y + 7, buf);
  
  // Draw gauge bar
  int barX = x + 2;
  int barY = y + 9;
  int barW = w - 4;
  int barH = 4;
  oled.drawFrame(barX, barY, barW, barH);
  
  // Draw filled portion
  int fillW = (int)((speed / maxSpeed) * (barW - 2));
  if (fillW > 0) {
    oled.drawBox(barX + 1, barY + 1, fillW, barH - 2);
  }
  
  // Draw upward arrow below gauge
  int arrowX = x + w / 2;
  int arrowY = y + h - 2;
  oled.drawLine(arrowX, arrowY, arrowX - 2, arrowY - 3);
  oled.drawLine(arrowX, arrowY, arrowX + 2, arrowY - 3);
  oled.drawLine(arrowX - 2, arrowY - 3, arrowX + 2, arrowY - 3);
  
  // Draw speed value
  oled.setFont(u8g2_font_5x8_tf);
  snprintf(buf, sizeof(buf), "%.0f km/h", speed);
  oled.drawStr(x + 2, y + h - 1, buf);
}

// Draw XYZ motion crosshair with position dot
void drawXYZMotion(U8G2 &oled, int x, int y, int w, int h, float xVal, float yVal, float maxVal) {
  xVal = clampFloat(xVal, -maxVal, maxVal);
  yVal = clampFloat(yVal, -maxVal, maxVal);
  
  // Draw frame
  oled.drawFrame(x, y, w, h);
  
  // Center point
  int cx = x + w / 2;
  int cy = y + h / 2;
  
  // Draw crosshair
  oled.drawHLine(x + 1, cy, w - 2);
  oled.drawVLine(cx, y + 1, h - 2);
  
  // Draw axis labels
  oled.setFont(u8g2_font_4x6_tf);
  oled.drawStr(cx - 8, y + 6, "Y+");
  oled.drawStr(x + 2, cy - 1, "X-");
  oled.drawStr(cx + 4, cy - 1, "X+");
  
  // Draw arrows
  // Y+ arrow (up)
  oled.drawLine(cx, y + 8, cx - 2, y + 10);
  oled.drawLine(cx, y + 8, cx + 2, y + 10);
  
  // X- arrow (left)
  oled.drawLine(x + 8, cy, x + 10, cy - 2);
  oled.drawLine(x + 8, cy, x + 10, cy + 2);
  
  // X+ arrow (right)
  oled.drawLine(x + w - 8, cy, x + w - 10, cy - 2);
  oled.drawLine(x + w - 8, cy, x + w - 10, cy + 2);
  
  // Y- arrow (down, implied)
  oled.drawLine(cx, y + h - 8, cx - 2, y + h - 10);
  oled.drawLine(cx, y + h - 8, cx + 2, y + h - 10);
  
  // Calculate dot position (normalized to display area)
  int dotX = cx + (int)((xVal / maxVal) * (w / 2 - 4));
  int dotY = cy - (int)((yVal / maxVal) * (h / 2 - 4)); // Invert Y for screen coords
  
  // Draw position dot
  oled.drawDisc(dotX, dotY, 2, U8G2_DRAW_ALL);
}

/******************** FUNCTIONAL BLOCKS ********************/
/**
 * Read left and right joystick Y-axis inputs for Mother control.
 * Stores raw ADC values directly into CmdPacket.leftPower and CmdPacket.rightPower.
 * 
 * Left stick Y (GPIO 39) -> leftPower
 * Right stick Y (GPIO 35) -> rightPower
 */
void readMotherJoysticks() {
  // Read Y-axis from left joystick (GPIO 39, ADC1_CH3)
  int leftY_raw = analogRead(PIN_J1Y);  // Raw ADC: 0-4095 (12-bit)
  
  // Read Y-axis from right joystick (GPIO 35, ADC1_CH7)
  int rightY_raw = analogRead(PIN_J2Y);  // Raw ADC: 0-4095 (12-bit)
  
  // Normalize ADC to [-1.0, +1.0] with deadzone, then map to -1000..+1000
  float leftNorm = adcToNorm(leftY_raw);
  float rightNorm = adcToNorm(rightY_raw);
  
  // Convert normalized values to power range (-1000..+1000)
  cmd.leftPower  = normToPower(leftNorm);
  cmd.rightPower = normToPower(rightNorm);
}

/**
 * Read middle joystick X and Y-axis inputs for Kid control.
 * Stores raw ADC values directly into CmdPacket.kidPower and CmdPacket.kidSteer.
 * 
 * Middle stick Y (GPIO 33, ADC1_CH5) -> kidPower (forward/back)
 * Middle stick X (GPIO 32, ADC1_CH4) -> kidSteer (turn left/right)
 */
void readKidJoysticks() {
  // Read Y-axis from middle joystick (GPIO 33, ADC1_CH5)
  int middleY_raw = analogRead(PIN_J3Y);  // Raw ADC: 0-4095 (12-bit)
  
  // Read X-axis from middle joystick (GPIO 32, ADC1_CH4)
  int middleX_raw = analogRead(PIN_J3X);  // Raw ADC: 0-4095 (12-bit)
  
  // Normalize ADC to [-1.0, +1.0] with deadzone, then map to -1000..+1000
  float middleYNorm = adcToNorm(middleY_raw);
  float middleXNorm = adcToNorm(middleX_raw);
  
  // Convert normalized values to power range (-1000..+1000)
  cmd.kidPower = normToPower(middleYNorm);  // Y-axis for forward/back power
  cmd.kidSteer = normToPower(middleXNorm);  // X-axis for left/right steering
}

/**
 * Read joystick + Z-axis buttons, build command packet.
 * Uses joystick Z-axis clicks for winch control and target switching.
 */
void readInputsBuildCmd() {
  // Read current joystick Z-axis button states
  bool leftZ = digitalRead(PIN_J1Z);   // Left joystick Z
  bool rightZ = digitalRead(PIN_J2Z);  // Right joystick Z
  bool middleZ = digitalRead(PIN_J3Z); // Middle joystick Z
  
  // Winch control: Maintain state while button is held
  // Left Z held (LOW = pressed): winch = +1 (extend)
  // Right Z held (LOW = pressed): winch = -1 (retract)
  // If both held, left takes precedence
  // If neither held: winch = 0 (stop)
  int8_t winch = 0;
  if (leftZ == LOW) {
    // Left Z button is pressed (held)
    winch = +1;
  } else if (rightZ == LOW) {
    // Right Z button is pressed (held)
    winch = -1;
  } else {
    // Neither button pressed, winch stops
    winch = 0;
  }
  
  // Update previous states (still needed for potential future edge detection)
  prevLeftZ = leftZ;
  prevRightZ = rightZ;
  
  // Target toggle: Middle Z button click toggles between Mother/Kid
  // Detect falling edge (HIGH -> LOW) to toggle
  if (prevMiddleZ == HIGH && middleZ == LOW) {
    // Middle Z button just pressed - toggle target
    currentTarget = (currentTarget == TARGET_MOTHER) ? TARGET_KID : TARGET_MOTHER;
  }
  prevMiddleZ = middleZ;
  
  uint8_t target = currentTarget;
  
  // Read joysticks and set fields based on target
  if (target == TARGET_MOTHER) {
    // Mother mode: use left/right joysticks for wheel control
    readMotherJoysticks();
    // Kid controls must be 0
    cmd.kidPower = 0;
    cmd.kidSteer = 0;
  } else {
    // Kid mode: use middle joystick for power/steering
    readKidJoysticks();
    // Mother wheel controls must be 0
    cmd.leftPower = 0;
    cmd.rightPower = 0;
  }
  
  // Update packet header fields
  cmd.seq = cmdSeq++;
  cmd.target = target;
  cmd.winch = winch;
  cmd.flags = 0;  // Flags no longer used for switches
}

/**
 * Initialize nRF24L01 radio module.
 * Configures radio settings, pipes, and starts listening mode.
 * Opens two reading pipes for Mother and Kid telemetry.
 */
bool radioInit() {
  // Initialize radio
  if (!radio.begin()) {
    Serial.println("[Radio] ERROR: Radio hardware not responding!");
    return false;
  }

  // Configure radio settings
  radio.setDataRate(RADIO_DATA_RATE);      // 250KBPS for stability
  radio.setChannel(RADIO_CHANNEL);         // Channel 90
  radio.setPALevel(RADIO_PA_LEVEL);        // PA level (LOW for short range)
  
  // Enable auto-acknowledge for reliability
  radio.setAutoAck(true);
  radio.setAutoAck(0, true);  // Enable auto-ack on pipe 0
  radio.setAutoAck(1, true);  // Enable auto-ack on pipe 1
  
  // Set retry configuration
  radio.setRetries(5, 15);  // 5 retries with 15*250us delay = ~3.75ms max delay
  
  // Handle different TX/RX packet sizes
  // CmdPacket (TX) and TelPacket (RX) have different sizes
  // Use fixed payload size set to largest packet (TelPacket)
  // This ensures both TX and RX can handle the maximum packet size
  radio.setPayloadSize(sizeof(TelPacket));  // Fixed payload size (accommodates both packets)
  
  // Open reading pipes (for receiving TEL from both Mother and Kid)
  radio.openReadingPipe(1, ADDR_M2C);  // Pipe 1: Mother -> Controller (TEL)
  radio.openReadingPipe(2, ADDR_K2C);  // Pipe 2: Kid -> Controller (TEL)
  
  // Open writing pipe initially to Mother (will switch dynamically in radioSendCmd)
  radio.openWritingPipe(ADDR_C2M);  // Controller -> Mother (CMD) - initial
  
  // Start in listening mode (RX)
  radio.startListening();
  
  // Print radio configuration
  Serial.println("[Radio] Initialized successfully");
  Serial.print("[Radio] Channel: ");
  Serial.println(RADIO_CHANNEL);
  Serial.print("[Radio] Data Rate: ");
  Serial.println(RADIO_DATA_RATE == RF24_250KBPS ? "250KBPS" : "1MBPS");
  Serial.print("[Radio] PA Level: ");
  Serial.println(RADIO_PA_LEVEL);
  Serial.print("[Radio] TX Address (Mother): ");
  for (int i = 0; i < 5; i++) {
    Serial.print((char)ADDR_C2M[i]);
  }
  Serial.println();
  Serial.print("[Radio] TX Address (Kid): ");
  for (int i = 0; i < 5; i++) {
    Serial.print((char)ADDR_C2K[i]);
  }
  Serial.println();
  Serial.print("[Radio] RX Pipe 1 (Mother): ");
  for (int i = 0; i < 5; i++) {
    Serial.print((char)ADDR_M2C[i]);
  }
  Serial.println();
  Serial.print("[Radio] RX Pipe 2 (Kid): ");
  for (int i = 0; i < 5; i++) {
    Serial.print((char)ADDR_K2C[i]);
  }
  Serial.println();
  
  return true;
}

/**
 * Send CMD packet to selected target (Mother or Kid).
 * Switches writing pipe dynamically based on cmd.target.
 * Returns true if transmission was successful.
 */
bool radioSendCmd() {
  // Stop listening to enable TX
  radio.stopListening();
  
  // Switch writing pipe based on target (Plan A: send only to selected target)
  if (cmd.target == TARGET_MOTHER) {
    radio.openWritingPipe(ADDR_C2M);  // Controller -> Mother (CMD)
  } else {
    radio.openWritingPipe(ADDR_C2K);  // Controller -> Kid (CMD)
  }
  
  // Send packet
  bool success = radio.write(&cmd, sizeof(cmd));
  
  // Track TX success/fail for debug
  if (success) {
    cmdTxSuccessCount++;
  } else {
    cmdTxFailCount++;
  }
  
  // Return to listening mode
  radio.startListening();
  
  return success;
}

/**
 * Receive telemetry from Mother and/or Kid if available.
 * Handles dual reading pipes and maintains separate telemetry packets.
 * Updates link status separately for Mother and Kid.
 * 
 * Note: Uses RF24 library's available(&pipeNum) to identify source pipe.
 * If not supported, falls back to reading and identifying by other means.
 */
void radioReceiveTel() {
  uint32_t now = millis();
  
  // Poll for available data and identify source pipe
  // TMRh20 RF24 library supports: radio.available(&pipeNum)
  uint8_t pipeNum = 255;  // Initialize to invalid value
  bool hasData = false;
  
  // Try to get pipe number (method depends on RF24 library version)
  // For TMRh20 RF24: available(&pipeNum) returns true and sets pipeNum
  #if defined(RF24_LINUX) || defined(__linux__)
    // Linux version may have different API
    hasData = radio.available();
  #else
    // ESP32/Arduino version - try available(&pipeNum)
    hasData = radio.available(&pipeNum);
  #endif
  
  if (hasData) {
    TelPacket tempTel;
    radio.read(&tempTel, sizeof(tempTel));
    
    // Identify source by pipe number
    if (pipeNum == 1) {
      // Pipe 1: Mother -> Controller TEL
      telMother = tempTel;
      lastTelMotherMs = now;
      linkStableMother = true;
    } else if (pipeNum == 2) {
      // Pipe 2: Kid -> Controller TEL
      telKid = tempTel;
      lastTelKidMs = now;
      linkStableKid = true;
    } else {
      // Unknown pipe - use heuristic based on current target
      // This handles cases where pipeNum is not properly set
      if (cmd.target == TARGET_MOTHER) {
        telMother = tempTel;
        lastTelMotherMs = now;
        linkStableMother = true;
      } else {
        telKid = tempTel;
        lastTelKidMs = now;
        linkStableKid = true;
      }
    }
  }
  
  // Check for timeout on Mother link
  if (lastTelMotherMs > 0) {
    if (now - lastTelMotherMs > TEL_TIMEOUT_MS) {
      linkStableMother = false;
    }
  } else {
    linkStableMother = false;
  }
  
  // Check for timeout on Kid link
  if (lastTelKidMs > 0) {
    if (now - lastTelKidMs > TEL_TIMEOUT_MS) {
      linkStableKid = false;
    }
  } else {
    linkStableKid = false;
  }
}

/**
 * Render control screen (LEFT OLED - matches first image design)
 */
void renderControlScreen() {
  oledLeft.clearBuffer();
  
  // Determine winch state (state-based, not percentage)
  const char* winchState = "STOP";
  bool winchPulling = false;
  if (cmd.winch > 0) {
    winchState = "PULLING";
    winchPulling = true;
  } else if (cmd.winch < 0) {
    winchState = "RELEASE";
  }
  
  // Determine target name
  const char* targetName = (cmd.target == TARGET_KID) ? "CHILD UNIT" : "MOTHER UNIT";
  
  // Determine link status for current target
  bool linkStable = (cmd.target == TARGET_MOTHER) ? linkStableMother : linkStableKid;
  const char* linkStatus = linkStable ? "STABLE" : "LOST";
  const char* linkTarget = (cmd.target == TARGET_KID) ? "CHILD" : "MOTHER";

  char buf[32];

  // Title: CONTROL (TX) with status dot
  oledLeft.setFont(u8g2_font_6x12_tf);
  drawStatusDot(oledLeft, 8, 6, true);  // Status indicator dot
  oledLeft.drawStr(12, 12, "CONTROL (TX)");
  
  // Separator line
  oledLeft.drawHLine(0, 15, 128);

  // Target: MOTHER UNIT / CHILD UNIT
  oledLeft.setFont(u8g2_font_5x8_tf);
  oledLeft.drawStr(0, 25, "Target:");
  oledLeft.setFont(u8g2_font_6x10_tf);
  oledLeft.drawStr(40, 25, targetName);

  // Separator
  oledLeft.drawHLine(0, 28, 128);

  // Display controls based on target mode
  if (cmd.target == TARGET_MOTHER) {
    // Mother mode: Show Left/Right Power
    int leftPct = powerToPct(cmd.leftPower);
    int rightPct = powerToPct(cmd.rightPower);
    
    // Left Power: XX% with progress bar to the right
    oledLeft.setFont(u8g2_font_5x8_tf);
    oledLeft.drawStr(0, 38, "Left Power :");
    snprintf(buf, sizeof(buf), "%d%%", leftPct);
    oledLeft.setFont(u8g2_font_6x10_tf);
    int leftPctX = 70;
    oledLeft.drawStr(leftPctX, 38, buf);

    // Progress bar positioned to the right of percentage, extending to edge
    int barX = leftPctX + 28;  // Space after percentage
    int barW = 128 - barX - 1;
    drawProgressBar(oledLeft, barX, 35, barW, 5, leftPct);

    // Right Power: XX% with progress bar to the right
    oledLeft.setFont(u8g2_font_5x8_tf);
    oledLeft.drawStr(0, 48, "Right Power :");
    snprintf(buf, sizeof(buf), "%d%%", rightPct);
    oledLeft.setFont(u8g2_font_6x10_tf);
    int rightPctX = 70;
    oledLeft.drawStr(rightPctX, 48, buf);
    
    // Progress bar positioned to the right of percentage, extending to edge
    drawProgressBar(oledLeft, barX, 45, barW, 5, rightPct);
  } else {
    // Kid mode: Show Kid Power/Kid Steer
    int kidPowerPct = powerToPct(cmd.kidPower);
    int kidSteerPct = powerToPct(cmd.kidSteer);
    
    // Kid Power: XX% with progress bar to the right
    oledLeft.setFont(u8g2_font_5x8_tf);
    oledLeft.drawStr(0, 38, "Kid Power :");
    snprintf(buf, sizeof(buf), "%d%%", kidPowerPct);
    oledLeft.setFont(u8g2_font_6x10_tf);
    int kidPowerX = 70;
    oledLeft.drawStr(kidPowerX, 38, buf);

    // Progress bar positioned to the right of percentage, extending to edge
    int barX = kidPowerX + 28;  // Space after percentage
    int barW = 128 - barX - 1;
    drawProgressBar(oledLeft, barX, 35, barW, 5, kidPowerPct);

    // Kid Steer: XX% with progress bar to the right
    oledLeft.setFont(u8g2_font_5x8_tf);
    oledLeft.drawStr(0, 48, "Kid Steer :");
    snprintf(buf, sizeof(buf), "%d%%", kidSteerPct);
    oledLeft.setFont(u8g2_font_6x10_tf);
    int kidSteerX = 70;
    oledLeft.drawStr(kidSteerX, 48, buf);
    
    // Progress bar positioned to the right of percentage, extending to edge
    drawProgressBar(oledLeft, barX, 45, barW, 5, kidSteerPct);
  }

  // Separator
  oledLeft.drawHLine(0, 52, 128);

  // Bottom section: List items vertically (no overlaps)
  // Winch Status: PULLING/RELEASE/STOP with arrow
  oledLeft.setFont(u8g2_font_5x8_tf);
  oledLeft.drawStr(0, 56, "Winch:");
  oledLeft.setFont(u8g2_font_6x10_tf);
  int winchX = 40;
  oledLeft.drawStr(winchX, 56, winchState);
  if (winchPulling) {
    drawUpArrow(oledLeft, winchX + 50, 54);
  }
  
  // LINK status (below winch, vertically arranged)
  oledLeft.setFont(u8g2_font_5x8_tf);
  snprintf(buf, sizeof(buf), "LINK: %s -> %s", linkStatus, linkTarget);
  oledLeft.drawStr(0, 64, buf);

  oledLeft.sendBuffer();
}

/**
 * Render telemetry screen (RIGHT OLED - matches second image design)
 */
void renderTelemetryScreen() {
  oledRight.clearBuffer();

  // Select telemetry based on current target
  TelPacket* tel = (cmd.target == TARGET_MOTHER) ? &telMother : &telKid;
  
  // Get data from tel packet
  float speedKmh = tel->speedM;  // Use speedM from telemetry
  if (speedKmh < 0) speedKmh = 0;  // Clamp to 0 minimum
  
  // Use IMU data from telemetry
  float ax = tel->imuM.ax;
  float ay = tel->imuM.ay;
  float az = tel->imuM.az;
  float roll = tel->imuM.roll;
  float pitch = tel->imuM.pitch;
  float yaw = tel->imuM.yaw;
  
  // Get Mother distance and Kid online status
  float distM = tel->distM;
  bool kidOnline = linkStableKid;  // Use link status instead of linkFlags

  char buf[32];

  // Title: TELEMETRY (RX) with green status dot
  oledRight.setFont(u8g2_font_6x12_tf);
  drawStatusDot(oledRight, 8, 6, true);  // Green status dot
  oledRight.drawStr(12, 12, "TELEMETRY (RX)");
  
  // Separator line
  oledRight.drawHLine(0, 15, 128);

  // Top section: Split into two columns
  // Left: Speed Gauge
  oledRight.setFont(u8g2_font_5x8_tf);
  oledRight.drawStr(2, 20, "SPEED GAUGE");
  drawSpeedGauge(oledRight, 0, 22, 64, 18, speedKmh, 60.0f);

  // Right: XYZ Motion
  oledRight.setFont(u8g2_font_5x8_tf);
  oledRight.drawStr(66, 20, "XYZ MOTION");
  // Use acceleration values for XYZ motion visualization
  float maxAccel = 4.0f;  // Max acceleration for scaling
  drawXYZMotion(oledRight, 64, 22, 64, 18, ax, ay, maxAccel);

  // Separator
  oledRight.drawHLine(0, 40, 128);

  // Bottom section: Acceleration and Angle values (vertically arranged, no overlaps)
  // Acceleration section (left) - list vertically
  oledRight.setFont(u8g2_font_5x8_tf);
  oledRight.drawStr(0, 48, "Accel (m/s²)");
  oledRight.setFont(u8g2_font_4x6_tf);
  snprintf(buf, sizeof(buf), "X:%+.1f", ax);
  oledRight.drawStr(0, 54, buf);
  snprintf(buf, sizeof(buf), "Y:%+.1f", ay);
  oledRight.drawStr(0, 60, buf);
  snprintf(buf, sizeof(buf), "Z:%+.1f", az);
  oledRight.drawStr(0, 63, buf);

  // Angle section (right) - list vertically, avoid overlap with Z
  oledRight.setFont(u8g2_font_5x8_tf);
  oledRight.drawStr(64, 48, "Angle (deg)");
  oledRight.setFont(u8g2_font_4x6_tf);
  snprintf(buf, sizeof(buf), "Roll:%+.1f", roll);
  oledRight.drawStr(64, 54, buf);
  snprintf(buf, sizeof(buf), "Pitch:%+.1f", pitch);
  oledRight.drawStr(64, 60, buf);
  snprintf(buf, sizeof(buf), "Yaw:%+.1f", yaw);
  oledRight.drawStr(64, 63, buf);

  // Additional info: Mother distance and Kid online status
  // Add a small status line at the very bottom if space allows
  // For now, we'll add it near the header or as a compact line
  // Since space is limited, add distance in header area and Kid status near link info
  oledRight.setFont(u8g2_font_4x6_tf);
  snprintf(buf, sizeof(buf), "Dist:%.1fm", distM);
  oledRight.drawStr(90, 12, buf);
  
  // Kid online indicator (small dot or text)
  if (kidOnline) {
    drawStatusDot(oledRight, 120, 6, true);  // Green dot if online
  } else {
    drawStatusDot(oledRight, 120, 6, false); // Empty circle if offline
  }

  oledRight.sendBuffer();
}

/**
 * Update OLED UI (both screens).
 * Note: linkStable is updated in radioReceiveTel() based on timeout and linkFlags.
 */
void uiUpdate() {
  // Render both screens
  renderControlScreen();   // Left OLED - Control screen
  renderTelemetryScreen(); // Right OLED - Telemetry screen
}

/******************** SETUP ********************/
void setup() {
  Serial.begin(115200);

  // Joystick Z-axis buttons (configured as inputs with pull-up)
  pinMode(PIN_J1Z, INPUT_PULLUP);  // Left joystick Z
  pinMode(PIN_J2Z, INPUT_PULLUP);  // Right joystick Z
  pinMode(PIN_J3Z, INPUT_PULLUP);  // Middle joystick Z

  // ADC settings (optional)
  analogReadResolution(12); // ESP32 default 12-bit
  // analogSetAttenuation(ADC_11db); // optional for full range

  // SPI init for nRF
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  // nRF24 radio init
  if (!radioInit()) {
    Serial.println("[Controller] WARNING: Radio initialization failed!");
    Serial.println("[Controller] Continuing without radio - link will show as lost");
  }

  // OLED init (dual OLED setup)
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  
  // Left OLED (Control screen) - address 0x3C
  oledLeft.setI2CAddress(0x3C << 1);  // 7-bit address shifted for U8g2
  oledLeft.begin();
  oledLeft.setContrast(255);
  
  // Right OLED (Telemetry screen) - address 0x3D
  oledRight.setI2CAddress(0x3D << 1);  // 7-bit address shifted for U8g2
  oledRight.begin();
  oledRight.setContrast(255);
  
  // Initial display
  renderControlScreen();
  renderTelemetryScreen();

  Serial.println("[Controller] Ready");
  Serial.println("[Controller] Direct star topology");
  Serial.println("[Controller] Sends CMD only to selected target");
  Serial.println("[Controller] Receives TEL from both Mother and Kid");
}

/******************** LOOP ********************/
void loop() {
  uint32_t now = millis();

  // 1) Always listen for telemetry
  radioReceiveTel();

  // 2) Periodic CMD send
  if (now - lastCmdSendMs >= CMD_PERIOD_MS) {
    lastCmdSendMs = now;
    readInputsBuildCmd();
    radioSendCmd();
  }

  // 3) Periodic UI refresh
  if (now - lastUiMs >= UI_PERIOD_MS) {
    lastUiMs = now;
    uiUpdate();  // Updates OLED display with current cmd and tel state
  }
}
