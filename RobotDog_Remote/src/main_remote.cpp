#include <Arduino.h>
#include <SPI.h>
#include "RobotDog.h"
#include "radio_protocol.h"

// ================= NRF24L01+ CONFIGURATION =================
// Arduino UNO R4 Minima SPI pins:
// MOSI: 11
// MISO: 12
// SCK: 13
// CE: 9 (can be changed)
// CSN: 10 (can be changed)

#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);

// ================= GLOBAL OBJECTS =================
RobotDog dog;

// ================= RADIO STATE =================
CmdPacket cmdPacket;
TelPacket telPacket;
unsigned long lastCmdTime = 0;
uint8_t telSeq = 0;

// ================= TELEMETRY TIMING =================
const uint32_t TEL_INTERVAL_MS = 50; // Send telemetry every 50ms (20Hz)
unsigned long lastTelTime = 0;

// ================= FAILSAFE STATE =================
bool failsafeActive = false;

void setup() {
    // Pin setup for Roll function (keep existing functionality)
    for(int i = 2; i <= 7; i++) {
        pinMode(i, OUTPUT);
    }
    digitalWrite(5, LOW);
    digitalWrite(7, LOW);
    
    // Serial for debugging
    Serial.begin(115200);
    delay(100);
    Serial.println("=== Mother Robot Dog - Remote Control Version ===");
    
    // ===== Initialize Radio =====
    Serial.println("Initializing NRF24L01+...");
    
    if (!radio.begin()) {
        Serial.println("ERROR: Radio hardware not responding!");
        while (1) {
            delay(1000);
            Serial.println("Radio init failed. Check wiring!");
        }
    }
    
    // Configure radio
    radio.setPALevel(RADIO_PA_LEVEL);
    radio.setDataRate(RADIO_DATA_RATE);
    radio.setChannel(RADIO_CHANNEL);
    radio.setAutoAck(true);
    radio.enableDynamicPayloads();
    
    // Set up pipes for Mother robot
    // RX Pipe 0: Receive commands from Controller (C2M)
    // TX: Send telemetry to Controller (M2C)
    radio.openReadingPipe(0, ADDR_C2M);
    radio.openWritingPipe(ADDR_M2C);
    
    radio.startListening();
    
    Serial.println("Radio initialized successfully!");
    Serial.print("Listening on channel: ");
    Serial.println(RADIO_CHANNEL);
    
    // ===== Initialize Robot Dog =====
    Serial.println("Initializing Robot Dog...");
    dog.begin();
    dog.setControl(true, 0, 0, 0, 0, 0); // Start in stand position
    delay(2000);
    
    Serial.println("=== Setup Complete - Ready for Remote Control ===");
    Serial.println("Waiting for controller commands...");
}

void loop() {
    unsigned long now = millis();
    
    // ===== Check for incoming commands =====
    if (radio.available()) {
        radio.read(&cmdPacket, sizeof(cmdPacket));
        
        // Check if command is for Mother robot
        if (cmdPacket.target == TARGET_MOTHER) {
            lastCmdTime = now;
            
            // Clear failsafe if it was active
            if (failsafeActive) {
                failsafeActive = false;
                Serial.println("Connection restored!");
            }
            
            // Process command
            // leftPower and rightPower are -1000 to 1000
            // For differential drive, we can map this to direction and turn
            
            int16_t forward = (cmdPacket.leftPower + cmdPacket.rightPower) / 2;
            int16_t turn = (cmdPacket.rightPower - cmdPacket.leftPower) / 2;
            
            // Map to RobotDog control
            // direction: 1 (forward), -1 (backward), 0 (stop)
            int direction = 0;
            if (abs(forward) > 100) { // Dead zone
                direction = (forward > 0) ? 1 : -1;
            }
            
            // For now, we keep the dog walking straight
            // You can add turning logic by modifying individual leg heights
            // or by adding turn-specific gait patterns
            
            bool force_stand = (direction == 0); // Stand still if no movement
            
            // Simple turning implementation using leg height adjustment
            // Positive turn = turn right, negative = turn left
            int h_lf = 0, h_lb = 0, h_rf = 0, h_rb = 0;
            if (abs(turn) > 100) {
                if (turn > 0) { // Turn right - lift left side slightly
                    h_lf = 1;
                    h_lb = 1;
                } else { // Turn left - lift right side slightly
                    h_rf = 1;
                    h_rb = 1;
                }
            }
            
            dog.setControl(force_stand, direction, h_lf, h_lb, h_rf, h_rb);
            
            // Optional: Print received command for debugging
            static unsigned long lastPrint = 0;
            if (now - lastPrint > 500) {
                lastPrint = now;
                Serial.print("CMD: L=");
                Serial.print(cmdPacket.leftPower);
                Serial.print(" R=");
                Serial.print(cmdPacket.rightPower);
                Serial.print(" Dir=");
                Serial.println(direction);
            }
        }
    }
    
    // ===== Failsafe Check =====
    if ((now - lastCmdTime > FAILSAFE_MS) && !failsafeActive) {
        failsafeActive = true;
        Serial.println("FAILSAFE: No commands received - stopping robot");
        dog.setControl(true, 0, 0, 0, 0, 0); // Force stand still
    }
    
    // ===== Run Robot Dog Control Loop =====
    dog.run();
    
    // ===== Send Telemetry =====
    if (now - lastTelTime >= TEL_INTERVAL_MS) {
        lastTelTime = now;
        
        // Stop listening to send telemetry
        radio.stopListening();
        
        // Prepare telemetry packet
        telPacket.seq = telSeq++;
        telPacket.linkFlags = failsafeActive ? 0x02 : 0x00; // bit1 = controller timeout
        
        // Get IMU data from gyro
        float roll, pitch, yaw;
        getAngles(roll, pitch, yaw);
        
        telPacket.imuM.roll = roll;
        telPacket.imuM.pitch = pitch;
        telPacket.imuM.yaw = yaw;
        telPacket.imuM.ax = 0.0; // Accelerometer data not used in current implementation
        telPacket.imuM.ay = 0.0;
        telPacket.imuM.az = 0.0;
        
        // Distance and speed (optional - set to 0 for now)
        telPacket.distM = 0.0;
        telPacket.speedM = 0.0;
        
        // Kid robot data (not applicable for Mother)
        telPacket.imuK.roll = 0.0;
        telPacket.imuK.pitch = 0.0;
        telPacket.imuK.yaw = 0.0;
        telPacket.imuK.ax = 0.0;
        telPacket.imuK.ay = 0.0;
        telPacket.imuK.az = 0.0;
        telPacket.distK = 0.0;
        
        // Send telemetry
        bool sent = radio.write(&telPacket, sizeof(telPacket));
        
        // Optional: Print telemetry status
        static unsigned long lastTelPrint = 0;
        if (now - lastTelPrint > 1000) {
            lastTelPrint = now;
            Serial.print("TEL sent=");
            Serial.print(sent ? "OK" : "FAIL");
            Serial.print(" R=");
            Serial.print(roll);
            Serial.print(" P=");
            Serial.println(pitch);
        }
        
        // Resume listening for commands
        radio.startListening();
    }
}
