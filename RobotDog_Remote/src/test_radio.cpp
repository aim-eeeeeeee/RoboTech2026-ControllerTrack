/*
 * NRF24L01+ Test Program for Mother Robot
 * 
 * This program tests the NRF24L01+ module independently
 * Use this to verify radio hardware before integrating with robot control
 * 
 * Expected behavior:
 * - On startup, should print "Radio OK"
 * - When receiving commands, prints command values
 * - Sends test telemetry every second
 */

#include <Arduino.h>
#include <SPI.h>
#include "radio_protocol.h"

#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);

CmdPacket cmdPacket;
TelPacket telPacket;
uint8_t telSeq = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n=== NRF24L01+ Module Test ===");
    Serial.println("Testing Mother Robot Radio...");
    
    // Initialize radio
    if (!radio.begin()) {
        Serial.println("ERROR: Radio NOT responding!");
        Serial.println("Check:");
        Serial.println("  - 3.3V power (NOT 5V!)");
        Serial.println("  - All SPI connections");
        Serial.println("  - CE pin 9, CSN pin 10");
        while (1) delay(1000);
    }
    
    Serial.println("Radio hardware OK!");
    
    // Configure radio
    radio.setPALevel(RADIO_PA_LEVEL);
    radio.setDataRate(RADIO_DATA_RATE);
    radio.setChannel(RADIO_CHANNEL);
    radio.setAutoAck(true);
    radio.enableDynamicPayloads();
    
    // Set up pipes
    radio.openReadingPipe(0, ADDR_C2M);
    radio.openWritingPipe(ADDR_M2C);
    
    radio.startListening();
    
    Serial.println("Radio configured:");
    Serial.print("  Channel: ");
    Serial.println(RADIO_CHANNEL);
    Serial.print("  Data Rate: ");
    Serial.println(RADIO_DATA_RATE == RF24_250KBPS ? "250kbps" : "1Mbps");
    Serial.print("  Power: ");
    Serial.println(RADIO_PA_LEVEL);
    Serial.println("\nWaiting for commands from controller...");
    Serial.println("----------------------------------------");
}

void loop() {
    static unsigned long lastPrint = 0;
    static unsigned long lastTel = 0;
    static int cmdCount = 0;
    unsigned long now = millis();
    
    // Check for incoming commands
    if (radio.available()) {
        radio.read(&cmdPacket, sizeof(cmdPacket));
        cmdCount++;
        
        Serial.println("\n>>> Command Received <<<");
        Serial.print("  Seq: ");
        Serial.println(cmdPacket.seq);
        Serial.print("  Target: ");
        Serial.println(cmdPacket.target == TARGET_MOTHER ? "MOTHER" : "KID");
        Serial.print("  Left Power: ");
        Serial.println(cmdPacket.leftPower);
        Serial.print("  Right Power: ");
        Serial.println(cmdPacket.rightPower);
        Serial.print("  Winch: ");
        Serial.println(cmdPacket.winch);
        Serial.print("  Total commands: ");
        Serial.println(cmdCount);
        
        lastPrint = now;
    }
    
    // Send test telemetry every second
    if (now - lastTel > 1000) {
        lastTel = now;
        
        radio.stopListening();
        
        // Fill test data
        telPacket.seq = telSeq++;
        telPacket.linkFlags = 0x00;
        telPacket.imuM.roll = 180.0 + sin(now / 1000.0) * 10.0;
        telPacket.imuM.pitch = sin(now / 1500.0) * 5.0;
        telPacket.imuM.yaw = 0.0;
        telPacket.imuM.ax = 0.0;
        telPacket.imuM.ay = 0.0;
        telPacket.imuM.az = 0.0;
        telPacket.distM = 0.0;
        telPacket.speedM = 0.0;
        telPacket.imuK.roll = 0.0;
        telPacket.imuK.pitch = 0.0;
        telPacket.imuK.yaw = 0.0;
        telPacket.imuK.ax = 0.0;
        telPacket.imuK.ay = 0.0;
        telPacket.imuK.az = 0.0;
        telPacket.distK = 0.0;
        
        bool sent = radio.write(&telPacket, sizeof(telPacket));
        
        Serial.print("\n[TEL] Seq=");
        Serial.print(telPacket.seq);
        Serial.print(" Roll=");
        Serial.print(telPacket.imuM.roll);
        Serial.print(" Pitch=");
        Serial.print(telPacket.imuM.pitch);
        Serial.print(" Sent=");
        Serial.println(sent ? "OK" : "FAIL");
        
        radio.startListening();
    }
    
    // Print "still waiting" message every 5 seconds if no commands
    if (cmdCount == 0 && (now - lastPrint > 5000)) {
        lastPrint = now;
        Serial.println("Still waiting... (Make sure controller is on and sending)");
    }
}
