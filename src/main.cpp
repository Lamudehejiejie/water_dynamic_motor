// ============================================================================
// LIBRARIES
// ============================================================================
#include <M5Unified.h>           // M5Stack hardware library
#include <Dynamixel2Arduino.h>   // ROBOTIS library for Dynamixel motor control
#include <UNIT_8ENCODER.h>       // M5Stack 8-Encoder Unit library

// ============================================================================
// COMMUNICATION SETUP - RS485 via DMX Base
// ============================================================================
// You are using RS485 communication, NOT plain serial!
// - M5Stack CoreS3 speaks TTL serial (3.3V logic)
// - DMX Base converts TTL → RS485 (differential signaling A/B)
// - Dynamixel motor speaks RS485
//
// The flow: M5Stack (TTL) → DMX Base (TTL→RS485) → Motor (RS485)
//
// M5Stack DMX Base Pin Configuration for CoreS3:
#define DXL_SERIAL Serial1       // Use Serial1 hardware UART
#define TXD_PIN 7                // GPIO 7 = TX (transmit data to DMX Base)
#define RXD_PIN 10               // GPIO 10 = RX (receive data from DMX Base)
#define DXL_DIR_PIN 6            // GPIO 6 = EN (direction control for RS485)
                                 // EN pin controls TX/RX mode on RS485 chip

// ============================================================================
// DYNAMIXEL MOTOR SETTINGS
// ============================================================================
const uint8_t DXL_ID = 1;                    // Motor ID (set in motor, default 1)
const float DXL_PROTOCOL_VERSION = 2.0;      // Protocol 2.0 for XL430
const uint32_t DXL_BAUD_RATE = 1000000;      // Communication speed (bits/second)
                                             // RS485 supports: 9600, 57600, 115200, 1000000, etc.
                                             // 1000000 = 1 Mbps (modern Dynamixel default)
                                             // Your motor is configured to 1000000 baud

// Create Dynamixel controller object
// This handles all the complex packet formatting and RS485 communication
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ============================================================================
// ENCODER SETUP - M5Stack 8-Encoder Unit
// ============================================================================
#define ENCODER_I2C_ADDR 0x41    // I2C address for 8-Encoder Unit
UNIT_8ENCODER encoder;           // Create encoder object

// ============================================================================
// MOTION CONTROL VARIABLES
// ============================================================================
bool encoder_found = false;        // Flag: encoder detected and working?
const int MAX_POSITION = 4095;     // XL430 position range: 0-4095 (0-360 degrees)
                                   // 0 = 0°, 2048 = 180°, 4095 = 360°
const int ENCODER_COUNTS_PER_REV = 60;  // Encoder counts per full rotation

// ============================================================================
// SETUP - Runs once when M5Stack starts
// ============================================================================
void setup() {
    // Initialize M5Stack hardware (display, buttons, sensors)
    auto cfg = M5.config();
    M5.begin(cfg);

    // Setup display
    M5.Display.clear();
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 10);
    M5.Display.println("Motor + Encoder Control");

    // ========================================================================
    // INITIALIZE I2C FOR ENCODER
    // ========================================================================
    // M5Stack CoreS3 Port.A: SDA=GPIO2, SCL=GPIO1
    Wire.begin(2, 1);  // Initialize I2C with SDA=GPIO2, SCL=GPIO1

    // Initialize 8-Encoder Unit
    if (encoder.begin(&Wire, ENCODER_I2C_ADDR, 2, 1)) {
        M5.Display.setCursor(10, 20);
        M5.Display.println("Encoder found!");

        // Reset Channel 1 encoder to 0 at startup
        encoder.resetCounter(0);  // Reset channel 0 (= Channel 1)

        encoder_found = true;
    } else {
        M5.Display.setCursor(10, 20);
        M5.Display.println("Encoder NOT found - HALT");
        encoder_found = false;
        while(1) { delay(100); }  // Stop here if no encoder
    }

    // ========================================================================
    // INITIALIZE RS485 COMMUNICATION
    // ========================================================================
    // Configure Serial1 UART with:
    // - Baud rate: 1000000 (must match motor's baud rate setting)
    // - SERIAL_8N1: 8 data bits, No parity, 1 stop bit (standard)
    // - RX pin: GPIO 10, TX pin: GPIO 7
    Serial1.begin(DXL_BAUD_RATE, SERIAL_8N1, RXD_PIN, TXD_PIN);

    // Initialize Dynamixel library
    // This sets up packet protocol and RS485 direction control (EN pin)
    dxl.begin(DXL_BAUD_RATE);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    M5.Display.setCursor(10, 30);
    M5.Display.println("Pinging motor...");

    // ========================================================================
    // FIND AND CONFIGURE MOTOR
    // ========================================================================
    // Send a "ping" packet to motor ID 1 via RS485
    // If motor responds, it's connected and communicating correctly
    if(dxl.ping(DXL_ID)) {
        M5.Display.setCursor(10, 50);
        M5.Display.println("Motor FOUND!");

        // Configure motor for EXTENDED position control mode (multi-turn)
        dxl.torqueOff(DXL_ID);                        // Turn off torque (motor can move freely)
        dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION);  // Extended position = unlimited rotations

        // Set motion profile for REAL-TIME encoder control
        // Profile Velocity: 0 = maximum speed (instant response)
        // Profile Acceleration: 0 = maximum acceleration (no smoothing)
        dxl.setProfileVelocity(DXL_ID, 0);
        dxl.setProfileAcceleration(DXL_ID, 0);

        dxl.torqueOn(DXL_ID);                         // Turn on torque (motor locked to position)


        M5.Display.setCursor(10, 70);
        M5.Display.println("Torque ON");
    } else {
        M5.Display.setCursor(10, 50);
        M5.Display.println("Motor NOT found!");
        // Check: wiring, power, baud rate, motor ID
    }

    delay(1000);  // Wait 1 second before starting loop
}

// ============================================================================
// LOOP - Runs continuously (many times per second)
// ============================================================================
void loop() {
    M5.update();  // Update M5Stack internal state (buttons, etc.)

    // ========================================================================
    // READ ENCODER VALUES
    // ========================================================================
    // Read switch status (returns true/false for switch state)
    bool switch_status = encoder.getSwitchStatus();

    // Read Channel 1 encoder value (incremental counter)
    int32_t encoder_value = encoder.getEncoderValue(0);  // Channel 0 = Channel 1

    // ========================================================================
    // MAP ENCODER TO MOTOR POSITION (EXTENDED MODE - MULTI-TURN)
    // ========================================================================
    // Scale encoder value to motor position
    // Encoder: 60 counts per rotation
    // Motor: 4095 positions per rotation (0-360°)
    // Extended mode: Motor can go beyond 0-4095 (unlimited rotations)
    // Scale factor: 4095 / 60 = 68.25

    int position = (-encoder_value * MAX_POSITION) / ENCODER_COUNTS_PER_REV;
    // -encoder is counter-clockwise, +encoder is clockwise.

    // NO WRAPPING - let position go beyond 0-4095 for continuous rotation
    // Extended position mode handles multi-turn automatically

    // Send position to motor (ALWAYS - removed switch check)
    if(!switch_status){
    dxl.setGoalPosition(DXL_ID, position);
    }
    // ========================================================================
    // READ CURRENT MOTOR POSITION
    // ========================================================================
    int32_t present_position = dxl.getPresentPosition(DXL_ID);
    // ========================================================================
    // DISPLAY DEBUG INFO
    // ========================================================================
    M5.Display.setCursor(10, 100);
    M5.Display.printf("Switch: %s    ", switch_status ? "ON" : "OFF");
    M5.Display.setCursor(10, 120);
    M5.Display.printf("Enc CH1: %ld    ", encoder_value);
    M5.Display.setCursor(10, 140);
    M5.Display.printf("Enc->Pos: %d    ", position);
    M5.Display.setCursor(10, 160);
    M5.Display.printf("Motor: %d    ", present_position);
    // M5.Display.setCursor(10, 180);
    // M5.Display.printf("Error: %d    ", position - present_position);

    delay(1000/120.);  // Update 20 times per second
}
