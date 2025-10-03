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

// Manual mode control variables
int current_velocity = 0;          // Track current velocity setting
int current_acceleration = 0;      // Track current acceleration setting
int direction_multiplier = 1;      // Direction: 1 = FWD, -1 = REV

// Auto mode control variables
unsigned long cycle_start_time = 0;     // When current cycle started
bool auto_mode_active = false;          // Track if we're in auto mode

// Auto mode parameters (controllable via encoders)
int auto_cycle_time = 2000;             // Total cycle time (ms) - CH1
int auto_push_velocity = 1200;          // Fast push velocity (500-2000) - CH2
int auto_push_acceleration = 2000;      // Fast push acceleration (100-5000) - CH3
int auto_return_velocity = 100;         // Slow return velocity (50-500) - CH4
int auto_return_acceleration = 100;     // Slow return acceleration (10-1000) - CH5
int auto_push_ratio = 50;               // Push time as % of cycle (10-90%) - CH6
int auto_direction_swap = 1;            // Direction multiplier: 1=normal, -1=swapped - CH7
int auto_stroke_range = 1024;           // Stroke range in units (±90°) - CH8

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

        // Set initial motion profile
        // We'll update these in loop() based on encoder values
        current_velocity = 300;          // Initial velocity
        current_acceleration = 100;      // Initial acceleration
        dxl.setProfileVelocity(DXL_ID, current_velocity);
        dxl.setProfileAcceleration(DXL_ID, current_acceleration);

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

    // Read all encoder values and buttons
    int32_t encoder_ch1_value = encoder.getEncoderValue(0);
    int32_t encoder_ch2_value = encoder.getEncoderValue(1);
    int32_t encoder_ch3_value = encoder.getEncoderValue(2);
    int32_t encoder_ch4_value = encoder.getEncoderValue(3);
    int32_t encoder_ch5_value = encoder.getEncoderValue(4);
    int32_t encoder_ch6_value = encoder.getEncoderValue(5);
    int32_t encoder_ch7_value = encoder.getEncoderValue(6);
    int32_t encoder_ch8_value = encoder.getEncoderValue(7);

    bool encoder_ch1_button = encoder.getButtonStatus(0);
    bool encoder_ch2_button = encoder.getButtonStatus(1);
    bool encoder_ch3_button = encoder.getButtonStatus(2);
    bool encoder_ch4_button = encoder.getButtonStatus(3);
    bool encoder_ch5_button = encoder.getButtonStatus(4);
    bool encoder_ch6_button = encoder.getButtonStatus(5);
    bool encoder_ch7_button = encoder.getButtonStatus(6);
    bool encoder_ch8_button = encoder.getButtonStatus(7);

    // ========================================================================
    // MODE SELECTION: MANUAL (SWITCH OFF) vs AUTO (SWITCH ON)
    // ========================================================================
    if(!switch_status){
        // ====================================================================
        // MANUAL MODE
        // ====================================================================

        // Reset auto mode flag when entering manual mode
        if (auto_mode_active) {
            auto_mode_active = false;
            M5.Display.clear();  // Clear display when switching modes

            // Home to position 0 when entering manual mode
            dxl.setProfileVelocity(DXL_ID, 500);      // Medium speed to home
            dxl.setProfileAcceleration(DXL_ID, 500);  // Medium acceleration
            dxl.setGoalPosition(DXL_ID, 0);           // Move to position 0

            delay(1000);  // Wait for motor to reach position 0

            // Reset encoder CH1 to 0 so it matches motor position
            encoder.resetCounter(0);
        }

        // ----------------------------------------------------------------
        // ENCODER CH4: DIRECTION TOGGLE (BUTTON)
        // ----------------------------------------------------------------
        static bool last_ch4_button = false;
        if (encoder_ch4_button && !last_ch4_button) {  // Button press detected
            direction_multiplier *= -1;  // Toggle between 1 and -1
        }
        last_ch4_button = encoder_ch4_button;

        // ----------------------------------------------------------------
        // ENCODER CH1: POSITION CONTROL
        // ----------------------------------------------------------------
        int position = (-encoder_ch1_value * MAX_POSITION * direction_multiplier) / ENCODER_COUNTS_PER_REV;

        // ----------------------------------------------------------------
        // ENCODER CH2: VELOCITY CONTROL
        // ----------------------------------------------------------------
        // Range: 0-2000 (covers full motor speed range ~0-458 RPM)
        int new_velocity = abs(encoder_ch2_value) * 20;
        if (new_velocity > 2000) new_velocity = 2000;

        if (new_velocity != current_velocity) {
            dxl.setProfileVelocity(DXL_ID, new_velocity);
            current_velocity = new_velocity;
        }

        // ----------------------------------------------------------------
        // ENCODER CH3: ACCELERATION CONTROL
        // ----------------------------------------------------------------
        // Range: 0-5000 (wide range for noticeable control)
        int new_acceleration = abs(encoder_ch3_value) * 50;
        if (new_acceleration > 5000) new_acceleration = 5000;

        if (new_acceleration != current_acceleration) {
            dxl.setProfileAcceleration(DXL_ID, new_acceleration);
            current_acceleration = new_acceleration;
        }

        // ----------------------------------------------------------------
        // SEND POSITION COMMAND
        // ----------------------------------------------------------------
        dxl.setGoalPosition(DXL_ID, position);

    } else {
        // ====================================================================
        // AUTO MODE - WATER RIPPLE CYCLE
        // ====================================================================

        // Initialize auto mode on first entry
        if (!auto_mode_active) {
            auto_mode_active = true;
            M5.Display.clear();  // Clear display when switching modes

            // Move to starting position (0) before beginning cycle
            dxl.setProfileVelocity(DXL_ID, 500);      // Medium speed to home
            dxl.setProfileAcceleration(DXL_ID, 500);  // Medium acceleration
            dxl.setGoalPosition(DXL_ID, 0);           // Move to position 0

            delay(1000);  // Wait for motor to reach position 0

            cycle_start_time = millis();  // Start cycle timing after homing
        }

        // ----------------------------------------------------------------
        // ENCODER CONTROLS FOR AUTO MODE
        // ----------------------------------------------------------------

        // CH1: CYCLE TIME (500-5000ms)
        auto_cycle_time = 500 + (abs(encoder_ch1_value) * 50);
        if (auto_cycle_time > 5000) auto_cycle_time = 5000;
        // Button: Reset to default
        static bool last_ch1_button = false;
        if (encoder_ch1_button && !last_ch1_button) {
            encoder.resetCounter(0);  // Reset encoder to 0
        }
        last_ch1_button = encoder_ch1_button;

        // CH2: PUSH VELOCITY (500-2000)
        auto_push_velocity = 500 + (abs(encoder_ch2_value) * 15);
        if (auto_push_velocity > 2000) auto_push_velocity = 2000;
        // Button: Set to maximum
        static bool last_ch2_button = false;
        if (encoder_ch2_button && !last_ch2_button) {
            auto_push_velocity = 2000;
        }
        last_ch2_button = encoder_ch2_button;

        // CH3: PUSH ACCELERATION (100-5000)
        auto_push_acceleration = 100 + (abs(encoder_ch3_value) * 50);
        if (auto_push_acceleration > 5000) auto_push_acceleration = 5000;
        // Button: Set to maximum
        static bool last_ch3_button = false;
        if (encoder_ch3_button && !last_ch3_button) {
            auto_push_acceleration = 5000;
        }
        last_ch3_button = encoder_ch3_button;

        // CH4: RETURN VELOCITY (50-500)
        auto_return_velocity = 50 + (abs(encoder_ch4_value) * 5);
        if (auto_return_velocity > 500) auto_return_velocity = 500;
        // Button: Set to minimum
        static bool last_ch4_button_auto = false;
        if (encoder_ch4_button && !last_ch4_button_auto) {
            auto_return_velocity = 50;
        }
        last_ch4_button_auto = encoder_ch4_button;

        // CH5: RETURN ACCELERATION (10-1000)
        auto_return_acceleration = 10 + (abs(encoder_ch5_value) * 10);
        if (auto_return_acceleration > 1000) auto_return_acceleration = 1000;
        // Button: Set to minimum
        static bool last_ch5_button = false;
        if (encoder_ch5_button && !last_ch5_button) {
            auto_return_acceleration = 10;
        }
        last_ch5_button = encoder_ch5_button;

        // CH6: PUSH/RETURN RATIO (10-90%)
        auto_push_ratio = 10 + (abs(encoder_ch6_value) * 2);
        if (auto_push_ratio > 90) auto_push_ratio = 90;
        if (auto_push_ratio < 10) auto_push_ratio = 10;
        // Button: Reset to 50%
        static bool last_ch6_button = false;
        if (encoder_ch6_button && !last_ch6_button) {
            encoder.resetCounter(5);  // Reset CH6 encoder
        }
        last_ch6_button = encoder_ch6_button;

        // CH7: DIRECTION SWAP (button only)
        static bool last_ch7_button = false;
        if (encoder_ch7_button && !last_ch7_button) {
            auto_direction_swap *= -1;  // Toggle between 1 and -1
        }
        last_ch7_button = encoder_ch7_button;

        // CH8: STROKE RANGE (0-4096 units, 0-360°)
        auto_stroke_range = abs(encoder_ch8_value) * 10;
        if (auto_stroke_range > 4096) auto_stroke_range = 4096;
        // Button: Cycle through presets
        static bool last_ch8_button = false;
        static int range_preset = 1;  // 0=±45°, 1=±90°, 2=±180°
        if (encoder_ch8_button && !last_ch8_button) {
            range_preset = (range_preset + 1) % 3;
            if (range_preset == 0) auto_stroke_range = 512;   // ±45°
            if (range_preset == 1) auto_stroke_range = 1024;  // ±90°
            if (range_preset == 2) auto_stroke_range = 2048;  // ±180°
        }
        last_ch8_button = encoder_ch8_button;

        // ----------------------------------------------------------------
        // CALCULATE CYCLE TIMING
        // ----------------------------------------------------------------
        unsigned long current_time = millis();
        unsigned long elapsed_time = current_time - cycle_start_time;
        unsigned long time_in_cycle = elapsed_time % auto_cycle_time;

        // Calculate push and return times based on ratio
        int push_time = (auto_cycle_time * auto_push_ratio) / 100;
        int return_time = auto_cycle_time - push_time;

        // ----------------------------------------------------------------
        // STATE MACHINE: PUSH vs RETURN STROKE
        // ----------------------------------------------------------------
        int target_position;
        int target_velocity;
        int target_acceleration;

        if (time_in_cycle < push_time) {
            // ============================================================
            // PUSH STROKE: FAST - creates water ripple
            // ============================================================
            target_position = -auto_stroke_range * auto_direction_swap;
            target_velocity = auto_push_velocity;
            target_acceleration = auto_push_acceleration;

        } else {
            // ============================================================
            // RETURN STROKE: SLOW - gentle return
            // ============================================================
            target_position = auto_stroke_range * auto_direction_swap;
            target_velocity = auto_return_velocity;
            target_acceleration = auto_return_acceleration;
        }

        // ----------------------------------------------------------------
        // UPDATE MOTOR PROFILE AND POSITION
        // ----------------------------------------------------------------
        // Update velocity/acceleration only if changed
        if (target_velocity != current_velocity) {
            dxl.setProfileVelocity(DXL_ID, target_velocity);
            current_velocity = target_velocity;
        }

        if (target_acceleration != current_acceleration) {
            dxl.setProfileAcceleration(DXL_ID, target_acceleration);
            current_acceleration = target_acceleration;
        }

        // Send target position
        dxl.setGoalPosition(DXL_ID, target_position);
    }

    // ========================================================================
    // READ CURRENT MOTOR POSITION
    // ========================================================================
    int32_t present_position = dxl.getPresentPosition(DXL_ID);
    // ========================================================================
    // DISPLAY INFO
    // ========================================================================
    if (!switch_status) {
        // MANUAL MODE DISPLAY
        M5.Display.setCursor(10, 100);
        M5.Display.printf("Mode: MANUAL  Dir: %s    ",
                          direction_multiplier == 1 ? "FWD" : "REV");

        M5.Display.setCursor(10, 120);
        M5.Display.printf("CH1 Pos: %ld    ", encoder_ch1_value);
        M5.Display.setCursor(10, 140);
        M5.Display.printf("CH2 Vel: %d (raw:%ld)  ", current_velocity, encoder_ch2_value);
        M5.Display.setCursor(10, 160);
        M5.Display.printf("CH3 Acc: %d (raw:%ld)  ", current_acceleration, encoder_ch3_value);
        M5.Display.setCursor(10, 180);
        M5.Display.printf("Motor Pos: %d    ", present_position);

    } else {
        // AUTO MODE DISPLAY
        unsigned long current_time = millis();
        unsigned long elapsed_time = current_time - cycle_start_time;
        unsigned long time_in_cycle = elapsed_time % auto_cycle_time;
        int cycle_progress = (time_in_cycle * 100) / auto_cycle_time;

        int push_time = (auto_cycle_time * auto_push_ratio) / 100;
        int stroke_angle = (auto_stroke_range * 360) / 4096;  // Convert to degrees

        M5.Display.setCursor(10, 100);
        M5.Display.println("=== AUTO MODE ===");

        // ----------------------------------------------------------------
        // MOTION GRAPH - Visual representation of the cycle
        // ----------------------------------------------------------------
        M5.Display.setCursor(10, 120);
        M5.Display.print("Graph: ");

        // Draw simple motion curve (30 chars wide)
        int graph_width = 30;
        int push_width = (graph_width * auto_push_ratio) / 100;
        int return_width = graph_width - push_width;
        int current_pos = (time_in_cycle * graph_width) / auto_cycle_time;

        for (int i = 0; i < graph_width; i++) {
            if (i == current_pos) {
                M5.Display.print("O");  // Current position marker
            } else if (i < push_width) {
                M5.Display.print("\\");  // Push stroke (down)
            } else {
                M5.Display.print("/");   // Return stroke (up)
            }
        }

        M5.Display.setCursor(10, 140);
        if (time_in_cycle < push_time) {
            M5.Display.printf("PUSH  Dir:%s  Prog:%d%%    ",
                              auto_direction_swap == 1 ? "NORM" : "SWAP", cycle_progress);
        } else {
            M5.Display.printf("RETURN Dir:%s Prog:%d%%   ",
                              auto_direction_swap == 1 ? "NORM" : "SWAP", cycle_progress);
        }

        M5.Display.setCursor(10, 160);
        M5.Display.printf("Cycle:%dms R:%d/%d%% +-%ddeg",
                          auto_cycle_time, auto_push_ratio, 100-auto_push_ratio, stroke_angle);

        M5.Display.setCursor(10, 180);
        M5.Display.printf("Push: V=%d A=%d      ",
                          auto_push_velocity, auto_push_acceleration);

        M5.Display.setCursor(10, 200);
        M5.Display.printf("Rtn:  V=%d A=%d      ",
                          auto_return_velocity, auto_return_acceleration);

        M5.Display.setCursor(10, 220);
        M5.Display.printf("Motor: %d   ", present_position);
    }

    delay(1000/120.);  // Update 120 times per second
}
