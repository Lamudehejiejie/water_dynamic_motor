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
int direction_multiplier = -1;      // Direction: 1 = FWD, -1 = REV

// Auto mode control variables
unsigned long cycle_start_time = 0;     // When current cycle started
bool auto_mode_active = false;          // Track if we're in auto mode
int saved_cycle_time = 2000;            // Remember cycle time when switching modes

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
    M5.Display.setTextSize(1.5);
    M5.Display.setCursor(10, 10);
    M5.Display.println("Motor + Encoder Control");

    // ========================================================================
    // INITIALIZE I2C FOR ENCODER
    // ========================================================================
    // M5Stack CoreS3 Port.A: SDA=GPIO2, SCL=GPIO1

    // Initialize 8-Encoder Unit with retry logic
    // Sometimes the encoder needs a moment to initialize after power-on
    encoder_found = false;
    for (int retry = 0; retry < 10; retry++) {
        delay(300);  // Wait longer between attempts

        // Reinitialize I2C on each retry to ensure clean state
        if (retry > 0) {
            Wire.end();
            delay(100);
        }
        Wire.begin(2, 1);
        delay(100);

        if (encoder.begin(&Wire, ENCODER_I2C_ADDR, 2, 1)) {
            M5.Display.setCursor(10, 20);
            M5.Display.printf("Encoder found! (try %d)", retry + 1);

            // Don't reset encoder - keep current value to maintain position
            // This allows the motor to start from its current position

            encoder_found = true;
            break;  // Success - exit retry loop
        }
    }

    if (!encoder_found) {
        M5.Display.setCursor(10, 20);
        M5.Display.println("Encoder NOT found - HALT");
        while(1) { delay(100); }  // Stop here if no encoder after retries
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
    int32_t encoder_ch2_value = max(0, encoder.getEncoderValue(1));
    int32_t encoder_ch3_value = max(0, encoder.getEncoderValue(2));
    int32_t encoder_ch4_value = max(0, encoder.getEncoderValue(3));     
    int32_t encoder_ch5_value = max(0, encoder.getEncoderValue(4));
    int32_t encoder_ch6_value = max(0, encoder.getEncoderValue(5));
    int32_t encoder_ch7_value = max(0, encoder.getEncoderValue(6));
    int32_t encoder_ch8_value = max(0, encoder.getEncoderValue(7));

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

            // Save current cycle time to restore it later
            saved_cycle_time = auto_cycle_time;

            // Get current motor position and set encoder to match it (no homing)
            int32_t current_motor_position = dxl.getPresentPosition(DXL_ID);
            // Calculate encoder value that corresponds to current motor position
            int32_t new_encoder_value = -(current_motor_position * ENCODER_COUNTS_PER_REV) / (MAX_POSITION * direction_multiplier);
            encoder.setEncoderValue(0, new_encoder_value);
        }

        // ----------------------------------------------------------------
        // ENCODER CH4: DIRECTION TOGGLE (BUTTON)
        // ----------------------------------------------------------------
        static bool last_ch4_button = false;
        static bool direction_just_changed = false;
        if (encoder_ch4_button && !last_ch4_button) {  // Button press detected
            // Read current motor position before changing direction
            int32_t current_motor_position = dxl.getPresentPosition(DXL_ID);

            // Toggle direction
            direction_multiplier *= -1;

            // Calculate what encoder value would maintain current motor position with new direction
            // position = (-encoder_ch1_value * MAX_POSITION * direction_multiplier) / ENCODER_COUNTS_PER_REV
            // Solve for encoder_ch1_value: encoder_ch1_value = -(position * ENCODER_COUNTS_PER_REV) / (MAX_POSITION * direction_multiplier)
            int32_t new_encoder_value = -(current_motor_position * ENCODER_COUNTS_PER_REV) / (MAX_POSITION * direction_multiplier);

            // Set encoder to this new value to maintain motor position
            encoder.setEncoderValue(0, new_encoder_value);

            // Send current position immediately to prevent movement
            dxl.setGoalPosition(DXL_ID, current_motor_position);
            direction_just_changed = true;
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
        // Use max(0, value) so negative values = 0 (stay at minimum)
        int new_velocity = max(0, encoder_ch2_value) * 2.5;
        if (new_velocity > 2000) new_velocity = 2000;

        if (new_velocity != current_velocity) {
            dxl.setProfileVelocity(DXL_ID, new_velocity);
            current_velocity = new_velocity;
        }

        // ----------------------------------------------------------------
        // ENCODER CH3: ACCELERATION CONTROL
        // ----------------------------------------------------------------
        // Range: 0-5000 (wide range for noticeable control)
        // Use max(0, value) so negative values = 0 (stay at minimum)
        int new_acceleration = max(0, encoder_ch3_value) * 2.5;
        if (new_acceleration > 5000) new_acceleration = 5000;

        if (new_acceleration != current_acceleration) {
            dxl.setProfileAcceleration(DXL_ID, new_acceleration);
            current_acceleration = new_acceleration;
        }

        // ----------------------------------------------------------------
        // SEND POSITION COMMAND
        // ----------------------------------------------------------------
        // Skip sending position if direction just changed (already sent in direction toggle logic)
        if (!direction_just_changed) {
            dxl.setGoalPosition(DXL_ID, position);
        } else {
            direction_just_changed = false;  // Reset flag for next loop
        }

    } else {
        // ====================================================================
        // AUTO MODE - WATER RIPPLE CYCLE
        // ====================================================================

        // Initialize auto mode on first entry
        if (!auto_mode_active) {
            auto_mode_active = true;
            M5.Display.clear();  // Clear display when switching modes

            // Restore saved cycle time from when we left auto mode
            auto_cycle_time = saved_cycle_time;

            // Start cycle timing from current position (no homing)
            cycle_start_time = millis();
        }

        // ----------------------------------------------------------------
        // ENCODER CONTROLS FOR AUTO MODE
        // ----------------------------------------------------------------

        // CH1: CYCLE TIME (2000-5000ms)
        auto_cycle_time = 2000 + (abs(encoder_ch1_value) * 50);
        if (auto_cycle_time > 5000) auto_cycle_time = 5000;
        // Button: Reset to default
        static bool last_ch1_button = false;
        if (encoder_ch1_button && !last_ch1_button) {
            encoder.resetCounter(0);  // Reset encoder to 0
        }
        last_ch1_button = encoder_ch1_button;

        // CH2: PUSH VELOCITY (500-2000)
        auto_push_velocity = 500 + (abs(encoder_ch2_value) * 2.5);
        if (auto_push_velocity > 2000) auto_push_velocity = 2000;

        // CH3: PUSH ACCELERATION (100-5000)
        auto_push_acceleration = 100 + (abs(encoder_ch3_value) * 2.5);
        if (auto_push_acceleration > 5000) auto_push_acceleration = 5000;

        // CH4: RETURN VELOCITY (50-500)
        auto_return_velocity = 50 + (abs(encoder_ch4_value) * 2.5);
        if (auto_return_velocity > 500) auto_return_velocity = 500;  // Fixed bug

        // CH5: RETURN ACCELERATION (10-1000)
        auto_return_acceleration = 10 + (abs(encoder_ch5_value) * 2.5);
        if (auto_return_acceleration > 1000) auto_return_acceleration = 1000;

        // CH6: PUSH/RETURN RATIO (10-90%)
        auto_push_ratio = 45 + (abs(encoder_ch6_value) * 2);
        if (auto_push_ratio > 90) auto_push_ratio = 90;
        if (auto_push_ratio < 45) auto_push_ratio = 45;

        // CH7: STROKE RANGE (0-4096 units, 0-360°)
        auto_stroke_range = 30 + abs(encoder_ch7_value) * 10;
        if (auto_stroke_range > 4096) auto_stroke_range = 4096;

        // CH8: DIRECTION SWAP (button only)
        static bool last_ch8_button = false;
        if (encoder_ch8_button && !last_ch8_button) {
            // Get current motor position before swapping
            int32_t current_motor_position = dxl.getPresentPosition(DXL_ID);

            // Toggle direction
            auto_direction_swap *= -1;

            // Send current position to motor to prevent movement
            dxl.setGoalPosition(DXL_ID, current_motor_position);
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
        int graph_width = 25;
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
        M5.Display.printf("Push: V=%d(%ld) A=%d(%ld)    ",
                          auto_push_velocity, encoder_ch2_value,
                          auto_push_acceleration, encoder_ch3_value);

        M5.Display.setCursor(10, 200);
        M5.Display.printf("Rtn:  V=%d(%ld) A=%d(%ld)    ",
                          auto_return_velocity, encoder_ch4_value,
                          auto_return_acceleration, encoder_ch5_value);

        M5.Display.setCursor(10, 220);
        M5.Display.printf("Motor: %d   ", present_position);
    }

    delay(1000/120.);  // Update 120 times per second
}
