// ============================================================================
// LIBRARIES
// ============================================================================
#include <M5Unified.h>           // M5Stack hardware library
#include <Dynamixel2Arduino.h>   // ROBOTIS library for Dynamixel motor control
#include <UNIT_8ENCODER.h>       // M5Stack 8-Encoder Unit library


// The flow: M5Stack (TTL) → DMX Base (TTL→RS485) → Motor (RS485)
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
//const uint32_t DXL_BAUD_RATE = 1000000;      // Communication speed (bits/second)
const uint32_t DXL_BAUD_RATE = 57600;
                                             // RS485 supports: 9600, 57600, 115200, 1000000, etc.
                                             // 1000000 = 1 Mbps (modern Dynamixel default) Dynamixel XL430 W250-T
                                             // 57600 = 1 Mbps (modern Dynamixel default) Dynamixel XL430 W250-T
                                             // Your motor is configured to 1000000 baud

// Create Dynamixel controller object
// This handles all the complex packet formatting and RS485 communication
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// Control Table Addresses for XW540 (Protocol 2.0)
#define ADDR_TORQUE_ENABLE 64
#define ADDR_HARDWARE_ERROR_STATUS 70
#define ADDR_MOVING 122

// ============================================================================
// ENCODER SETUP - M5Stack 8-Encoder Unit
// ============================================================================
#define ENCODER_I2C_ADDR 0x41    // I2C address for 8-Encoder Unit
UNIT_8ENCODER encoder;           // Create encoder object

// ============================================================================
// MOTION CONTROL VARIABLES
// ============================================================================
bool encoder_found = false;        // Flag: encoder detected and working?
const int MAX_POSITION = 4095;     // XW540-T260-R: 4095 = 360° (one full rotation)
const int CENTER_POSITION = 2048;  // Center position (180°) - safe starting point
const int MAX_POSITION_LIMIT = 4095;  // Hard limit: never exceed one full rotation
                                   // 0 = 0°, 2048 = 180°, 4095 = 360°
//tenkii, change here!                    
const int ENCODER_CH1_RANGE = 22;  // CH1 encoder range: -22 to +22 (44 clicks total, safety limited)
const int POSITION_SCALE = 68;     // Scale factor: 4095/60 ≈ 68 (same scale as ±30, just limited range)
const int WALL_ALIGNED_CH1 = 20;
// Manual mode control variables
int current_velocity = 0;          // Track current velocity setting
int current_acceleration = 0;      // Track current acceleration setting
const int direction_multiplier = 1;      // Direction: 1 = FWD, -1 = REV

// Auto mode control variables
unsigned long cycle_start_time = 0;     // When current cycle started
bool auto_mode_active = false;          // Track if we're in auto mode
int saved_cycle_time = 3000;            // Remember cycle time when switching modes (default: 5000ms)
bool use_encoder_control = false;       // Flag: use encoder values or keep defaults
int auto_mode_center_position = 0;      // Motor position when entering auto mode (oscillate around this)
bool auto_mode_homed = false;
int wall_motor_position = 0;
// Power scaling factor: maps user range (0-2000) to motor capability
// 0.5 = 50% of max motor power (max 32767), 1.0 = 100% of motor power
const float POWER_SCALE = 0.2;          // Adjust this to increase/decrease max power

// Auto mode parameters (controllable via encoders)
int auto_cycle_time = 3000;             // Total cycle time (ms) - CH1 
int auto_push_velocity = 500;          // Fast push velocity (500-2000) - CH2 
int auto_push_acceleration = 500;       // Fast push acceleration (100-5000) - CH3 
int auto_return_velocity = 100;         // Slow return velocity (50-500) - CH4
int auto_return_acceleration = 100;     // Slow return acceleration (10-1000) - CH5
int auto_push_ratio = 33;               // Push time as % of cycle (5-90%) - CH6 
int auto_direction_swap = 1;           // Direction multiplier: 1=normal, -1=swapped - CH7
int auto_stroke_range = 512;           // Stroke range in units (±45°) - CH8 - 

//random function
int auto_cycle_interval = 1000;

// ============================================================================
// SETUP - Runs once when M5Stack starts
// ============================================================================
void setup() {
    // Initialize M5Stack hardware AFTER I2C cleanup
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Power.setExtOutput(true);  // turn on Grove 5V, 
    delay(1000);                   // wait a bit

    for (int i = 0; i < 3; i++) {
        Wire.begin(2, 1);
        encoder.begin(&Wire, 0x41, 2, 1);
        Wire.beginTransmission(0x00);
        Wire.write(0x06);  // General call reset
        Wire.endTransmission();
        Wire.beginTransmission(ENCODER_I2C_ADDR);
        Wire.endTransmission();
        Wire.end();
    }

    // Final I2C initialization
    Wire.begin(2, 1);
    // delay(100);

    // Setup display
    M5.Display.clear();
    M5.Display.setTextSize(1.5);
    M5.Display.setCursor(10, 10);
    M5.Display.println("Motor + Encoder Control");
    M5.Display.setCursor(10, 30);
    M5.Display.println("Cleaning I2C bus...");
    delay(100);

    // Try to initialize encoder with retries
    encoder_found = false;
    for (int retry = 0; retry < 25; retry++) {  // Reduced from 50 to 25
        // Every 5th retry, do a full I2C reset
        if (retry > 0 && retry % 5 == 0) {
            M5.Display.setCursor(10, 40);
            M5.Display.printf("Retry %d - Deep reset...    ", retry);
            M5.Display.setCursor(10, 60);
            M5.Display.println("Try unplug encoder if stuck");
            // Complete I2C bus reset
            Wire.end();
            delay(300);  // Reduced from 1000ms

            // Reinitialize I2C
            Wire.begin(2, 1);
            delay(300);  // Reduced from 1000ms

            // Try sending general call reset to I2C bus (address 0x00)
            Wire.beginTransmission(0x00);
            Wire.write(0x06);  // General call reset command
            Wire.endTransmission();
            delay(200);  // Reduced from 500ms
        }

        // Try to initialize encoder
        if (encoder.begin(&Wire, ENCODER_I2C_ADDR, 2, 1)) {
            for (int reset_attempt = 0; reset_attempt < 5; reset_attempt++) {
                for (int ch = 0; ch < 8; ch++) {
                    encoder.resetCounter(ch);
                }
                delay(50); 
            }

            delay(100);
            int32_t ch1_check = encoder.getEncoderValue(0);

            encoder_found = true;
            break;  // Success - exit retry loop
        }

        delay(200);  // Reduced from 400ms
    }

    if (!encoder_found) {
        M5.Display.clear();
        M5.Display.setCursor(10, 10);
        M5.Display.println("=== WARNING ===");
        M5.Display.setCursor(10, 30);
        M5.Display.println("Encoder NOT found!");
        M5.Display.setCursor(10, 50);
        M5.Display.println("Entering SAFE MODE...");
        M5.Display.setCursor(10, 70);
        M5.Display.println("Motor: Auto +/-180deg");
        M5.Display.setCursor(10, 90);
        M5.Display.println("No manual control");
        delay(3000);
        // Continue without encoder - will use default auto mode
    }

    Serial1.begin(DXL_BAUD_RATE, SERIAL_8N1, RXD_PIN, TXD_PIN);


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

        // Read and display motor model
        uint16_t model = dxl.getModelNumber(DXL_ID);
        M5.Display.setCursor(10, 70);
        M5.Display.printf("Model: %d ", model);
        if (model == 1180) {
            M5.Display.println("(XW540)");
        } else if (model == 1060) {
            M5.Display.println("(XL430)");
        }

        // Check hardware errors BEFORE configuration
        uint8_t hw_error = dxl.readControlTableItem(ADDR_HARDWARE_ERROR_STATUS, DXL_ID);
        M5.Display.setCursor(10, 90);
        if (hw_error != 0) {
            M5.Display.printf("HW ERROR: 0x%02X!", hw_error);
            delay(2000);
        } else {
            M5.Display.println("No errors");
        }

        // Configure motor - turn off torque first
        dxl.torqueOff(DXL_ID);
        delay(100);

        dxl.setOperatingMode(DXL_ID, OP_POSITION);
        delay(100);

        // Set motion profile to match manual mode defaults
        current_velocity = 500;
        current_acceleration = 500;
        dxl.setProfileVelocity(DXL_ID, current_velocity);
        dxl.setProfileAcceleration(DXL_ID, current_acceleration);
        delay(100);

        // Enable torque - try both methods
        dxl.torqueOn(DXL_ID);
        delay(50);
        // Also try direct write to control table
        dxl.writeControlTableItem(ADDR_TORQUE_ENABLE, DXL_ID, 1);
        delay(100);

        // Verify
        uint8_t torque_check = dxl.readControlTableItem(ADDR_TORQUE_ENABLE, DXL_ID);
        M5.Display.setCursor(10, 110);

        if (torque_check == 1) {
            M5.Display.println("Ready! Torque ON");
            dxl.ledOn(DXL_ID);
            // Encoder already reset during initialization
        } else {

        }
    } else {
        M5.Display.setCursor(10, 50);
        M5.Display.println("Motor NOT found!");
        M5.Display.setCursor(10, 70);
        M5.Display.println("Check:");
        M5.Display.setCursor(10, 90);
        M5.Display.println("- Power (12V)");
        M5.Display.setCursor(10, 110);
        M5.Display.println("- Wiring (A/B)");
        M5.Display.setCursor(10, 130);
        M5.Display.println("- Baud: 57600");
    }

    // ========================================================================
    // HOMING: Move motor to center position (2048) if outside safe range
    // ========================================================================
    if (encoder_found) {
        int32_t current_position = dxl.getPresentPosition(DXL_ID);

        M5.Display.setCursor(10, 155);
        M5.Display.printf("Current Pos: %ld", current_position);
        delay(500);

        // Check if position is outside safe range (0-4095)
        if (current_position < 0 || current_position > MAX_POSITION_LIMIT) {
            M5.Display.setCursor(10, 185);
            M5.Display.println("OUT OF RANGE! Homing...");

            // Home to center position (2048)
            dxl.setGoalPosition(DXL_ID, CENTER_POSITION);
            delay(3000);  // Wait for motor to reach center

            M5.Display.setCursor(10, 185);
            M5.Display.println("Homed to center (2048)");
            delay(1000);
        } else {
            M5.Display.setCursor(10, 175);
            M5.Display.println("Position OK, no homing needed");
            delay(1000);
        }

        // Encoder CH1 is already at 0 (represents center position 2048)
        M5.Display.setCursor(10, 215);
        M5.Display.println("Ready! CH1=0 = Pos 2048");
        delay(1500);
    }
}

// ============================================================================
// LOOP - Runs continuously (many times per second)
// ============================================================================
void loop() {
    M5.update();  // Update M5Stack internal state (buttons, etc.)

    // ========================================================================
    // SAFE MODE - Run default auto mode if encoder not found
    // ========================================================================
    if (!encoder_found) {
        // Force auto mode on with default settings (±45° for XW540)
        if (!auto_mode_active) {
            auto_mode_active = true;
            cycle_start_time = millis();
            auto_stroke_range = 512;  // ±45 degrees (512 units)

            // Always center at 2048 in safe mode too
            auto_mode_center_position = CENTER_POSITION;
        }

        // Calculate cycle timing
        unsigned long current_time = millis();
        unsigned long elapsed_time = current_time - cycle_start_time;
        unsigned long time_in_cycle = elapsed_time % auto_cycle_time;

        int push_time = (auto_cycle_time * auto_push_ratio) / 100;

        int target_position;
        int target_velocity;
        int target_acceleration;

        if (time_in_cycle < push_time) {
            // PUSH STROKE - relative to center position
            target_position = auto_mode_center_position + (-auto_stroke_range * auto_direction_swap);
            target_velocity = auto_push_velocity;
            target_acceleration = auto_push_acceleration;
        } else {
            // RETURN STROKE - relative to center position
            target_position = auto_mode_center_position + (auto_stroke_range * auto_direction_swap);
            target_velocity = auto_return_velocity;
            target_acceleration = auto_return_acceleration;
        }

        // Limit to valid range (0-4095, one full rotation max)
        if (target_position < 0) target_position = 0;
        if (target_position > MAX_POSITION_LIMIT) target_position = MAX_POSITION_LIMIT;

        // Update motor profile
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

        // Display safe mode info with DEBUG
        int32_t present_position = dxl.getPresentPosition(DXL_ID);
        int32_t present_velocity = dxl.getPresentVelocity(DXL_ID);
        uint8_t moving_status = dxl.readControlTableItem(ADDR_MOVING, DXL_ID);

        M5.Display.setCursor(10, 110);
        M5.Display.printf("=== SAFE MODE (AUTO) ===     ");
        M5.Display.setCursor(10, 130);
        M5.Display.printf("Target: %d (+-%ddeg)      ", target_position, (auto_stroke_range * 360) / MAX_POSITION);
        M5.Display.setCursor(10, 150);
        M5.Display.printf("Current: %d       ", present_position);
        M5.Display.setCursor(10, 170);
        M5.Display.printf("Vel: %d  Move: %s      ", present_velocity, moving_status ? "YES" : "NO");
        M5.Display.setCursor(10, 190);
        M5.Display.printf("V:%d A:%d      ", target_velocity, target_acceleration);

        delay(1000/120.);
        return;  // Skip normal encoder-based control
    }

    // ========================================================================
    // READ ENCODER VALUES (only if encoder found)
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

            // Save current cycle time to restore it later
            saved_cycle_time = auto_cycle_time;

            // Keep encoder values preserved when switching modes
            // (encoders are NOT reset, parameters persist across mode switches)
        }

        // ----------------------------------------------------------------
        // ENCODER CH4: DIRECTION TOGGLE (BUTTON)
        // ----------------------------------------------------------------
        // static bool last_ch4_button = false;
        // static bool direction_just_changed = false;
        // if (encoder_ch4_button && !last_ch4_button) {  // Button press detected
        //     // Read current motor position before changing direction
        //     int32_t current_motor_position = dxl.getPresentPosition(DXL_ID);

        //     // Toggle direction
        //     direction_multiplier *= -1;

        //     // Calculate encoder value to maintain position with new direction
        //     // Solve: position = CENTER + (CH1 * SCALE * direction)
        //     // CH1 = (position - CENTER) / (SCALE * direction)
        //     int32_t new_encoder_value = (current_motor_position - CENTER_POSITION) / (POSITION_SCALE * direction_multiplier);

        //     // Set encoder to this new value to maintain motor position
        //     encoder.setEncoderValue(0, new_encoder_value);

        //     direction_just_changed = true;
        // }
        // last_ch4_button = encoder_ch4_button;

        // ----------------------------------------------------------------
        // RUNTIME SAFETY CLAMP FOR CH1
        // ----------------------------------------------------------------
        // Safety check: Limit CH1 to ±22 range
        if (encoder_ch1_value < -ENCODER_CH1_RANGE) {
            encoder.setEncoderValue(0, -ENCODER_CH1_RANGE);
            encoder_ch1_value = -ENCODER_CH1_RANGE;
        } else if (encoder_ch1_value > ENCODER_CH1_RANGE) {
            encoder.setEncoderValue(0, ENCODER_CH1_RANGE);
            encoder_ch1_value = ENCODER_CH1_RANGE;
        }

        // ----------------------------------------------------------------
        // ENCODER CH1: POSITION CONTROL (centered at 2048)
        // ----------------------------------------------------------------
        // Formula: position = 2048 + (CH1 * 68 * direction)
        // CH1=-22 → position ≈ 552 (safety margin from 0)
        // CH1=0   → position = 2048 (center)
        // CH1=+22 → position ≈ 3544 (safety margin from 4095)
        int position = CENTER_POSITION + (encoder_ch1_value * POSITION_SCALE * direction_multiplier);

        // Hard limit to prevent wrap-around (CRITICAL SAFETY!)
        if (position < 0) position = 0;
        if (position > MAX_POSITION_LIMIT) position = MAX_POSITION_LIMIT;

        // ----------------------------------------------------------------
        // ENCODER CH2: VELOCITY CONTROL
        // ----------------------------------------------------------------
        // Adjust from default 500, range: 10-600
        int new_velocity = 500 + (encoder_ch2_value * 2.5);
        if (new_velocity < 10) new_velocity = 10;
        if (new_velocity > 600) new_velocity = 600;

        if (new_velocity != current_velocity) {
            dxl.setProfileVelocity(DXL_ID, new_velocity);
            current_velocity = new_velocity;
        }

        // ----------------------------------------------------------------
        // ENCODER CH3: ACCELERATION CONTROL
        // ----------------------------------------------------------------
        // Adjust from default 500, range: 10-600
        int new_acceleration = 500 + (encoder_ch3_value * 2.5);
        if (new_acceleration < 10) new_acceleration = 10;
        if (new_acceleration > 600) new_acceleration = 600;

        if (new_acceleration != current_acceleration) {
            dxl.setProfileAcceleration(DXL_ID, new_acceleration);
            current_acceleration = new_acceleration;
        }

        // ----------------------------------------------------------------
        // SEND POSITION COMMAND
        // ----------------------------------------------------------------
        // Skip sending position if direction just changed (already sent in direction toggle logic)
        dxl.setGoalPosition(DXL_ID, position);
        // if (!direction_just_changed) {
        //     dxl.setGoalPosition(DXL_ID, position);
        // } else {
        //     direction_just_changed = false;  // Reset flag for next loop
        // }

    } else {
        // ====================================================================
        // AUTO MODE - WATER RIPPLE CYCLE
        // ====================================================================

        // Initialize auto mode on first entry
        if (!auto_mode_active) {
            auto_mode_active = true;
            use_encoder_control = true;
            auto_mode_homed = false;
            M5.Display.clear();  // Clear display when switching modes

            // Restore saved cycle time from when we left auto mode
            auto_cycle_time = saved_cycle_time;

            // Keep encoder values preserved when switching modes
            // (encoders are NOT reset, parameters persist across mode switches)

            // CRITICAL: Always center auto mode at 2048 (center position)
            // This prevents dangerous wrap-around at 0/4095 boundary
            
            wall_motor_position = CENTER_POSITION + (WALL_ALIGNED_CH1 * POSITION_SCALE * direction_multiplier);
            auto_mode_center_position = wall_motor_position;
            // Start cycle timing from current position (no homing)
            dxl.setProfileVelocity(DXL_ID, auto_return_velocity);
            dxl.setProfileAcceleration(DXL_ID, auto_return_acceleration);
            dxl.setGoalPosition(DXL_ID, wall_motor_position);

            
        }

 
        // Enable encoder control immediately since we're preserving values
        
        
        // ----------------------------------------------------------------
        // ENCODER CONTROLS FOR AUTO MODE
        // ----------------------------------------------------------------
        // Update auto mode parameters from encoder values
        if (!auto_mode_homed)
        {
            //int wall_motor_position = CENTER_POSITION +  (WALL_ALIGNED_CH1 * POSITION_SCALE * direction_multiplier);
            int32_t current_pos = dxl.getPresentPosition(DXL_ID);
            if (abs(current_pos - wall_motor_position) < 10)
            {
                auto_mode_homed = true;
                cycle_start_time = millis();
            }
            return;
            // M5.Display.setCursor(10, 90);
            // M5.Display.printf("HOMING...                    ");
            // M5.Display.setCursor(10, 110);
            // M5.Display.printf("Current: %ld Target: %d      ",
            // current_pos, wall_motor_position);
            // M5.Display.setCursor(10, 130);
            // M5.Display.printf("Distance: %ld                ",
            // abs(current_pos - wall_motor_position));
        }

    if (auto_mode_homed)
    {
        if (use_encoder_control) {
            // CH1: CYCLE TIME (adjust from default 3000ms, range: 1000-8000ms)
            auto_cycle_time = 3000 + (encoder_ch1_value * 25);  // 25ms per encoder click
            if (auto_cycle_time < 100) auto_cycle_time = 100;
            if (auto_cycle_time > 8000) auto_cycle_time = 8000;

            // CH2: PUSH VELOCITY (adjust from default 500, range: 10-600)
            auto_push_velocity = 500 + (encoder_ch2_value * 2.5);
            if (auto_push_velocity < 10) auto_push_velocity = 10;
            if (auto_push_velocity > 600) auto_push_velocity = 600;

            // CH3: PUSH ACCELERATION (adjust from default 500, range: 10-600)
            auto_push_acceleration = 500 + (encoder_ch3_value * 2.5);
            if (auto_push_acceleration < 10) auto_push_acceleration = 10;
            if (auto_push_acceleration > 600) auto_push_acceleration = 600;

            // CH4: RETURN VELOCITY (adjust from default 100, range: 10-600)
            auto_return_velocity = 100 + (encoder_ch4_value * 2.5);
            if (auto_return_velocity < 10) auto_return_velocity = 10;
            if (auto_return_velocity > 600) auto_return_velocity = 600;

            // CH5: RETURN ACCELERATION (adjust from default 100, range: 10-600)
            auto_return_acceleration = 100 + (encoder_ch5_value * 2.5);
            if (auto_return_acceleration < 10) auto_return_acceleration = 10;
            if (auto_return_acceleration > 600) auto_return_acceleration = 600;

            // CH6: PUSH/RETURN RATIO (adjust from default 33%, range: 5-90%)
            auto_push_ratio = 33 + (encoder_ch6_value * 2);
            if (auto_push_ratio > 90) auto_push_ratio = 90;
            if (auto_push_ratio < 5) auto_push_ratio = 5;

            // CH7: STROKE RANGE (adjust from default 512 units/45°, range: 512-2048 units/45-180°)
            auto_stroke_range = 512 + (encoder_ch7_value * 12);
            const int MAX_STROKE_LIMIT = 2048;  // Maximum 2048 to prevent exceeding 0-4095 range when centered at 2048
            if (auto_stroke_range > MAX_STROKE_LIMIT) auto_stroke_range = MAX_STROKE_LIMIT;
            if (auto_stroke_range < 8) auto_stroke_range = 8;

            //CH8: interval between cycle
            auto_cycle_interval = encoder_ch8_value * 25;
            if (auto_cycle_interval > 5000) auto_cycle_interval = 5000;
            if (auto_cycle_interval < 0) auto_cycle_interval = 0;
        }
   
        // CH1 Button: Reset all encoders to restore defaults
        static bool last_ch1_button = false;
        if (encoder_ch1_button && !last_ch1_button) {
            // Reset all encoder channels to return to default settings
            for (int ch = 0; ch < 8; ch++) {
                encoder.resetCounter(ch);
            }
            use_encoder_control = false;  // Return to using defaults
        }
        last_ch1_button = encoder_ch1_button;

        // CH8: DIRECTION SWAP (button only)
        // static bool last_ch8_button = false;
        // if (encoder_ch8_button && !last_ch8_button) {
        //     // Get current motor position before swapping
        //     int32_t current_motor_position = dxl.getPresentPosition(DXL_ID);

        //     // Toggle direction
        //     auto_direction_swap *= -1;

        //     // Send current position to motor to prevent movement
        //     // dxl.setGoalPosition(DXL_ID, current_motor_position);
        // }
        // last_ch8_button = encoder_ch8_button;

        // ----------------------------------------------------------------
        // CALCULATE CYCLE TIMING
        // ----------------------------------------------------------------
        unsigned long current_time = millis();
        unsigned long elapsed_time = current_time - cycle_start_time;
        unsigned long time_in_cycle = elapsed_time % auto_cycle_time;

        static bool was_in_push_last_loop = false;

        // Calculate push and return times based on ratio
        int push_time = (auto_cycle_time * auto_push_ratio) / 100;
        bool currently_in_push = (time_in_cycle < push_time);
        if (currently_in_push && ! was_in_push_last_loop)
        {
            delay(auto_cycle_interval);
        }
        was_in_push_last_loop = currently_in_push;

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
            // Calculate position RELATIVE to center (where motor was when entering auto mode)
            target_position = auto_mode_center_position - auto_stroke_range;
            target_velocity = auto_push_velocity;
            target_acceleration = auto_push_acceleration;
            // ============================================================
            // PUSH from the wall
            // ============================================================
            
            
        } else {
            // ============================================================
            // RETURN STROKE: SLOW - gentle return
            // ============================================================
            // Calculate position RELATIVE to center (where motor was when entering auto mode)
            target_position = auto_mode_center_position;
            target_velocity = auto_return_velocity;
            target_acceleration = auto_return_acceleration;
        }

        // Limit to valid range (0-4095, one full rotation max)
        if (target_position < 0) target_position = 0;
        if (target_position > MAX_POSITION_LIMIT) target_position = MAX_POSITION_LIMIT;

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
}
    // ========================================================================
    // DISPLAY INFO - Update only every 100ms to reduce flicker
    // ========================================================================
    static unsigned long last_display_update = 0;
    if (millis() - last_display_update > 100) {
        last_display_update = millis();

        // Read current motor status for display
        int32_t present_position = dxl.getPresentPosition(DXL_ID);
        int32_t present_velocity = dxl.getPresentVelocity(DXL_ID);

        if (!switch_status) {
            // MANUAL MODE - Clean display
            M5.Display.setCursor(10, 110);
            M5.Display.printf("=== MANUAL MODE ===      ");

            // M5.Display.setCursor(10, 130);
            // M5.Display.printf("Goal Pos: %d", present_position);

            M5.Display.setCursor(10, 125);
            M5.Display.printf("Velocity: %d (CH2:%ld)     ", current_velocity, encoder_ch2_value);

            M5.Display.setCursor(10, 170);
            M5.Display.printf("Accel: %d (CH3:%ld)       ", current_acceleration, encoder_ch3_value);

            M5.Display.setCursor(10, 200);
            M5.Display.printf("Encoder CH1: %ld          ", encoder_ch1_value);

            M5.Display.setCursor(10, 230);
            if (encoder_ch1_value == -22) {
                M5.Display.printf("MIN LIMIT (Pos ~552)        ");
            } else if (encoder_ch1_value == 0) {
                M5.Display.printf("CENTER (Pos 2048)           ");
            } else if (encoder_ch1_value == 22) {
                M5.Display.printf("MAX LIMIT (Pos ~3544)       ");
            } else {
                M5.Display.printf("Range: -22 to +22           ");
            }

            M5.Display.setCursor(10, 10);
            int wall_position_calc = CENTER_POSITION +
            (WALL_ALIGNED_CH1 * POSITION_SCALE * direction_multiplier);
            M5.Display.printf("Wall Pos: %d (CH1=%d)     ",
            wall_position_calc, WALL_ALIGNED_CH1);

            M5.Display.setCursor(10, 40);
            int current_ch1_position = CENTER_POSITION +
            (encoder_ch1_value * POSITION_SCALE *
            direction_multiplier);
            M5.Display.printf("CH1 Pos: %d     ",
            current_ch1_position);

            M5.Display.setCursor(10, 70);
            M5.Display.printf("dir: %d     ",
            direction_multiplier);

    } else {
        // AUTO MODE DISPLAY
        unsigned long current_time = millis();
        unsigned long elapsed_time = current_time - cycle_start_time;
        unsigned long time_in_cycle = elapsed_time % auto_cycle_time;
        int cycle_progress = (time_in_cycle * 100) / auto_cycle_time;

        int push_time = (auto_cycle_time * auto_push_ratio) / 100;
        int stroke_angle = (auto_stroke_range * 360) / MAX_POSITION;  // Convert to degrees

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
        M5.Display.printf("Cycle:%dms(%ld) R:%d/%d%",
                          auto_cycle_time, encoder_ch1_value, auto_push_ratio, 100-auto_push_ratio);

        M5.Display.setCursor(10, 180);
        M5.Display.printf("Push: V=%d(%ld) A=%d(%ld)    ",
                          auto_push_velocity, encoder_ch2_value,
                          auto_push_acceleration, encoder_ch3_value);

        M5.Display.setCursor(10, 200);
        M5.Display.printf("Rtn:  V=%d(%ld) A=%d(%ld)    ",
                          auto_return_velocity, encoder_ch4_value,
                          auto_return_acceleration, encoder_ch5_value);

        M5.Display.setCursor(10,0);
        M5.Display.printf("Cycle_interval: %dms(%ld)",
                        auto_cycle_interval, encoder_ch8_value);
        
        M5.Display.setCursor(10,25);
        M5.Display.printf("wall_motor_position: %d",
                        wall_motor_position);

        M5.Display.setCursor(10, 50);
        M5.Display.printf("auto_mode_center: %d    ",
                        auto_mode_center_position); 

        M5.Display.setCursor(10, 75);
        M5.Display.printf("Motor Pos: %ld    ",
                            present_position);

        M5.Display.setCursor(10, 85);
        M5.Display.printf("dir: %d     ",
                            direction_multiplier);
                        
        M5.Display.setCursor(10, 220);
        if (stroke_angle >= 180) {
            M5.Display.printf("LIMIT: +-%d deg (MAX)     ", stroke_angle);
        } else {
            M5.Display.printf("Range: +-%d deg          ", stroke_angle);
        }
        }


        
    }

    delay(1000/120.);  // FPS
}
