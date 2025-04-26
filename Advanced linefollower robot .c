/*
 * Advanced Line Follower Robot
 * Features:
 * - 5x IR sensor array
 * - PID control algorithm
 * - Motor acceleration limiting
 * - Auto-calibration
 * - Debug mode
 * - Emergency stop
 * 
 * Hardware Connections:
 * - Motors: L298N driver (ENA:5, IN1:4, IN2:3, IN3:2, IN4:7, ENB:6)
 * - IR Sensors: A0-A4 (Left to Right)
 * - Emergency Button: D12 (optional)
 */

#include <stdint.h>
#include <Arduino.h>

// Pin Definitions
const uint8_t ENA = 5;
const uint8_t IN1 = 4;
const uint8_t IN2 = 3;
const uint8_t IN3 = 2;
const uint8_t IN4 = 7;
const uint8_t ENB = 6;
const uint8_t EMERGENCY_PIN = 12;
const uint8_t IR_PINS[] = {A0, A1, A2, A3, A4};
#define NUM_SENSORS 5

// Configuration Structures
typedef struct {
    uint8_t base_speed;    // Normal forward speed (0-255)
    uint8_t max_speed;     // Maximum allowed speed
    uint8_t turn_speed;    // Turning speed
    uint8_t min_speed;     // Minimum working speed
} SpeedConfig;

typedef struct {
    uint16_t sharp_delay;  // Sharp turn duration (ms)
    uint16_t adjust_delay; // Small adjustment duration
    uint16_t calib_time;   // Calibration duration (ms)
} TimingConfig;

typedef struct {
    float Kp;             // Proportional gain
    float Ki;             // Integral gain
    float Kd;             // Derivative gain
    int16_t max_i;        // Anti-windup limit
} PIDConfig;

// Current Configuration
SpeedConfig speed = {
    .base_speed = 150,
    .max_speed = 200,
    .turn_speed = 180,
    .min_speed = 80
};

TimingConfig timing = {
    .sharp_delay = 300,
    .adjust_delay = 100,
    .calib_time = 5000
};

PIDConfig pid = {
    .Kp = 0.8,
    .Ki = 0.01,
    .Kd = 0.1,
    .max_i = 100
};

// Global Variables
uint8_t sensor_values[NUM_SENSORS];
uint8_t calibrated = 0;
float pid_error = 0, pid_last_error = 0, pid_integral = 0;

// Function Prototypes
void setup_hardware();
void read_sensors();
void calibrate_sensors();
void set_motors(int8_t left, int8_t right);
void safe_set_motors(int8_t left, int8_t right);
void pid_control();
void emergency_check();
void debug_print();

void setup() {
    Serial.begin(9600);
    setup_hardware();
    calibrate_sensors();
}

void loop() {
    emergency_check();
    read_sensors();
    
    if(millis() < timing.calib_time && !calibrated) {
        calibrate_sensors();
        return;
    }
    
    debug_print();
    pid_control();
}

// Initialize all hardware components
void setup_hardware() {
    // Set motor control pins as outputs
    DDRD |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);
    DDRD |= (1 << ENA) | (1 << ENB);
    
    // Set sensor pins as inputs
    for(uint8_t i = 0; i < NUM_SENSORS; i++) {
        pinMode(IR_PINS[i], INPUT);
    }
    
    // Emergency stop button
    pinMode(EMERGENCY_PIN, INPUT_PULLUP);
    
    // Increase PWM frequency for smoother motor control
    TCCR0B = (TCCR0B & 0b11111000) | 0b00000010;
}

// Read all IR sensors and store values
void read_sensors() {
    for(uint8_t i = 0; i < NUM_SENSORS; i++) {
        sensor_values[i] = !digitalRead(IR_PINS[i]); // Invert logic (1 = line detected)
    }
}

// Automatic sensor calibration
void calibrate_sensors() {
    static uint16_t calib_count = 0;
    
    if(calib_count == 0) {
        Serial.println("Starting calibration...");
    }
    
    // Simple calibration - just wait for sensors to stabilize
    if(calib_count++ > 100) {
        calibrated = 1;
        Serial.println("Calibration complete");
    }
    delay(10);
}

// Set motor speeds with direction control
void set_motors(int8_t left, int8_t right) {
    // Constrain speeds to safe limits
    left = constrain(left, -speed.max_speed, speed.max_speed);
    right = constrain(right, -speed.max_speed, speed.max_speed);
    
    // Right motor control
    if(right > 0) {
        PORTD |= (1 << IN1);
        PORTD &= ~(1 << IN2);
    } else if(right < 0) {
        PORTD &= ~(1 << IN1);
        PORTD |= (1 << IN2);
    } else {
        PORTD &= ~((1 << IN1) | (1 << IN2));
    }
    
    // Left motor control
    if(left > 0) {
        PORTD |= (1 << IN3);
        PORTD &= ~(1 << IN4);
    } else if(left < 0) {
        PORTD &= ~(1 << IN3);
        PORTD |= (1 << IN4);
    } else {
        PORTD &= ~((1 << IN3) | (1 << IN4));
    }
    
    // Apply PWM speeds
    analogWrite(ENA, abs(right));
    analogWrite(ENB, abs(left));
}

// Safer motor control with acceleration limiting
void safe_set_motors(int8_t left, int8_t right) {
    static int8_t last_left = 0, last_right = 0;
    const int8_t accel_limit = 30; // Maximum speed change per call
    
    // Limit acceleration
    left = constrain(left, last_left - accel_limit, last_left + accel_limit);
    right = constrain(right, last_right - accel_limit, last_right + accel_limit);
    
    // Ensure minimum speed when moving
    if(abs(left) > 0 && abs(left) < speed.min_speed) {
        left = (left > 0) ? speed.min_speed : -speed.min_speed;
    }
    if(abs(right) > 0 && abs(right) < speed.min_speed) {
        right = (right > 0) ? speed.min_speed : -speed.min_speed;
    }
    
    set_motors(left, right);
    last_left = left;
    last_right = right;
}

// PID control algorithm
void pid_control() {
    // Calculate weighted error (-2 to +2 scale)
    pid_error = (-2 * sensor_values[0] 
                -1 * sensor_values[1] 
                +0 * sensor_values[2] 
                +1 * sensor_values[3] 
                +2 * sensor_values[4]);
    
    // PID terms calculation
    pid_integral += pid_error;
    pid_integral = constrain(pid_integral, -pid.max_i, pid.max_i); // Anti-windup
    
    float derivative = pid_error - pid_last_error;
    float correction = pid.Kp * pid_error 
                     + pid.Ki * pid_integral 
                     + pid.Kd * derivative;
    
    // Apply correction
    safe_set_motors(speed.base_speed - correction, 
                   speed.base_speed + correction);
    
    pid_last_error = pid_error;
}

// Check for emergency stop condition
void emergency_check() {
    if(digitalRead(EMERGENCY_PIN) {
        set_motors(0, 0);
        Serial.println("EMERGENCY STOP ACTIVATED!");
        while(1); // Halt execution
    }
}

// Debug printing function
void debug_print() {
    #define DEBUG 1  // Set to 0 to disable debug output
    
    if(DEBUG) {
        // Print sensor values
        Serial.print("Sensors: ");
        for(uint8_t i = 0; i < NUM_SENSORS; i++) {
            Serial.print(sensor_values[i]);
            Serial.print(" ");
        }
        
        // Print PID values
        Serial.print("| Error: ");
        Serial.print(pid_error);
        Serial.print(" Int: ");
        Serial.print(pid_integral);
        Serial.print(" Adj: ");
        Serial.print(pid.Kp * pid_error + pid.Ki * pid_integral + pid.Kd * (pid_error - pid_last_error));
        
        Serial.println();
    }
}