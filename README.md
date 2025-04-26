# 🚦 Advanced Line Follower Robot

An Arduino-based line following robot with PID control and safety features.

## 📋 Technical Specifications
- **Controller**: Arduino Uno
- **Sensors**: 5x IR (A0-A4)
- **Motor Driver**: L298N
- **Key Features**:
  - PID control (Kp=0.8, Ki=0.01, Kd=0.1)
  - Motor acceleration/speed limiting
  - Emergency stop (Pin D12)
  - Serial debug mode

## 🔧 Wiring
```cpp
// Pin Definitions:
const uint8_t ENA = 5;    // Right motor PWM
const uint8_t IN1 = 4;    // Right motor direction
const uint8_t IN2 = 3;
const uint8_t IN3 = 2;    // Left motor direction
const uint8_t IN4 = 7;
const uint8_t ENB = 6;    // Left motor PWM
const uint8_t IR_PINS[] = {A0, A1, A2, A3, A4}; // Line sensors