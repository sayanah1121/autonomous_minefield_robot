#include <Arduino.h>

// --- CONFIGURATION ---
// Motor Pins (L298N)
const int IN1 = 5; const int IN2 = 6; const int ENA = 9;  // Left Motor
const int IN3 = 7; const int IN4 = 8; const int ENB = 10; // Right Motor

// Sensor Pins
const int METAL_PIN = A0;      // Inductive Sensor Analog Output
const int ENC_L_A = 2;         // Left Encoder A (Interrupt)
const int ENC_R_A = 3;         // Right Encoder A (Interrupt)

// Variables
volatile long left_ticks = 0;
volatile long right_ticks = 0;
unsigned long last_time = 0;

// Interrupt Service Routines for Encoders
void readEncoderL() { left_ticks++; }
void readEncoderR() { right_ticks++; }

void setup() {
  Serial.begin(115200); // High baud rate for low latency
  
  // Motor Setup
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  
  // Sensor Setup
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), readEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), readEncoderR, RISING);
}

void loop() {
  // 1. Read Commands from ROS (Non-blocking)
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    parseCommand(cmd);
  }

  // 2. Send Data to ROS @ 20Hz (Every 50ms)
  if (millis() - last_time > 50) {
    int metal_val = analogRead(METAL_PIN);
    
    // Protocol: "L_TICKS,R_TICKS,METAL_VAL"
    Serial.print(left_ticks);
    Serial.print(",");
    Serial.print(right_ticks);
    Serial.print(",");
    Serial.println(metal_val);
    
    last_time = millis();
  }
}

void parseCommand(String cmd) {
  // Format: "LINEAR_PWM,ANGULAR_PWM" (Simplified for direct control)
  // Example: "150,0" (Forward), "0,150" (Spin)
  int commaIndex = cmd.indexOf(',');
  if (commaIndex > 0) {
    int fwd = cmd.substring(0, commaIndex).toInt();
    int rot = cmd.substring(commaIndex + 1).toInt();
    driveMotors(fwd, rot);
  }
}

void driveMotors(int fwd, int rot) {
  int left_speed = fwd - rot;
  int right_speed = fwd + rot;

  // Constrain to PWM range
  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);

  setMotor(IN1, IN2, ENA, left_speed);
  setMotor(IN3, IN4, ENB, right_speed);
}

void setMotor(int p1, int p2, int en, int speed) {
  if (speed > 0) {
    digitalWrite(p1, HIGH); digitalWrite(p2, LOW);
  } else if (speed < 0) {
    digitalWrite(p1, LOW); digitalWrite(p2, HIGH);
  } else {
    digitalWrite(p1, LOW); digitalWrite(p2, LOW);
  }
  analogWrite(en, abs(speed));
}
