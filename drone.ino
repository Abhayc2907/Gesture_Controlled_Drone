/*
ESP32 Gesture Controller for LiteWing Drone
- MPU6050 (basic Wire library)
- LDR trigger (pin 34)
- USB Serial (COM6) + Bluetooth
- Kalman filter for smooth tracking
*/

#include <Wire.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// MPU6050 I2C address
#define MPU6050_ADDR 0x68
const int LDRSensor = 34; // LDR pin

// Kalman filter variables
float x_angle = 0, y_angle = 0;
float x_bias = 0, y_bias = 0;
float P[2][2] = { { 1, 0 }, { 0, 1 } };
float q_angle = 0.01;
float q_bias = 0.01;
float r_measure = 0.01;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT"); // Bluetooth name
  
  Wire.begin(21, 22); // SDA=21, SCL=22
  
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0);    // Wake up
  Wire.endTransmission(true);
  
  delay(100);
  Serial.println("ESP32 Gesture Controller Ready!");
  Serial.println("Format: X,Y,Trigger");
}

float kalmanFilter(float newAngle, float newRate, float dt, float &angle, float &bias) {
  float rate = newRate - bias;
  angle += dt * rate;
  
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += q_bias * dt;
  
  float S = P[0][0] + r_measure;
  float K[2] = { P[0][0] / S, P[1][0] / S };
  
  float y = newAngle - angle;
  angle += K[0] * y;
  bias += K[1] * y;
  
  P[0][0] -= K[0] * P[0][0];
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];
  
  return angle;
}

void loop() {
  // Read MPU6050 raw data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();
  
  // Convert to physical units (±8g, ±500°/s)
  float accelX = ax / 4096.0;  // ±8g = 4096 LSB/g
  float accelY = ay / 4096.0;
  float gyroX = gx / 65.5;     // ±500°/s = 65.5 LSB/°/s
  float gyroY = gy / 65.5;
  
  // Time for Kalman
  static unsigned long prevTime = millis();
  float dt = (millis() - prevTime) / 1000.0;
  prevTime = millis();
  
  // Apply Kalman filter
  float filteredX = kalmanFilter(accelX, gyroX, dt, x_angle, x_bias);
  float filteredY = kalmanFilter(accelY, gyroY, dt, y_angle, y_bias);
  
  // LDR trigger (dark = fist closed = 0)
  int ldrValue = analogRead(LDRSensor);
  int trigger = (ldrValue < 2000) ? 0 : 1;
  
  // Output format: "X,Y,trigger"
  String data = String(filteredX, 2) + "," + String(filteredY, 2) + "," + String(trigger);
  
  Serial.println(data);
  SerialBT.println(data);
  
  delay(50); // 20Hz update rate
}
