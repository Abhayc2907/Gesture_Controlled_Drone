# Gesture_Controlled_Drone

A gesture‑controlled drone that maps hand tilt to roll/pitch commands using an ESP32‑based handheld controller (MPU6050 + LDR) and a LiteWing ESP32 programmable drone. The system uses a Python ground station to read serial gesture data from the ESP32, apply a dead-zone and exponential smoothing, and convert the values into RPYT setpoints over Wi‑Fi. The control script disables high-level commander mode, arms the drone with zero-thrust packets, and then continuously sends either flight commands or an emergency stop depending on the trigger state, enabling low‑altitude, real‑time human–drone interaction for prototyping and education.

Key features
Wearable ESP32 gesture controller with MPU6050 IMU and LDR trigger

Complementary filter for roll/pitch estimation (runs on ESP32)

Python ground station that reads serial data, parses gesture values, and maps them to flight commands

Dead-zone filtering, command clamping, and exponential smoothing for stable control

LiteWing ESP32‑S3 drone controlled over Wi‑Fi using Crazyflie setpoints

Safety: binary emergency stop (open hand), bounded commands, and fail‑safe zero thrust on comms loss

Low‑cost, reproducible design

Hardware
LiteWing ESP32‑S3 drone

ESP32 development board for the gesture controller

MPU6050 6‑axis IMU

LDR with resistor divider for binary trigger detection

LiPo battery for the drone, USB power for the controller

Software
ESP32 firmware (Arduino / PlatformIO) — IMU read, complementary filter, serial output

Python 3 ground station — pyserial, cflib, and timing control

Optional utilities: plotting and logging tools for testing and tuning
