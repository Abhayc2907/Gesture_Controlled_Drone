import time
import serial
import cflib.crtp
from cflib.crazyflie import Crazyflie

DRONE_URI   = "udp://192.168.43.42"
SER_PORT    = "COM6"
SER_BAUD    = 115200

THRUST_HOVER = 30000          # works for your takeoff
ROLL_MAX     = 8000           # smaller than before
PITCH_MAX    = 8000
DEADZONE     = 3.0            # stronger dead-zone
DT           = 0.03           # 30–35 Hz
SMOOTH_ALPHA = 0.3            # 0..1, lower = smoother

def parse_line(line: str):
    parts = line.split(',')
    if len(parts) != 3:
        return None
    try:
        x = float(parts[0])
        y = float(parts[1])
        trig = parts[2]
        return x, y, trig
    except ValueError:
        return None

def map_tilt_to_cmd(val, max_cmd):
    if abs(val) < DEADZONE:
        return 0
    if val > 10.0:
        val = 10.0
    if val < -10.0:
        val = -10.0
    return int((val / 10.0) * max_cmd)

ser = serial.Serial(SER_PORT, SER_BAUD, timeout=1)
print(f"ESP32 on {SER_PORT}")
time.sleep(1)

cflib.crtp.init_drivers(enable_debug_driver=False)
cf = Crazyflie()
print("Connecting to LiteWing...")
cf.open_link(DRONE_URI)
time.sleep(1.0)
print("Connected")

cf.param.set_value('commander.enHighLevel', '0')
time.sleep(0.2)

print("Arming with zero thrust...")
for _ in range(20):
    cf.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(DT)

print("Ready: fist = fly, open = stop")

# previous smoothed commands
roll_s   = 0
pitch_s  = 0

try:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            time.sleep(DT)
            continue

        data = parse_line(line)
        if not data:
            continue

        gx, gy, trig = data
        trig = trig.strip()

        if trig == "1":
            roll_target  = map_tilt_to_cmd(gx, ROLL_MAX)
            pitch_target = -map_tilt_to_cmd(gy, PITCH_MAX)

            # exponential smoothing to avoid sudden jumps
            roll_s  = int(roll_s  + SMOOTH_ALPHA * (roll_target  - roll_s))
            pitch_s = int(pitch_s + SMOOTH_ALPHA * (pitch_target - pitch_s))

            cf.commander.send_setpoint(roll_s, pitch_s, 0, THRUST_HOVER)
        else:
            roll_s = 0
            pitch_s = 0
            cf.commander.send_setpoint(0, 0, 0, 0)

        time.sleep(DT)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    cf.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(0.2)
    cf.close_link()
    ser.close()
    print("Done")
