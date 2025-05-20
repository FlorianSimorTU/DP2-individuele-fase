import time
import board
import analogio
import pwmio
import math
from adafruit_motor import servo

# --- Pin setup ---
thermistor = analogio.AnalogIn(board.A4)
pwm = pwmio.PWMOut(board.D4, duty_cycle=2 ** 15, frequency=50)
gas_valve = servo.Servo(pwm, min_pulse=500, max_pulse=2500)

# --- Constants for thermistor ---
V_IN = 3.3
R_FIXED = 10000
BETA = 3950
T0 = 297.15  # Kelvin (24°C)
R0 = 10000

def read_voltage(pin):
    return (pin.value * V_IN) / 65535

def voltage_to_temp_c(voltage):
    if voltage <= 0 or voltage >= V_IN:
        return None
    r_therm = R_FIXED * (V_IN - voltage) / voltage
    temp_k = 1 / (1 / T0 + (1 / BETA) * math.log(r_therm / R0))
    return temp_k - 273.15

def heat_to(temp_target, hold_seconds, tolerance=1.0):
    print(f"\n🔄 Heating to {temp_target}°C and holding for {hold_seconds} seconds...\n")
    hold_start = None
    current_angle = 0
    MAX_ANGLE = 170
    MIN_ANGLE = 0
    MAX_DIFF = 5.0

    while True:
        voltage = read_voltage(thermistor)
        temp = voltage_to_temp_c(voltage)

        if temp is None:
            print("⚠️ Invalid temp reading")
            gas_valve.angle = 0
            current_angle = 0
            continue

        temp_diff = temp_target - temp

        if temp_diff > tolerance:
            proportion = min(temp_diff / MAX_DIFF, 1.0)
            target_angle = int(MIN_ANGLE + proportion * (MAX_ANGLE - MIN_ANGLE))
        elif temp_diff < -tolerance:
            target_angle = 0
            hold_start = None
        else:
            target_angle = 0
            if hold_start is None:
                hold_start = time.monotonic()
            elif time.monotonic() - hold_start >= hold_seconds:
                print(f"✅ Held at {temp_target}°C for {hold_seconds} seconds.")
                break

        if target_angle != current_angle:
            gas_valve.angle = target_angle
            current_angle = target_angle

        diff_status = "below target" if temp_diff > 0 else "above target" if temp_diff < 0 else "on target"
        print(f"🌡️ Temp: {temp:.2f}°C | 🔧 Servo angle: {current_angle}° | 📏 Diff: {abs(temp_diff):.2f}°C {diff_status}")

        time.sleep(0.05)

# --- Steps to simulate heating and holding ---
STEPS = [
    {"temp": 27, "hold": 20},
    #{"temp": 25, "hold": 5}
]

while True:
    for step in STEPS:
        heat_to(step["temp"], step["hold"])
    print("✅ Program done.")
    while True:
        gas_valve.angle = 0

time.sleep(0.05)
