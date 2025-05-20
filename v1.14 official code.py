import time
import board
import pwmio
import digitalio
import adafruit_ds18x20
import adafruit_onewire.bus
from adafruit_motor import servo

# --- DS18B20 Setup ---
ow_bus = adafruit_onewire.bus.OneWireBus(board.A4)
devices = ow_bus.scan()
if not devices:
    raise RuntimeError("No DS18B20 sensor found!")
ds18 = adafruit_ds18x20.DS18X20(ow_bus, devices[0])

# --- Servo setup (TowerPro SG90-HV continuous rotation) ---
pwm = pwmio.PWMOut(board.D4, duty_cycle=2 ** 15, frequency=50)
gas_valve = servo.ContinuousServo(pwm, min_pulse=500, max_pulse=2500)

# --- Button setup for manual/automatic toggle ---
button = digitalio.DigitalInOut(board.D2)  # Change pin if needed
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.UP  # Internal pull-up, button connects to GND when pressed

# --- Constants ---
MAX_ANGLE = 485.0   # Max valve rotation in degrees
MOVEMENT_SPEED = 0.2  # Throttle value (0.0 to 1.0)
TIME_PER_DEGREE_FORWARD = 0.013  # Time per degree when opening (tuned)
TIME_PER_DEGREE_BACKWARD = 0.011  # Time per degree when closing (tune if needed)
MAX_DIFF = 5.0  # Max temperature difference for proportional control
DEBOUNCE_TIME = 0.2  # Seconds to debounce button

# Track current position, mode, and button state
current_angle = 0.0
manual_mode = True  # Start in manual mode
last_button_state = True  # Assume button not pressed initially (pull-up)

def move_servo_to_angle(target_angle):
    """Moves the valve servo to a given angle with bounds checking."""
    global current_angle
    target_angle = max(0.0, min(target_angle, MAX_ANGLE))
    delta = target_angle - current_angle

    if abs(delta) < 1.0:
        gas_valve.throttle = 0.0
        return

    direction = -1 if delta > 0 else 1
    time_per_degree = TIME_PER_DEGREE_FORWARD if delta > 0 else TIME_PER_DEGREE_BACKWARD
    gas_valve.throttle = MOVEMENT_SPEED * direction
    time.sleep(abs(delta) * time_per_degree)
    gas_valve.throttle = 0.0

    current_angle = target_angle

def heat_to(temp_target, hold_seconds, tolerance=1.0):
    """Heat to target temperature and hold for specified time."""
    global manual_mode, last_button_state
    print(f"\n🔄 Heating to {temp_target}°C and holding for {hold_seconds} seconds...\n")
    hold_start = None

    while True:
        # Check for button press to switch to manual mode
        current_button_state = button.value
        if last_button_state and not current_button_state:  # Button pressed (True -> False)
            time.sleep(DEBOUNCE_TIME)  # Debounce
            if not button.value:  # Confirm stable press
                print("🛠️ Switching to manual mode.")
                move_servo_to_angle(0.0)  # Close valve
                manual_mode = True
                last_button_state = button.value
                return
        last_button_state = current_button_state

        try:
            temp = ds18.temperature
        except Exception as e:
            print("⚠️ Temp read error:", e)
            move_servo_to_angle(0.0)  # Close valve
            continue

        temp_diff = temp_target - temp

        if abs(temp_diff) <= tolerance:
            if hold_start is None:
                hold_start = time.monotonic()
            else:
                held_time = time.monotonic() - hold_start
                if held_time >= hold_seconds:
                    print(f"✅ Held at {temp_target}°C for {hold_seconds} seconds.")
                    return
        else:
            hold_start = None

        # Adjust valve only if temperature is outside tolerance
        if temp_diff > tolerance:
            proportion = min(temp_diff / MAX_DIFF, 5.0)
            target_angle = proportion * MAX_ANGLE
        elif temp_diff < -tolerance:
            target_angle = 0.0
            move_servo_to_angle(0.0)  # Close valve
        else:
            target_angle = current_angle

        move_servo_to_angle(target_angle)

        diff_status = (
            "below target" if temp_diff > 0 else
            "above target" if temp_diff < 0 else
            "on target"
        )

        print(
            f"🌡️ Temp: {temp:.2f}°C | 🔄 Valve angle: {current_angle:.1f}° | 📏 Diff: {abs(temp_diff):.2f}°C {diff_status} | ⏱️ Held: {(time.monotonic() - hold_start):.1f}s"
            if hold_start else
            f"🌡️ Temp: {temp:.2f}°C | 🔄 Valve angle: {current_angle:.1f}° | 📏 Diff: {abs(temp_diff):.2f}°C {diff_status} | ⏱️ Held: 0.0s"
        )

        time.sleep(0.01)

# --- Heating steps ---
STEPS = [
    {"temp": 90, "hold": 20},
]

# Start directly in manual mode without initial calibration
gas_valve.throttle = 0.0  # Ensure servo is stopped

while True:
    if manual_mode:
        print("🛠️ Manual mode: Valve stopped. Press button to start automatic mode.")
        gas_valve.throttle = 0.0  # Allow manual adjustment
        while manual_mode:
            current_button_state = button.value
            if last_button_state and not current_button_state:  # Button pressed (True -> False)
                time.sleep(DEBOUNCE_TIME)  # Debounce
                if not button.value:  # Confirm stable press
                    print("🔄 Switching to automatic mode.")
                    manual_mode = False
                    last_button_state = button.value
                    break
            last_button_state = current_button_state
            time.sleep(0.01)
    else:
        print("🤖 Automatic mode: Starting temperature control.")
        move_servo_to_angle(0.0)  # Set valve to 0°
        time.sleep(1)  # Settle

        for step in STEPS:
            heat_to(step["temp"], step["hold"])
            if manual_mode:  # Exit if switched to manual
                break
