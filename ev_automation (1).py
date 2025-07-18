"""
EV Automation with cost-based AI planning (Fast Downward) + Firebase logging
"""

import RPi.GPIO as GPIO
import time
import serial
import re
import Adafruit_DHT
from luma.core.interface.serial import spi
from luma.core.render import canvas
from luma.oled.device import sh1107
from PIL import ImageFont
import subprocess
import sys
from pathlib import Path

# ──────────────────────────────────────────────────────────────────────────────
# Firebase Setup
# ──────────────────────────────────────────────────────────────────────────────
import firebase_admin
from firebase_admin import credentials, db

cred = credentials.Certificate("firebase_config.json")
firebase_admin.initialize_app(
    cred,
    {"databaseURL": "https://iot-project-5d8f9-default-rtdb.europe-west1.firebasedatabase.app/"},
)
firebase_ref = db.reference("ev_monitor")

# ──────────────────────────────────────────────────────────────────────────────
# OLED Setup
# ──────────────────────────────────────────────────────────────────────────────
serial_interface = spi(device=0, port=0, gpio_DC=25, gpio_RST=24)
device = sh1107(serial_interface, width=128, height=128)
font_large = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 12)
font_small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 9)

def update_display(charging, voltage, percentage, temperature, rpm, motor_state, ac_state, info_state):
    with canvas(device) as draw:
        draw.rectangle((0, 0, 127, 127), outline=255)
        for y in range(0, 128, 4):
            draw.point((0, y), fill=0)
            draw.point((127, y), fill=0)
        for x in range(0, 128, 4):
            draw.point((x, 0), fill=0)
            draw.point((x, 127), fill=0)

        draw.text((4, 2), "EV Charging Status", font=font_large, fill=255)
        draw.line((0, 18, 127, 18), fill=255)

        draw.text((4, 22), "CHARGING:", font=font_small, fill=255)
        draw.text((80, 22), charging, font=font_small, fill=255)
        draw.text((110, 22), "⚡", font=font_small, fill=255)

        draw.text((4, 32), "VOLTAGE", font=font_small, fill=255)
        draw.text((80, 32), f"{voltage:4.2f}V", font=font_small, fill=255)

        draw.text((4, 42), "BATTERY", font=font_small, fill=255)
        draw.text((80, 42), f"{percentage}%", font=font_small, fill=255)
        draw.text((110, 42), "▯▯▯", font=font_small, fill=255)

        draw.text((4, 52), "TEMP", font=font_small, fill=255)
        draw.text((80, 52), f"{temperature:.1f}°C", font=font_small, fill=255)

        draw.line((0, 62, 127, 62), fill=255)

        draw.text((4, 66), "MOTOR", font=font_small, fill=255)
        draw.text((80, 66), motor_state.upper(), font=font_small, fill=255)

        draw.text((4, 76), "AC", font=font_small, fill=255)
        draw.text((80, 76), ac_state.upper(), font=font_small, fill=255)
        draw.text((110, 76), "🌀", font=font_small, fill=255)

        draw.line((0, 86, 127, 86), fill=255)

        draw.text((4, 90), "INFOTAIN", font=font_small, fill=255)
        draw.text((80, 90), info_state.upper(), font=font_small, fill=255)
        draw.text((110, 90), "🔊", font=font_small, fill=255)

        draw.text((4, 110), "MOT", font=font_small, fill=255)
        draw.text((40, 108), f"{rpm:4d}", font=font_large, fill=255)
        draw.text((90, 110), "rpm", font=font_small, fill=255)

# ──────────────────────────────────────────────────────────────────────────────
# GPIO & Sensors
# ──────────────────────────────────────────────────────────────────────────────
IR_SENSOR_PIN = 22
HALL_SENSOR_PIN = 26
IN1, IN2, ENA = 17, 27, 18
LED_PIN = 23
DHT_SENSOR, DHT_PIN = Adafruit_DHT.DHT22, 19
SERIAL_PORT, BAUD_RATE = "/dev/ttyACM0", 9600
UART_RE = re.compile(r"Voltage:(\d+(?:\.\d+)?),Battery:(\d+)")

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_SENSOR_PIN, GPIO.IN)
GPIO.setup(HALL_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup([IN1, IN2, ENA], GPIO.OUT)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.output(IN1, GPIO.HIGH)
GPIO.output(IN2, GPIO.LOW)
pwm = GPIO.PWM(ENA, 1000)
pwm.start(0)

pulse_count, last_rpm_time = 0, time.time()
state_vars = {"motor": "off", "ac": "off", "info": "off"}

def pulse_cb(_):
    global pulse_count
    pulse_count += 1

GPIO.add_event_detect(HALL_SENSOR_PIN, GPIO.FALLING, callback=pulse_cb, bouncetime=10)

def parse_uart(line):
    m = UART_RE.search(line)
    if m:
        return float(m.group(1)), int(m.group(2))
    return 0.0, 0

def read_temperature():
    _, t = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)
    return t or 0.0

def rpm_now():
    global pulse_count, last_rpm_time
    now, rpm = time.time(), 0
    if now - last_rpm_time >= 1:
        rpm = pulse_count * 60
        pulse_count, last_rpm_time = 0, now
    return rpm

def batt_state(pct):
    if pct >= 100:
        return "battery_full"
    if pct >= 71:
        return "battery_high"
    if pct >= 36:
        return "battery_medium"
    if pct >= 20:
        return "battery_low"
    return "battery_extremely_low"

def temp_state(temp):
    if temp < 45.0:
        return "temp_safe"
    elif temp < 55.0:
        return "temp_high"
    else:
        return "temp_critical"

# ──────────────────────────────────────────────────────────────────────────────
# PDDL generation - UPDATED FOR NEW DOMAIN.PDDL
# ──────────────────────────────────────────────────────────────────────────────
PROBLEM_FILE = Path("problem.pddl")

def write_problem(batt_st, temp_st, charging):
    with PROBLEM_FILE.open("w") as f:
        f.write("(define (problem ev_problem)\n"
                "  (:domain ev_charging)\n"
                "  (:init\n"
                "    (= (total-cost) 0)\n")
        
        # Add all battery states (explicitly setting one true and others false)
        battery_states = ["battery_extremely_low", "battery_low", 
                         "battery_medium", "battery_high", "battery_full"]
        for state in battery_states:
            f.write(f"    ({state})" if state == batt_st else f"    (not ({state}))")
            f.write("\n")
        
        # Add temperature state
        f.write(f"    ({'temperature_high' if temp_st == 'temp_high' else 'not (temperature_high)'})\n")
        
        # Add charging status
        if charging:
            f.write("    (charging)\n")
        
        # Initial device states
        f.write("    (motor_off)\n    (infotainment_off)\n    (ac_off)\n  )\n")

        # Goal conditions based on battery state (UPDATED)
        goals = {
            "battery_extremely_low": "(and (motor_off) (infotainment_off) (ac_off))",
            "battery_low": "(and (motor_low) (infotainment_on) (ac_off))",
            "battery_medium": "(and (or (motor_low) (motor_medium)) (infotainment_on) (ac_on))",
            "battery_high": "(and (or (motor_medium) (motor_full)) (infotainment_on) (ac_on))",
            "battery_full": "(and (motor_full) (infotainment_on) (ac_on))",
        }

        goal_str = "(and (motor_off) (infotainment_off) (ac_off))" if temp_st == "temp_critical" else goals[batt_st]

        f.write(f"  (:goal {goal_str})\n")
        f.write("  (:metric minimize (total-cost))\n)\n")

# ──────────────────────────────────────────────────────────────────────────────
# Planning & Action Application (UNCHANGED)
# ──────────────────────────────────────────────────────────────────────────────
def run_planner():
    cmd = [
        sys.executable,
        "fast-downward/fast-downward.py",
        "domain.pddl",
        "problem.pddl",
        "--search",
        "lazy_wastar([ff()],w=1)"
    ]
    print("[PLANNER] →", " ".join(cmd))
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=20)

    plan_file = Path("sas_plan")
    if not plan_file.exists():
        print("[PLANNER] No sas_plan produced.")
        return []

    actions = []
    with plan_file.open() as pf:
        for ln in pf:
            ln = ln.strip().lower()
            if ln and not ln.startswith(";"):
                act = ln.strip("()").split()[0]
                actions.append(act)
    return actions

def apply_actions(plan):
    for act in plan:
        if act == "set_motor_off":
            state_vars["motor"] = "off"
        elif act == "set_motor_low":
            state_vars["motor"] = "low"
        elif act == "set_motor_medium":
            state_vars["motor"] = "medium"
        elif act == "set_motor_full":
            state_vars["motor"] = "full"
        elif act == "set_ac_on":
            state_vars["ac"] = "on"
        elif act == "set_ac_off":
            state_vars["ac"] = "off"
        elif act == "set_infotainment_on":
            state_vars["info"] = "on"
        elif act == "set_infotainment_off":
            state_vars["info"] = "off"

    duty = {"off": 0, "low": 40, "medium": 60, "full": 100}[state_vars["motor"]]
    pwm.ChangeDutyCycle(duty)

    print(f"[ACTION] Motor={state_vars['motor']}({duty}%), AC={state_vars['ac']}, Infotainment={state_vars['info']}")

# ──────────────────────────────────────────────────────────────────────────────
# Main Loop (UNCHANGED)
# ──────────────────────────────────────────────────────────────────────────────
def main():
    print("=== EV automation with AI planning + Firebase logging ===")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)

    while True:
        charging = GPIO.input(IR_SENSOR_PIN) == GPIO.LOW
        if charging and ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()
            volts, batt = parse_uart(line)
        else:
            volts, batt = 0.0, 0

        temp = read_temperature()
        rpm = rpm_now()
        batt_st = batt_state(batt)
        temp_st = temp_state(temp)

        if temp_st == "temp_safe":
            temp_status = "Temperature is Safe"
        elif temp_st == "temp_high":
            temp_status = "Temperature is High"
        else:
            temp_status = "Temperature is Critical!"

        print(f"[SENSE] batt={batt}% V={volts:.2f} temp={temp:.1f} ({temp_st}) rpm={rpm} charging={charging} state={batt_st}")

        if charging:
            GPIO.output(LED_PIN, GPIO.HIGH)
            write_problem(batt_st, temp_st, charging)
            plan = run_planner()
            apply_actions(plan if plan else ["set_motor_off", "set_ac_off", "set_infotainment_off"])
        else:
            print("[IDLE] Not charging; turning everything off.")
            apply_actions(["set_motor_off", "set_ac_off", "set_infotainment_off"])

        update_display(
            "ON" if charging else "OFF",
            volts,
            batt,
            temp,
            rpm,
            state_vars["motor"],
            state_vars["ac"],
            state_vars["info"]
        )
        
        firebase_ref.update({
            "Charging": charging,
            "Battery Percentage": batt,
            "Voltage": volts,
            "Temperature": temp,
            "Temprature Status": temp_status,
            "Motor Speed in RPM": rpm,
            "Motor state": state_vars["motor"],
            "Air-Conditioning": state_vars["ac"],
            "Infotainment": state_vars["info"],
            "TimeStamp": time.strftime("%Y-%m-%d %H:%M:%S")
        })

        time.sleep(1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[EXIT] User interrupted")
    finally:
        pwm.stop()
        GPIO.cleanup()
