# ============================================================
# PORT SCANNER — detects what's plugged into each port.
# Motors: wiggles +10 / -10 degrees so you can SEE which is which.
# Sensors: prints type and a live reading.
# ============================================================

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Port
from pybricks.tools import wait

hub = PrimeHub()

PORTS = [Port.A, Port.B, Port.C, Port.D, Port.E, Port.F]

print("=== PORT SCAN START ===")

for port in PORTS:
    name = str(port)

    # --- Try motor first ---
    try:
        m = Motor(port)
        print(name, ": MOTOR  -> wiggling now, watch the robot!")
        wait(1000)
        m.run_angle(200, 10)    # +10 degrees
        wait(300)
        m.run_angle(200, -10)   # back to start
        wait(1500)
        continue
    except OSError:
        pass

    # --- Try color sensor ---
    try:
        s = ColorSensor(port)
        print(name, ": COLOR SENSOR -> reading:", s.color(), s.hsv())
        continue
    except OSError:
        pass

    # --- Try ultrasonic sensor ---
    try:
        u = UltrasonicSensor(port)
        print(name, ": ULTRASONIC SENSOR -> distance:", u.distance(), "mm")
        continue
    except OSError:
        pass

    # --- Try force sensor ---
    try:
        f = ForceSensor(port)
        print(name, ": FORCE SENSOR -> force:", f.force(), "N")
        continue
    except OSError:
        pass

    print(name, ": (empty)")

print("=== PORT SCAN COMPLETE ===")
