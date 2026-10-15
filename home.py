# Homing script
# (convert to Rust!)

import time
from pymycobot.mycobot280 import MyCobot280

PORT = "/dev/ttyAMA0"
BAUD = 1000000
SPEED = 20            # slow on purpose: start pose is unknown
TOL = 3               # degrees of tolerance
TIMEOUT = 40          # seconds to wait for the move to finish
TARGET = [0, 0, 0, 0, 0, 0]

# What to do with the gripper at the end, then motor off.
GRIPPER_ACTION = "close"   # "close" or "open"
GRIPPER_SPEED = 80
GRIPPER_WAIT = 0           # seconds between the move command and motor off


def connect(retries=10, delay=2):
    """The Atom may not be ready right at boot, so retry."""
    for i in range(retries):
        try:
            mc = MyCobot280(PORT, BAUD, debug=False)
            time.sleep(0.5)
            if mc.is_controller_connected() == 1:
                return mc
        except Exception as e:
            print(f"Connect attempt {i+1} failed: {e}")
        time.sleep(delay)
    raise RuntimeError("Could not connect to myCobot")


def at_target(mc):
    angles = mc.get_angles()
    if not angles or angles == -1 or len(angles) != 6:
        return False
    return all(abs(a - t) <= TOL for a, t in zip(angles, TARGET))


def gripper_then_motor_off(mc):
    flag = 1 if GRIPPER_ACTION == "close" else 0
    mc.set_gripper_state(flag, GRIPPER_SPEED, 4)    # close (1) or open (0)
    time.sleep(GRIPPER_WAIT)
    mc.set_gripper_state(254, GRIPPER_SPEED, 4)     # gripper motor off
    time.sleep(0.3)


mc = connect()
print("Connected")

mc.power_on()
time.sleep(1.5)

print("Start angles:", mc.get_angles())

mc.send_angles(TARGET, SPEED)

start = time.time()
while time.time() - start < TIMEOUT:
    if at_target(mc):
        break
    time.sleep(0.5)
else:
    # Not there yet: resend once, since a command can be dropped right after power-up
    mc.send_angles(TARGET, SPEED)
    time.sleep(10)

gripper_then_motor_off(mc)

print("Final angles:", mc.get_angles())
print("Error info:", mc.get_error_information())
print("Homed" if at_target(mc) else "WARNING: did not reach zero")
