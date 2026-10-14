# Fixes Gripper buzz

The log shows the cause. The close command (`66 01`) leaves the motor being driven to hold the position, and the release command (`66 fe`) stops that drive, so the buzz was the hold. That would mean the gripper isn't faulty, and the earlier tests, cable and power checks weren't needed after all.

Here's a small helper to use in your scripts, so the gripper is never left in the buzzing state:

```python
def grip_open(mc):
    mc.set_gripper_state(0, 80, 4)
    time.sleep(2)

def grip_close(mc, hold=1.5, release=True):
    mc.set_gripper_state(1, 80, 4)
    time.sleep(hold)
    if release:
        mc.set_gripper_state(254, 80, 4)   # stop driving the motor
```

## Sample python code:

```python
import os, sys, time
sys.path.append(os.path.dirname(__file__))
from port_setup import setup

mc = setup()
mc.power_on()
time.sleep(1.5)

mc.set_gripper_state(0, 80, 4)  # Open
time.sleep(2)

mc.set_gripper_state(1, 80, 4)  # Close and grip pen
time.sleep(2)

# Leave the gripper closed while holding the pen.
input("Press Enter to release the pen...")

mc.set_gripper_state(0, 80, 4)  # Open and release
time.sleep(2)
```
