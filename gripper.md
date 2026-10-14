# Fixes Gripper buzz

Glad that finally worked. The log shows the cause. The close command (`66 01`) leaves the motor being driven to hold the position, and the release command (`66 fe`) stops that drive, so the buzz was the hold. That would mean the gripper isn't faulty, and the earlier tests, cable and power checks weren't needed after all. That's my reading of the result and I haven't seen documentation confirming it, but it fits everything you've seen.

The thing I don't know yet is what release does to the grip. Your message doesn't say whether the fingers stayed closed or sprang open afterwards. That matters in practice, because if release lets go, you can't release to silence it while holding something. Test it with a soft object between the fingers (a foam ball or eraser): close, wait a second, release, and see whether it drops. Keep your hand near the object and the switch while you do.

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

try:
    mc.set_gripper_state(0, 80, 4)       # open
    time.sleep(2)
    mc.set_gripper_state(1, 80, 4)       # close, buzz expected
    time.sleep(1.5)
    mc.set_gripper_state(254, 80, 4)     # release
    time.sleep(2)
    input("Did the buzz stop after release? Enter to finish...")
finally:
    mc.set_gripper_state(0, 80, 4)       # always end open
    time.sleep(2)
```
