from machine import Pin
from motion import Rig

STEPS_PER_MM = 1000
VMAX_MM_S = 1.0
AMAX_MM_S2 = 0.25

# MOTOR0/MOTOR1 driver sockets are typically active-low enable -- drive low to
# enable. Flip value= if your driver boards are wired the other way.
Pin.board.MOTOR0_EN.init(Pin.OUT, value=0)
Pin.board.MOTOR1_EN.init(Pin.OUT, value=0)

axes = [
    (Pin.board.MOTOR0_STEP, Pin.board.MOTOR0_DIR),
    (Pin.board.MOTOR1_STEP, Pin.board.MOTOR1_DIR),
]

rig = Rig(len(axes))
for i, (step_pin, dir_pin) in enumerate(axes):
    rig.stepper(i, step_pin=step_pin, dir_pin=dir_pin)
    rig.scale(i, unit_scale=1 / STEPS_PER_MM)
    rig.rates(i, vmax=VMAX_MM_S, amax=AMAX_MM_S2)
