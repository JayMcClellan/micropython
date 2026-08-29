from machine import Pin
from motion import Rig
import math

STEPS_PER_MM = 1000
VMAX_MM_S = 100.0
AMAX_MM_S2 = 1000.0

# MOTOR0/MOTOR1 driver sockets are typically active-low enable -- drive low to
# enable. Flip value= if your driver boards are wired the other way.
Pin.board.MOTOR0_EN.init(Pin.OUT, value=0)
Pin.board.MOTOR1_EN.init(Pin.OUT, value=0)
Pin.board.MOTOR2_EN.init(Pin.OUT, value=0)
Pin.board.MOTOR3_EN.init(Pin.OUT, value=0)
Pin.board.MOTOR4_EN.init(Pin.OUT, value=0)
Pin.board.MOTOR5_EN.init(Pin.OUT, value=0)
Pin.board.MOTOR6_EN.init(Pin.OUT, value=0)
#Pin.board.MOTOR7_EN.init(Pin.OUT, value=0)   # MOTOR7 conflicts with SWD

axes = [
    (Pin.board.MOTOR0_STEP, Pin.board.MOTOR0_DIR),
    (Pin.board.MOTOR1_STEP, Pin.board.MOTOR1_DIR),
    (Pin.board.MOTOR2_STEP, Pin.board.MOTOR2_DIR),
    (Pin.board.MOTOR3_STEP, Pin.board.MOTOR3_DIR),
    (Pin.board.MOTOR4_STEP, Pin.board.MOTOR4_DIR),
    (Pin.board.MOTOR5_STEP, Pin.board.MOTOR5_DIR),
    (Pin.board.MOTOR6_STEP, Pin.board.MOTOR6_DIR),
    (Pin.board.EXP2_1, Pin.board.EXP2_2), # MOTOR7 conflicts with SWD
]

rig = Rig(len(axes), q_depth=20)
for i, (step_pin, dir_pin) in enumerate(axes):
    rig.stepper(i, step_pin=step_pin, dir_pin=dir_pin, pulse_us=1, low_min_us=1, dir_setup_us=1, dir_hold_us=1)
    rig.scale(i, unit_scale=1 / STEPS_PER_MM)
    rig.rates(i, vmax=VMAX_MM_S, amax=AMAX_MM_S2)

def yoyo():
  for i in range(10):
      while rig.queue_avail() < 1:
          pass
      rig.move([100,101,102,103,104,105,106,107])
      rig.move([0,0,0,0,0,0,0,0])

