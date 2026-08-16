from machine import Pin
from motion import Rig
import math

STEPS_PER_MM = 1000
VMAX_MM_S = 10.0
AMAX_MM_S2 = 20

# MOTOR0/MOTOR1 driver sockets are typically active-low enable -- drive low to
# enable. Flip value= if your driver boards are wired the other way.
Pin.board.MOTOR0_EN.init(Pin.OUT, value=0)
Pin.board.MOTOR1_EN.init(Pin.OUT, value=0)

axes = [
    (Pin.board.MOTOR0_STEP, Pin.board.MOTOR0_DIR),
    (Pin.board.MOTOR1_STEP, Pin.board.MOTOR1_DIR),
]

rig = Rig(len(axes), n_segs=20)
for i, (step_pin, dir_pin) in enumerate(axes):
    rig.stepper(i, step_pin=step_pin, dir_pin=dir_pin)
    rig.scale(i, unit_scale=1 / STEPS_PER_MM)
    rig.rates(i, vmax=VMAX_MM_S, amax=AMAX_MM_S2)

def yoyo():
  for i in range(10):
      while rig.get_queue_free() < 6:
          pass
      rig.move([10,10])
      rig.move([0,0])

def circle():
    radius = 10
    stop_deg = 45
    rig.move([radius,0])
    while rig.is_running():
        pass
    for deg in range(360):
        rad = deg * math.pi / 180.0
        x = radius * math.cos(rad)
        y = radius * math.sin(rad)
        while rig.get_queue_free() < 2:
            pass
        deg_to_end = min(deg, 359 - deg)
        
        speed = VMAX_MM_S if deg_to_end > stop_deg else VMAX_MM_S * deg_to_end / stop_deg
        rig.segment((x,y), speed)
    while rig.is_running():
        pass
    rig.move([0,0], VMAX_MM_S)
