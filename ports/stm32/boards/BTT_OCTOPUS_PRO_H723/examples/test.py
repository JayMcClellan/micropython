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
Pin.board.MOTOR4_EN.init(Pin.OUT, value=0)

axes = [
    (Pin.board.MOTOR0_STEP, Pin.board.MOTOR0_DIR),
    (Pin.board.MOTOR1_STEP, Pin.board.MOTOR1_DIR),
    (Pin.board.MOTOR4_STEP, Pin.board.MOTOR4_DIR),
]

rig = Rig(len(axes), q_depth=20)
for i, (step_pin, dir_pin) in enumerate(axes):
    rig.stepper(i, step_pin=step_pin, dir_pin=dir_pin, unit_scale= 1/STEPS_PER_MM, vmax=VMAX_MM_S, amax=AMAX_MM_S2)

def yoyo():
  for i in range(10):
      while rig.queue_avail() < 1:
          pass
      rig.move([10,10])
      rig.move([0,0])

def circle():
    radius = 10
    stop_deg = 45
    max_speed = VMAX_MM_S

    end = 359
    for deg in range(end + 1):
        rad = deg * math.pi / 180.0
        x = radius * math.cos(rad)
        y = radius * math.sin(rad)
        while rig.queue_avail() < 1:
            pass
        deg_to_end = min(deg, end - deg)

        speed = max_speed if deg_to_end > stop_deg else max_speed * deg_to_end / stop_deg
        rig.move([x - radius, y], cruise_speed=speed)
    while rig.is_running():
        pass

def yaw():
    radius = 10
    stop_deg = 45
    max_speed = VMAX_MM_S

    end = 359
    for deg in range(end + 1):
        rad = deg * math.pi / 180.0
        x = radius * math.cos(rad)
        y = radius * math.sin(rad)
        while rig.queue_avail() < 1:
            pass
        deg_to_end = min(deg, end - deg)

        speed = max_speed if deg_to_end > stop_deg else max_speed * deg_to_end / stop_deg
        rig.move([x - radius, None, y], cruise_speed=speed)
    while rig.is_running():
        pass