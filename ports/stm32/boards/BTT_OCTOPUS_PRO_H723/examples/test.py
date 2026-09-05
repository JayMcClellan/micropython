from array import array

from machine import Pin
from motion import Rig
from rigsvg import RigSVG
import gcode

import math
import micropython

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

        speed = max_speed if deg_to_end > stop_deg else max(max_speed * deg_to_end / stop_deg, 0.01)
        rig.move([x - radius, y], speed=speed)
    while rig.is_running():
        pass

def square(side=10, blend = 1):
    max_speed = VMAX_MM_S

    rig.pause()

    for n in range(4):
        while rig.queue_avail() < 4:
            pass
        rig.move([side, 0], speed=max_speed, entry_blend=blend, exit_blend=blend)
        rig.move([side, side], speed=max_speed, entry_blend=blend, exit_blend=blend)
        rig.move([0, side], speed=max_speed, entry_blend=blend, exit_blend=blend)
        rig.move([0, 0], speed=max_speed, entry_blend=blend, exit_blend=blend)
        if n == 0:
            rig.resume()
    while rig.is_running():
        pass

def svg(side=10, blend = 1):
    rig = Rig(2, q_depth=20, hardware_timer=False)
    rig.stepper(0, unit_scale=1/1000, vmax=10.0, amax=20.0)
    rig.stepper(1, unit_scale=1/1000, vmax=10.0, amax=20.0)
    rig.set_position([side/2, 0])

    f = open(f"square_{side}_{blend}.svg", "w")
    svg = RigSVG(rig, f)
    svg.move([side, 0], entry_blend=0, exit_blend=blend); 
    svg.move([side, side], entry_blend=blend, exit_blend=blend); 
    svg.move([0, side], entry_blend=blend, exit_blend=blend); 
    svg.move([0, 0], entry_blend=blend, exit_blend=blend)
    svg.move([side/2, 0], entry_blend=blend, exit_blend=0)
    svg.close()
    f.close()

def arcs(radius=10, segments=24):
    rig = Rig(2, q_depth=40, hardware_timer=False)
    rig.stepper(0, unit_scale=1/1000, vmax=10.0, amax=20.0)
    rig.stepper(1, unit_scale=1/1000, vmax=10.0, amax=20.0)
    rig.set_position([radius, 0])

    f = open(f"circle_{radius}_{segments}.svg", "w")
    svg = RigSVG(rig, f)
    # Two CCW semicircles about the origin: (r,0) over the top to (-r,0),
    # then under the bottom back to (r,0). The joins are tangent, no blend.
    svg.arc([-radius, 0], radius, ccw=True, segments=segments)
    svg.arc([radius, 0], radius, ccw=True, segments=segments)
    svg.close()
    f.close()

def gcode_square(rig, side=10, blend=1):
    start = array('f', [0.0, 0.0])
    rig.get_trajectory(pos = start)
    gc = gcode.Parser(rig)
    gc.parse(f"G64 P{blend}")
    for _ in range(2):
        gc.parse(f"G1 X{start[0] + side}"); 
        gc.parse(f"G1 Y{start[1] + side}"); 
        gc.parse(f"G1 X{start[0]}"); 
        gc.parse(f"G1 Y{start[1]}")

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

        speed = max_speed if deg_to_end > stop_deg else max(max_speed * deg_to_end / stop_deg, 0.01)
        rig.move([x - radius, None, y], speed=speed)
    while rig.is_running():
        pass