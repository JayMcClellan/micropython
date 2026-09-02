"""Stream a soft-timed Rig's XY path to an SVG stream.

RigSVG wraps a Rig built with hardware_timer=False, exposes the normal Rig API
(move/dwell/stepper/...), and drives the update loop itself -- writing the path
channels 0 and 1 trace out to an SVG <path> as motion proceeds. close() finishes
the file (running the queue to rest first) and drops a red marker at every XY
destination that was commanded. It never closes the stream.

    from motion import Rig
    from rigsvg import RigSVG

    rig = Rig(2, q_depth=20, hardware_timer=False)
    rig.stepper(0, unit_scale=1/1000, vmax=10.0, amax=20.0)
    rig.stepper(1, unit_scale=1/1000, vmax=10.0, amax=20.0)

    f = open("/flash/square.svg", "w")
    svg = RigSVG(rig, f)
    for _ in range(2):
        svg.move([10, 0]); svg.move([10, 10]); svg.move([0, 10]); svg.move([0, 0])
    svg.close()
    f.close()

Coordinates are cartesian (y up) via a scale(1,-1) group. There is no viewBox --
it can't be computed while streaming into a write-only stream -- so a viewer must
auto-fit or the file is post-processed.
"""

from array import array
import micropython

class RigSVG:
    def __init__(self, rig, stream):
        if rig.n_channels < 2:
            raise ValueError("RigSVG needs a rig with at least 2 channels")
        if rig.hardware_timer:
            raise ValueError("RigSVG needs a soft-timed Rig (hardware_timer=False)")

        self._rig = rig
        self._out = stream
        self._now = 0
        self._pos = array("f", bytes(4 * rig.n_channels))
        self._dests = []
        self._closed = False

        # A new path point is written only once some coordinate has moved more
        # than this from the last one written. Constant for now.
        self.epsilon = 0.1

        # Style -- plain fields for now; params may come later.
        self.path_stroke = "blue"
        self.path_stroke_width = 0.1
        self.marker_stroke = "red"
        self.marker_stroke_width = 0.1
        self.marker_fill = "none"
        self.marker_diameter = 2.0

        self._dot_format = ('<circle cx="%%.4f" cy="%%.4f" r="%.4f" fill="%s" stroke="%s" stroke-width="%.4f" />\n'
              % (self.marker_diameter / 2.0, self.marker_fill,
                 self.marker_stroke, self.marker_stroke_width))
        
        self._write_header()
    
    def _write_header(self):
        w = self._out.write
        w('<?xml version="1.0" encoding="UTF-8"?>\n')
        w('<svg xmlns="http://www.w3.org/2000/svg">\n')
        #w('<g transform="scale(1,-1)">\n')
        w('<path fill="none" stroke="%s" stroke-width="%.4f" d="'
          % (self.path_stroke, self.path_stroke_width))

        self._rig.get_trajectory(position=self._pos)
        xy = (self._pos[0], self._pos[1])
        w("M%.4f %.4f" % (xy[0], xy[1]))
        self._last = xy
        self._dest_xy = xy

    @micropython.native
    def _sample(self):
        self._rig.get_trajectory(position=self._pos)
        x = self._pos[0]
        y = self._pos[1]
        lx, ly = self._last
        if abs(x - lx) > self.epsilon or abs(y - ly) > self.epsilon:
            self._out.write("L%.4f %.4f" % (x, y))
            self._last = (x, y)

    @micropython.native
    def _pump_until(self, cond):
        r = self._rig
        while cond():
            self._now += r.update(self._now)
            self._sample()

    # --- wrapped Rig API -----------------------------------------------------
    @micropython.native
    def move(self, target, **kw):
        r = self._rig
        # Drain down to at most one outstanding move so they don't accumulate,
        # while still keeping the executing move + this new one queued together
        # for corner blending.
        self._pump_until(lambda: (r.q_depth - r.queue_avail()) > 1)
        r.move(target, **kw)

        x, y = self._dest_xy
        if len(target) > 0 and target[0] is not None:
            x = target[0]
        if len(target) > 1 and target[1] is not None:
            y = target[1]
        self._dest_xy = (x, y)
        self._dests.append((x, y))

    @micropython.native
    def dwell(self, duration, **kw):
        r = self._rig
        self._pump_until(lambda: (r.q_depth - r.queue_avail()) > 1)
        r.dwell(duration, **kw)


    @micropython.native
    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)
        return getattr(self._rig, name)

    # --- finish ------------------------------------------------------------
    @micropython.native
    def close(self):
        if self._closed:
            return
        self._closed = True

        self._pump_until(self._rig.is_running)
        self._sample()

        w = self._out.write
        w('" />\n')

        for x, y in self._dests:
            w(self._dot_format % (x, y))

        w("</g>\n</svg>\n")
