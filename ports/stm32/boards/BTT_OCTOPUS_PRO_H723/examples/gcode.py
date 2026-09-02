"""A small G-code front end for a motion.Rig.

Supported now:

    G0 / G1   linear move   -- axis words (XYZABCUVW) + optional F
    G4        dwell         -- P milliseconds or S seconds
    M400      wait for all queued motion to finish

A non-blank line must start with one of those codes; everything after it is that
code's arguments, in any order, and an argument the code doesn't accept is an
error. Axis letters map to Rig channels 0.. in the order of the `axes` string
(default "XYZABCUVW"); the axis count is min(len(axes), rig.n_channels) and any
extra characters in `axes` are ignored. Only G0/G1 take axis words. Blank lines
and comments (`; ...` and `( ... )`) are skipped.
N line numbers and `*` checksums are not supported.

Each code is handled by a method named exactly like the code (G0, G4, M400),
looked up by name, so adding a code is just adding a method.

Feed rate F is mm/min and is passed to rig.move() as speed = F / 60, so the
rig's channels are assumed to be scaled to millimetres. G0 and G1 keep separate
stored speeds (self.linear_speed[0] and [1]).

    import gcode

    gc = gcode.Parser(rig)                 # rig: a configured motion.Rig
    gc.parse("G0 F1000 ; rapid rate")
    gc.parse("G0 X10 Y20 ; go to start")
    gc.parse("G1 Z5 F300")
    gc.parse("G1 X10 Y20 Z0")
    gc.parse("G4 P500 (dwell 0.5 s)")
    gc.parse("M400")
    gc.finish()

parse() blocks until the rig can take another line. For cooperative code use
AsyncParser, whose parse()/finish() are coroutines.
"""

import time
from micropython import const
import asyncio

DEFAULT_SPEED = 1000.0 / 60.0   # mm/sec, used for G0 and G1 until an F word sets them
POLL_MS = const(1)                        # spin interval while waiting on the rig
READY_SLOTS = const(2)                 # ready once queue_avail() exceeds this

_CH_SEMICOLON = const(ord(";"))
_CH_PAREN1 = const(ord("("))
_CH_PAREN2 = const(ord(")"))
_CH_SPACE = const(ord(" "))
_CH_CR = const(ord("\r"))
_CH_LF = const(ord("\n"))
_CH_0 = const(ord("0"))
_CH_9 = const(ord("9"))
_CH_A = const(ord("A"))
_CH_F = const(ord("F"))
_CH_P = const(ord("P"))
_CH_S = const(ord("S"))
_CH_Z = const(ord("Z"))
_CH_a = const(ord("a"))
_CH_z = const(ord("z"))
_CH_DOT = const(ord("."))
_CH_MINUS = const(ord("-"))

class Tokenizer:
    def start(self, line):
        self._bytes = bytes(line, "ascii")
        self._pos = 0

    def command(self):
        key = self.key()
        if key is None:
            return None
        return chr(key) + str(self.value(False))
    
    def key(self):
        b = self._bytes
        ln = len(b)
        while True:
            if self._pos >= ln:
                return None
            ch = b[self._pos]
            self._pos += 1
            if ch == _CH_SPACE:
                continue
            if ch == _CH_SEMICOLON or ch == _CH_CR or ch == _CH_LF:
                return None
            if ch == _CH_PAREN1:
                while self._pos < ln:
                    ch = b[self._pos]
                    self._pos += 1
                    if ch == _CH_PAREN2:
                        break
                continue
            if (_CH_a <= ch <= _CH_z):
                ch = ch - _CH_a + _CH_A
            if not (_CH_A <= ch <= _CH_Z):
                raise ValueError("Invalid character %r at position %d" % (chr(ch), self._pos - 1))
            return ch
    
    def value(self, decimal=True):
        b = self._bytes
        ln = len(b)
        start = self._pos
        while True:
            if self._pos >= ln:
                break
            ch = b[self._pos]
            if not ((_CH_0 <= ch <= _CH_9) or ch == _CH_MINUS or ch == _CH_DOT):
                break
            self._pos += 1
        if self._pos > start:
            try:
                return float(b[start:self._pos]) if decimal else int(b[start:self._pos])
            except (ValueError, TypeError):
                pass
        raise ValueError("Expected a numeric value at position %d" % start)

class Parser:
    def __init__(self, rig, *, axes="XYZABCUVW"):
        self._rig = rig
        # Axis count is min(len(axes), rig.n_channels); any extra characters in
        # `axes` are ignored.
        self._naxes = min(len(axes), rig.n_channels)
        self._axis_ix = {ord(c): i for i, c in enumerate(axes[:self._naxes])}
        self._target = [None] * self._naxes                      # reused by _linear()
        self.linear_speed = [DEFAULT_SPEED, DEFAULT_SPEED]        # mm/sec
        self.line_no = 0
        self._tokenizer = Tokenizer()

    # --- flow control ----------------------------------------------------

    def ready(self):
        return self._rig.queue_avail() > READY_SLOTS

    def wait_ready(self):
        while not self.ready():
            time.sleep_ms(POLL_MS)

    def wait_finish(self):
        while self._rig.is_running():
            time.sleep_ms(POLL_MS)

    # --- parsing -------------------------------------------------------

    def parse(self, line):
        self.wait_ready()
        if self.execute(line):
            self.wait_finish()

    def execute(self, line):
        """Interpret one line. Non-blocking. Returns True iff the line asked to
        wait for motion to finish (M400), which parse() then honours."""
        self.line_no += 1
        try:
            self._tokenizer.start(line)
            command = self._tokenizer.command()
            if not command:
                return False
            handler = getattr(self, command, None)
            if handler is None:
                raise ValueError("unsupported %s" % command)
            return bool(handler())
        except ValueError as e:
            # Syntax errors (Tokenizer, _key_err, the raises below) and a
            # non-ascii line (UnicodeError subclasses ValueError) get the line
            # number attached. A rig exception -- MotionError etc. -- is a
            # RuntimeError subclass and propagates untouched.
            raise ValueError("line %d: %s" % (self.line_no, e))

    # --- code handlers ------------------------------------------------

    def G0(self):
        self._linear(0)

    def G1(self):
        self._linear(1)

    def _linear(self, mode):
        target = self._target
        for i in range(self._naxes):
            target[i] = None
        have_axis = False
        while True:
            key = self._tokenizer.key()
            if key is None:
                break
            if key in self._axis_ix:
                target[self._axis_ix[key]] = self._tokenizer.value()
                have_axis = True
            elif key == _CH_F:
                self.linear_speed[mode] = self._tokenizer.value() / 60.0
            else:
                raise self._key_err(key)
        if have_axis:
            self._rig.move(target, speed=self.linear_speed[mode])

    def G4(self):
        key = self._tokenizer.key()
        if key == _CH_S:
            secs = self._tokenizer.value()
        elif key == _CH_P:
            secs = self._tokenizer.value() / 1000.0
        else:
            raise ValueError("G4 requires P (ms) or S (s)")
        self._no_more_keys()
        self._rig.dwell(secs)

    def M400(self):
        self._no_more_keys()
        return True # True means wait for motion to finish

    def _no_more_keys(self):
        key = self._tokenizer.key()
        if not key is None:
            raise self._key_err(key)

    def _key_err(self, key):
        return ValueError("unexpected key %s" % chr(key))

class AsyncParser(Parser):
    """Asynchronous Parser: parse()/wait_ready()/wait_finish() are coroutines that
    yield to the event loop instead of blocking."""

    async def parse(self, line):
        await self.wait_ready()
        if self.execute(line):
            await self.wait_finish()

    async def wait_ready(self):
        while not self.ready():
            await asyncio.sleep_ms(POLL_MS)

    async def wait_finish(self):
        while self._rig.is_running():
            await asyncio.sleep_ms(POLL_MS)



