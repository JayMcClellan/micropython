"""A small G-code front end for a motion.Rig.

Supported now:

    G0 / G1       linear move   -- axis words (XYZABCUVW) + optional F
    G4            dwell         -- P milliseconds or S seconds
    G92           set position  -- axis words; unnamed axes keep their value
    G21/G90/G94   no-ops        -- mm / absolute / feed-per-minute (already assumed)
    M0 / M1       pause         -- rig.pause();  state -> PARSER_PAUSED
    M24           resume        -- rig.resume(); state -> PARSER_OK
    M2 / M30      program end    -- wait for motion; state -> PARSER_ENDED
    M112          stop          -- rig.stop();   state -> PARSER_STOPPED
    M400          wait for all queued motion to finish

A non-blank line must start with one of those codes; everything after it is that
code's arguments, in any order, and an argument the code doesn't accept is an
error. Axis letters map to Rig channels 0.. in the order of the `axes` string
(default "XYZABCUVW"); the axis count is min(len(axes), rig.n_channels) and any
extra characters in `axes` are ignored. Only G0/G1/G92 take axis words. Blank
lines and comments (`; ...` and `( ... )`) are skipped.
N line numbers and `*` checksums are not supported.

Each code is handled by a method named `_<code>` (e.g. `_G0`, `_M400`), looked
up by name, so adding a code is just adding a method.

Feed rate F is mm/min and is passed to rig.move() as speed = F / 60, so the
rig's channels are assumed to be scaled to millimetres. G0 and G1 keep separate
stored speeds (self.linear_speed[0] and [1]).

state() returns PARSER_OK / PARSER_PAUSED / PARSER_STOPPED / PARSER_ENDED /
PARSER_ERROR. It is advisory only -- the parser never changes what it does based
on it; a caller polls it between lines to decide whether to keep feeding. PAUSED
(M0/M1) is cleared by M24; ENDED (M2/M30) and STOPPED (M112) are terminal; ERROR
is set when a line raises and cleared by the next line. start() resets the state
and the line counter for a new program.

    import gcode

    gc = gcode.Parser(rig)                 # rig: a configured motion.Rig
    gc.parse("G0 F1000 ; rapid rate")
    gc.parse("G0 X10 Y20 ; go to start")
    gc.parse("G1 Z5 F300")
    gc.parse("G1 X10 Y20 Z0")
    gc.parse("G4 P500 (dwell 0.5 s)")
    gc.parse("M400")
    gc.wait_finish()

parse() blocks until the rig can take another line. For cooperative code the same
Parser offers coroutine equivalents: parse_async(), await_ready(), await_finish().
"""

import time
from micropython import const
import asyncio

DEFAULT_SPEED = 1000.0 / 60.0   # mm/sec, used for G0 and G1 until an F word sets them
POLL_MS = const(1)                        # spin interval while waiting on the rig
READY_SLOTS = const(2)                 # ready once queue_avail() exceeds this

_CH_SEMICOLON = ord(";")
_CH_PAREN1 = ord("(")
_CH_PAREN2 = ord(")")
_CH_SPACE = ord(" ")
_CH_CR = ord("\r")
_CH_LF = ord("\n")
_CH_0 = ord("0")
_CH_9 = ord("9")
_CH_A = ord("A")
_CH_F = ord("F")
_CH_P = ord("P")
_CH_S = ord("S")
_CH_Z = ord("Z")
_CH_a = ord("a")
_CH_z = ord("z")
_CH_DOT = ord(".")
_CH_MINUS = ord("-")

class Tokenizer:
    def start(self, line):
        self._bytes = bytes(line, "ascii")
        self._pos = 0

    def command(self):
        key = self.key()
        if key is None:
            return None
        return "_" + chr(key) + str(self.value(False))
    
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

PARSER_OK = const(0)
PARSER_PAUSED = const(1)
PARSER_STOPPED = const(2)
PARSER_ENDED = const(3)
PARSER_ERROR = const(-1)

class Parser:
    def __init__(self, rig, *, axes="XYZABCUVW"):
        self._rig = rig
        # Axis count is min(len(axes), rig.n_channels); any extra characters in
        # `axes` are ignored.
        self._naxes = min(len(axes), rig.n_channels)
        self._axis_ix = {ord(c): i for i, c in enumerate(axes[:self._naxes])}
        self._target = [None] * self._naxes                    # scratch for _linear()/_G92()
        self.linear_speed = [DEFAULT_SPEED, DEFAULT_SPEED]        # mm/sec
        self.blend = None
        self.line_no = 0
        self._tokenizer = Tokenizer()
        self._state = PARSER_OK

    # --- flow control ----------------------------------------------------

    def start(self):
        """Start a new program. Resets the line counter and parser state."""
        self.line_no = 0
        self._state = PARSER_OK

    def state(self):
        """Return the current parser state."""
        return self._state
    
    def ready(self):
        """Return True if the rig is ready to accept more commands."""
        return self._rig.queue_avail() > READY_SLOTS

    def wait_ready(self):
        """Block until the rig is ready to accept more commands."""
        while not self.ready():
            time.sleep_ms(POLL_MS)

    async def await_ready(self):
        """Asynchronously wait until the rig is ready to accept more commands."""
        while not self.ready():
            await asyncio.sleep_ms(POLL_MS)

    def wait_finish(self):
        """Block until the rig has finished all motion."""
        while self._rig.is_running():
            time.sleep_ms(POLL_MS)

    async def await_finish(self):
        """Asynchronously wait until the rig has finished all motion."""
        while self._rig.is_running():
            await asyncio.sleep_ms(POLL_MS)

    # --- parsing -------------------------------------------------------

    def parse(self, line):
        """Parse and execute one line. Blocks until the rig is ready and, if necessary, until motion finishes."""
        self.wait_ready()
        if self.execute(line):
            self.wait_finish()

    async def parse_async(self, line):
        """Asynchronously parse and execute one line. Awaits until the rig is ready and, if necessary, until motion finishes."""
        await self.await_ready()
        if self.execute(line):
            await self.await_finish()

    def execute(self, line):
        """Interpret one line. Non-blocking (queues motion, never waits). Returns
        True iff the line asked to wait for motion to finish (M400 / M2 / M30),
        which parse() / parse_async() then honour."""
        self.line_no += 1
        if self._state == PARSER_ERROR:      # a fresh line retries after an error
            self._state = PARSER_OK
        try:
            self._tokenizer.start(line)
            command = self._tokenizer.command()
            if not command:
                return False
            handler = getattr(self, command, None)
            if handler is None:
                raise ValueError("unsupported %s" % command[1:])
            return bool(handler())
        except ValueError as e:
            self._state = PARSER_ERROR
            # Syntax errors (Tokenizer, _key_err, the raises below) and a
            # non-ascii line (UnicodeError subclasses ValueError) get the line
            # number attached. A rig exception (MotionError etc.) is a
            # RuntimeError subclass, so it skips this handler, keeps its type,
            # and is only flagged PARSER_ERROR by the one below.
            raise ValueError("line %d: %s" % (self.line_no, e))
        except Exception:
            self._state = PARSER_ERROR
            raise

    # --- code handlers ------------------------------------------------

    def _G0(self):
        """ Linear rapid move """
        self._linear(0)

    def _G1(self):
        """ Linear controlled move """
        self._linear(1)

    def _read_axes(self, mode=None):
        """Clear self._target, then read the remaining words into it: each axis
        word into its channel slot, and -- only when `mode` is given -- an F
        word into self.linear_speed[mode]. Any other key is an error. Returns
        True if at least one axis word was seen."""
        target = self._target
        for i in range(self._naxes):
            target[i] = None
        have_axis = False
        while True:
            key = self._tokenizer.key()
            if key is None:
                return have_axis
            if key in self._axis_ix:
                target[self._axis_ix[key]] = self._tokenizer.value()
                have_axis = True
            elif mode is not None and key == _CH_F:
                self.linear_speed[mode] = self._tokenizer.value() / 60.0
            else:
                raise self._key_err(key)

    def _linear(self, mode):
        """ Linear move; mode 0 = rapid, 1 = controlled """
        if self._read_axes(mode):
            self._rig.move(self._target, speed=self.linear_speed[mode], blend=self.blend)

    def _G4(self):
        """ Dwell for a specified time (P = milliseconds, S = seconds) """
        key = self._tokenizer.key()
        if key == _CH_S:
            secs = self._tokenizer.value()
        elif key == _CH_P:
            secs = self._tokenizer.value() / 1000.0
        else:
            raise ValueError("G4 requires P (ms) or S (s)")
        self._no_more_keys()
        self._rig.dwell(secs)

    def _G21(self):
        """ Set units to millimeters (ignored) """
        self._no_more_keys()

    def _G64(self):
        """ Set path blending """
        key = self._tokenizer.key()
        if key == _CH_P:
            self.blend = self._tokenizer.value()
        else:
            raise ValueError("G64 requires P (blend value)")
        self._no_more_keys()

    def _G90(self):
        """ Set to absolute positioning (ignored) """
        self._no_more_keys()

    def _G92(self):
        """ Set current position for named axes; unnamed axes keep their value.
        Best issued when idle -- precede with M400 if mid-program. """
        if self._read_axes():
            self._rig.set_position(self._target)

    def _G94(self):
        """ Set to feedrate per minute mode (ignored) """
        self._no_more_keys()

    def _M0(self):
        """ Program pause """
        self._no_more_keys()
        self._rig.pause()
        self._state = PARSER_PAUSED

    def _M1(self):
        """ Optional stop (treated as M0) """
        return self._M0()

    def _M2(self):
        """ Program end """
        self._no_more_keys()
        self._state = PARSER_ENDED
        return True # True means wait for motion to finish
        
    def _M24(self):
        """ Program resume """
        self._no_more_keys()
        self._rig.resume()
        self._state = PARSER_OK

    def _M30(self):
        """ Program end """
        return self._M2()

    def _M112(self):
        """ Stop """
        self._no_more_keys()
        self._rig.stop()
        self._state = PARSER_STOPPED
    
    def _M400(self):
        """ Wait for all motion to finish """
        self._no_more_keys()
        return True # True means wait for motion to finish

    def _no_more_keys(self):
        key = self._tokenizer.key()
        if not key is None:
            raise self._key_err(key)

    def _key_err(self, key):
        return ValueError("unexpected key %s" % chr(key))
