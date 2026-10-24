"""Detect loss of velocity commands using a monotonic clock."""

import math
import time


class CommandWatchdog:
    """Report each command-stream timeout once, until a new command arrives."""

    def __init__(self, timeout_sec, clock=time.monotonic):
        if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
            raise ValueError("command_timeout_sec must be finite and positive")
        self.timeout_sec = float(timeout_sec)
        self.clock = clock
        self.last_command_at = None
        self.reported = False

    def received(self):
        """Start or refresh the deadline, including explicit zero commands."""
        self.last_command_at = self.clock()
        self.reported = False

    def expired(self):
        """Return true once for each interrupted command stream."""
        if self.last_command_at is None or self.reported:
            return False
        if self.clock() - self.last_command_at < self.timeout_sec:
            return False
        self.reported = True
        return True

    def reset(self):
        """Discard commands after explicit release/disconnection."""
        self.last_command_at = None
        self.reported = False
