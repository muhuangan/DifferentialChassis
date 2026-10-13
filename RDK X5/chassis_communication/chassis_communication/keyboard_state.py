"""Terminal key state. Key repeat timeout is NOT a real key-release event."""


class KeyboardState:
    def __init__(self, linear_speed, angular_speed, linear_step, angular_step,
                 max_linear, max_angular, stop_delay):
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.linear_step = linear_step
        self.angular_step = angular_step
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.stop_delay = stop_delay
        self.motion = None
        self.last_motion = None
        self.escape = None
        self.escape_at = 0.0

    def feed(self, key, now):
        """Return False on Ctrl+C; ignore ANSI arrow/function key sequences."""
        if key == '\x03':
            self.motion = None
            return False
        if self.escape is not None and now - self.escape_at > 0.1:
            self.escape = None
        if key == '\x1b':
            self.escape = 'start'
            self.escape_at = now
            return True
        if self.escape is not None:
            if self.escape == 'start' and key in '[O':
                self.escape = 'sequence'
            elif self.escape == 'start' or '@' <= key <= '~':
                self.escape = None
            return True
        key = key.lower()
        if key in ('w', 's', 'a', 'd'):
            self.motion = key
            self.last_motion = now
        elif key == ' ':
            self.motion = None
        elif key == 'q':
            self.linear_speed = min(self.max_linear, self.linear_speed + self.linear_step)
            self.angular_speed = min(self.max_angular, self.angular_speed + self.angular_step)
        elif key == 'e':
            self.linear_speed = max(0.0, self.linear_speed - self.linear_step)
            self.angular_speed = max(0.0, self.angular_speed - self.angular_step)
        return True

    def velocity(self, now):
        if self.last_motion is None or now - self.last_motion >= self.stop_delay:
            self.motion = None
        # Each direction replaces the previous direction; no stale second axis.
        return {
            'w': (self.linear_speed, 0.0),
            's': (-self.linear_speed, 0.0),
            'a': (0.0, self.angular_speed),
            'd': (0.0, -self.angular_speed),
        }.get(self.motion, (0.0, 0.0))
