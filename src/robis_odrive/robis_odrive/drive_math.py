"""Motor mapping and encoder odometry, independent of ROS and USB."""

from dataclasses import dataclass
import math


def positive(value, name):
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


@dataclass(frozen=True)
class DriveGeometry:
    wheel_track: float
    tyre_circumference: float
    encoder_yaw_sign: float = 1.0

    def __post_init__(self):
        positive(self.wheel_track, "wheel_track")
        positive(self.tyre_circumference, "tyre_circumference")
        if self.encoder_yaw_sign not in (-1.0, 1.0):
            raise ValueError("encoder_yaw_sign must be -1 or 1")

    def motor_commands(self, forward, yaw):
        if not all(math.isfinite(value) for value in (forward, yaw)):
            raise ValueError("Velocity commands must be finite")
        angular = self.encoder_yaw_sign * yaw * self.wheel_track / 2
        # With yaw_sign=-1, preserve -(v+w*b/2)/c and -(v-w*b/2)/c.
        return (-(forward - angular) / self.tyre_circumference,
                -(forward + angular) / self.tyre_circumference)


@dataclass(frozen=True)
class EncoderSample:
    """Feedback with the existing left / negative-right polarity applied."""
    left_count: float
    right_count: float
    left_velocity: float
    right_velocity: float
    cpr: float

    def __post_init__(self):
        positive(self.cpr, "encoder_cpr")
        if not all(math.isfinite(value) for value in (
            self.left_count, self.right_count, self.left_velocity, self.right_velocity,
        )):
            raise ValueError("Encoder feedback must be finite")


@dataclass
class OdometryState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0


def wrapped_delta(current, previous, cpr):
    """Single-turn difference; assumes < half a turn per sample."""
    delta = current - previous
    if delta > cpr / 2:
        delta -= cpr
    elif delta < -cpr / 2:
        delta += cpr
    return delta


class EncoderOdometry:
    def __init__(self, geometry, mode="legacy", velocity_units="turns_per_second",
                 legacy_velocity_scale=100.0):
        if mode not in ("legacy", "encoder"):
            raise ValueError("odometry_mode must be legacy or encoder")
        if velocity_units not in ("turns_per_second", "counts_per_second"):
            raise ValueError("encoder_velocity_units must be turns_per_second or counts_per_second")
        self.geometry, self.mode, self.velocity_units = geometry, mode, velocity_units
        self.legacy_velocity_scale = positive(legacy_velocity_scale, "legacy_velocity_scale")
        self.state = OdometryState()
        self.previous = None

    def rebase(self):
        self.previous = None
        self.state.linear_velocity = self.state.angular_velocity = 0.0

    def reset(self):
        self.state = OdometryState()

    def update(self, sample):
        g, state = self.geometry, self.state
        if self.mode == "legacy":
            scale = g.tyre_circumference * self.legacy_velocity_scale / sample.cpr
            state.linear_velocity = (sample.left_velocity + sample.right_velocity) * scale / 2
            state.angular_velocity = (sample.right_velocity - sample.left_velocity) * scale / g.wheel_track
        else:
            scale = g.tyre_circumference
            if self.velocity_units == "counts_per_second":
                scale /= sample.cpr
            left, right = sample.left_velocity * scale, sample.right_velocity * scale
            state.linear_velocity = (left + right) / 2
            state.angular_velocity = g.encoder_yaw_sign * (right - left) / g.wheel_track
        previous, self.previous = self.previous, sample
        if previous is None or previous.cpr != sample.cpr:
            return state
        left = wrapped_delta(sample.left_count, previous.left_count, sample.cpr) * g.tyre_circumference / sample.cpr
        right = wrapped_delta(sample.right_count, previous.right_count, sample.cpr) * g.tyre_circumference / sample.cpr
        distance = (left + right) / 2
        angle = g.encoder_yaw_sign * (right - left) / g.wheel_track
        if self.mode == "legacy":
            dx, dy = math.cos(angle) * distance, -math.sin(angle) * distance
        else:
            half = angle / 2
            sinc = math.sin(half) / half if abs(half) > 1e-8 else 1 - half*half / 6
            dx, dy = distance * sinc * math.cos(half), distance * sinc * math.sin(half)
        state.x += math.cos(state.yaw) * dx - math.sin(state.yaw) * dy
        state.y += math.sin(state.yaw) * dx + math.cos(state.yaw) * dy
        state.yaw = (state.yaw + angle) % (2 * math.pi)
        return state
