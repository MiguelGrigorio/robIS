"""Read-only profile checks and bounded telemetry guards, independent of ROS."""

from collections import deque
import json
import math


class SafetyFault(RuntimeError):
    """A session must stop and require an explicit reconnect."""


def finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise SafetyFault(f'Invalid telemetry: {name}')
    return value


def property_value(obj, path):
    for name in path.split('.'):
        obj = getattr(obj, name)
    return obj


class SafetyProfile:
    def __init__(self, data):
        self.data = data
        self.serial = data['device_serial_number'].upper()
        self.firmware = tuple(int(n) for n in data['firmware'].split('.'))
        self.properties = data['per_axis_properties']
        self.circumference = 2 * math.pi * data['wheel_radius_m']
        self.velocity_limit = self.properties['controller.config.vel_limit']
        self.motor_signs = {int(k): v for k, v in data.get('motor_signs', {'0': -1, '1': 1}).items()}
        if set(self.motor_signs) != {0, 1} or any(v not in (-1, 1) for v in self.motor_signs.values()):
            raise ValueError('motor_signs must declare +/-1 for axes 0 and 1')

    @classmethod
    def load(cls, path):
        with open(path) as stream:
            return cls(json.load(stream))

    def verify(self, device):
        if format(int(device.serial_number), 'X') != self.serial:
            raise SafetyFault('ODrive serial does not match the commissioned robot')
        version = (device.fw_version_major, device.fw_version_minor, device.fw_version_revision)
        if version != self.firmware or int(device.fw_version_unreleased) != self.data.get('firmware_unreleased', 0):
            raise SafetyFault('ODrive firmware does not match the commissioned profile')
        if (device.hw_version_major, device.hw_version_minor, device.hw_version_variant) != (3, 6, 56):
            raise SafetyFault('Expected ODrive v3.6 56 V')
        if not device.user_config_loaded:
            raise SafetyFault('ODrive did not load its saved configuration')
        for index, axis in enumerate((device.axis0, device.axis1)):
            if axis.current_state != 1:
                raise SafetyFault(f'axis{index} must be IDLE')
            check_errors(axis)
            for path, expected in self.properties.items():
                actual = finite(property_value(axis, path), path)
                if not math.isclose(actual, expected, rel_tol=1e-5, abs_tol=1e-5):
                    raise SafetyFault(f'axis{index}.{path}: {actual} != {expected}')
            for path, expected in (('encoder.config.cpr', 90), ('encoder.config.mode', 1),
                                   ('motor.config.pole_pairs', 15)):
                if property_value(axis, path) != expected:
                    raise SafetyFault(f'axis{index}.{path} does not match Hall motors')
            if not axis.motor.is_calibrated or not axis.encoder.is_ready:
                raise SafetyFault(f'axis{index} is not calibrated/ready')
            if abs(finite(axis.controller.input_vel, 'input_vel')) > 1e-5:
                raise SafetyFault(f'axis{index} has a residual velocity command')


def check_errors(axis):
    for path in ('error', 'motor.error', 'encoder.error', 'controller.error',
                 'fet_thermistor.error'):
        if property_value(axis, path):
            raise SafetyFault(f'ODrive {path} is nonzero; errors were not cleared')


class MotionGuard:
    """Compare measured travel to native ramp references, including on curves.

    No equal-wheel-distance assumption is made. The 2.5 cm guard is on the
    difference of the two tracking residuals. It still needs physical tuning.
    """

    def __init__(self, circumference, cpr=90, motor_signs=(-1, 1)):
        self.circumference, self.cpr = circumference, cpr
        self.motor_signs = motor_signs
        self.previous = None
        self.residual = [0.0, 0.0]
        self.stalled_since = [None, None]
        self.stopped = deque()

    def update(self, now, rows, commands):
        if self.previous is not None:
            stamp, old = self.previous
            dt = now - stamp
            if not 0 < dt <= 0.5:
                raise SafetyFault('Telemetry interval exceeds 0.5 s or clock regressed')
            for i, row in enumerate(rows):
                measured = row['position'] - old[i]['position']
                expected = dt * (row['setpoint'] + old[i]['setpoint']) / 2
                self.residual[i] += self.motor_signs[i] * (measured - expected)
            if abs(self.residual[0] - self.residual[1]) * self.circumference > 0.025:
                raise SafetyFault('Wheel tracking residual difference exceeds 2.5 cm')
        self.previous = (now, rows)
        for i, row in enumerate(rows):
            suspected = (abs(row['setpoint']) >= 0.3 and
                         abs(row['velocity']) < 0.4 * abs(row['setpoint']) and
                         abs(row['iq_setpoint']) > 1.0)
            if suspected:
                if self.stalled_since[i] is None:
                    self.stalled_since[i] = now
                elif now - self.stalled_since[i] >= 0.1:
                    raise SafetyFault(f'Possible wheel stall on axis{i}')
            else:
                self.stalled_since[i] = None
        if any(abs(v) > 0.001 for v in commands) or any(abs(r['setpoint']) > 0.001 for r in rows):
            self.stopped.clear()
            return False
        self.stopped.append((now, [r['position'] for r in rows]))
        while len(self.stopped) > 1 and now - self.stopped[0][0] > 0.3:
            self.stopped.popleft()
        if len(self.stopped) < 3 or now - self.stopped[0][0] < 0.24:
            return False
        for i in (0, 1):
            positions = [item[1][i] for item in self.stopped]
            if max(positions) - min(positions) > 2 / self.cpr or abs(positions[-1] - positions[0]) > 1 / self.cpr:
                return False
        return True
