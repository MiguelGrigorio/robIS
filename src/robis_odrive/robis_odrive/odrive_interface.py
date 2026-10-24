#!/usr/bin/env python3
import logging
import math
import time
import traceback

import odrive
from odrive.enums import *
from .drive_safety import MotionGuard, SafetyFault, SafetyProfile, check_errors, finite, property_value

default_logger = logging.getLogger(__name__)
default_logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

default_logger.addHandler(ch)


class ODriveFailure(Exception):
    pass


class ODriveInterfaceAPI(object):
    driver = None
    encoder_cpr = 4096
    right_axis = None
    left_axis = None
    connected = False
    _preroll_started = False
    _preroll_completed = False
    # engaged = False

    def __init__(self, logger=None, active_odrive=None, profile=None, clock=time.monotonic):
        self.logger = logger if logger else default_logger
        if profile is None:
            from ament_index_python.packages import get_package_share_directory
            from pathlib import Path
            profile = SafetyProfile.load(Path(get_package_share_directory('robis_odrive'))
                                         / 'params/ground_05mps.json')
        self.profile, self.clock = profile, clock
        self.right_axis_index = 0
        self.driver = None
        self.axes = ()
        self._active = self._stop_confirmed = False
        self._guard = None
        self._commands = (0.0, 0.0)
        self.telemetry = None
        self._straight_origin = self._straight_direction = None

        if (
            active_odrive
        ):  # pass in the odrv0 object from odrivetool shell to use it directly.
            self.profile.verify(active_odrive)
            self.driver = active_odrive
            self.axes = (self.driver.axis0, self.driver.axis1)
            self.right_axis = self.driver.axis0
            self.left_axis = self.driver.axis1
            self.logger.info(
                "Loaded pre-existing ODrive interface. Check index search status."
            )
            self.encoder_cpr = self.driver.axis0.encoder.config.cpr
            self.connected = True
            self._preroll_started = False
            self._preroll_completed = True

    def __del__(self):
        self.disconnect()

    def update_time(self, curr_time):
        # provided so simulator can update position
        pass

    def connect(self, port=None, right_axis=0, timeout=10):
        if right_axis not in (0, 1):
            raise ValueError('right_axis must be 0 or 1')
        if self.driver:
            self.disconnect()
        try:
            candidate = odrive.find_any(serial_number=self.profile.serial, timeout=timeout)
            self.profile.verify(candidate)
            self.driver = candidate
            self.axes = (self.driver.axis0, self.driver.axis1)
        except Exception as error:
            self.logger.error(f'ODrive discovery/profile check failed: {error}')
            return False

        # save some parameters for easy access
        self.right_axis = self.driver.axis0 if right_axis == 0 else self.driver.axis1
        self.left_axis = self.driver.axis1 if right_axis == 0 else self.driver.axis0
        self.right_axis_index = right_axis

        # check for no errors
        for axis in [self.right_axis, self.left_axis]:
            if axis.error != 0:
                error_str = (
                    "ODrive reports an active error; leaving it unchanged. "
                    "Axis error 0x%x, motor error 0x%x, encoder error 0x%x."
                    % (axis.error, axis.motor.error, axis.encoder.error)
                )
                self.logger.error(error_str)
                self.connected = False
                return False

        self.encoder_cpr = self.driver.axis0.encoder.config.cpr

        self.connected = True
        self.logger.info("Connected to ODrive. " + self.get_version_string())

        self._preroll_started = False
        self._preroll_completed = True

        return True

    def disconnect(self):
        if not self.driver:
            return True

        try:
            self.release()
            self.logger.info("Odrive disconnected.")
        except:
            self.logger.error("Error in timer: " + traceback.format_exc())
            return False
        finally:
            self.driver = None
            self.connected = self._active = False
            self.right_axis = self.left_axis = None
            self.axes = ()
        return True

    def get_version_string(self):
        if not self.driver or not self.connected:
            return "Not connected."
        return "ODrive %s, hw v%d.%d-%d, fw v%d.%d.%d%s" % (
            format(int(self.driver.serial_number), 'X'),
            self.driver.hw_version_major,
            self.driver.hw_version_minor,
            self.driver.hw_version_variant,
            self.driver.fw_version_major,
            self.driver.fw_version_minor,
            self.driver.fw_version_revision,
            "-dev" if self.driver.fw_version_unreleased else "",
        )

    def reboot(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        try:
            self.driver.reboot()
        except KeyError:
            self.logger.error("Rebooted ODrive.")
        except:
            self.logger.error("Failed to reboot: " + traceback.format_exc())
        finally:
            self.driver = None
        return True

    def _wait_idle(self, axes, timeout):
        deadline = time.monotonic() + timeout
        # Allow requested_state to reach the firmware before checking IDLE.
        time.sleep(min(1.0, timeout))
        for axis in axes:
            while axis.current_state != AXIS_STATE_IDLE:
                if time.monotonic() >= deadline:
                    self.release()
                    raise ODriveFailure("Timed out waiting for ODrive axis")
                time.sleep(0.1)

    def calibrate(self, timeout=60):
        if not self.driver:
            self.logger.error("Not connected.")
            return False

        self.logger.info("Vbus %.2fV" % self.driver.vbus_voltage)

        for i, axis in enumerate(self.axes):
            self.logger.info("Calibrating axis %d..." % i)
            axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            self._wait_idle([axis], timeout)
            if axis.error != 0:
                self.logger.error(
                    "Failed calibration with axis error 0x%x, motor error 0x%x"
                    % (axis.error, axis.motor.error)
                )
                return False

        return True

    def preroll(self, wait=True, timeout=60):
        if not self.driver:
            self.logger.error("Not connected.")
            return False

        if self._preroll_started:  # must be prerolling or already prerolled
            return False
        self._preroll_started = True
        self._preroll_completed = False

        # self.logger.info("Vbus %.2fV" % self.driver.vbus_voltage)

        for i, axis in enumerate(self.axes):
            self.logger.info("Index search preroll axis %d..." % i)
            axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH

        if wait:
            self._wait_idle(self.axes, timeout)
            self._preroll_started = False
            for i, axis in enumerate(self.axes):
                if axis.error != 0:
                    self.logger.error(
                        "Failed preroll with left_axis error 0x%x, motor error 0x%x"
                        % (axis.error, axis.motor.error)
                    )
                    return False
            self._preroll_completed = True
            self.logger.info("Index search preroll complete.")
            return True
        else:
            return False

    def ensure_prerolled(self):
        # preroll success
        if self._preroll_completed:
            return True
        # started, not completed
        elif self._preroll_started:
            # self.logger.info("Checking for preroll complete.")
            if (
                self.axes[0].current_state != AXIS_STATE_ENCODER_INDEX_SEARCH
                and self.axes[1].current_state != AXIS_STATE_ENCODER_INDEX_SEARCH
            ):
                # completed, check for errors before marking complete
                for i, axis in enumerate(self.axes):
                    if axis.error != 0:
                        self._preroll_started = False
                        error_str = (
                            "Failed index search preroll with axis error 0x%x, motor error 0x%x, encoder error 0x%x. Rebooting."
                            % (axis.error, axis.motor.error, axis.encoder.error)
                        )
                        # self.reboot()
                        self.logger.error(error_str)
                        raise Exception(error_str)
                # no errors, success
                self._preroll_started = False
                self._preroll_completed = True
                self.logger.info("Index search preroll complete. Ready to drive.")
                return True
            else:
                # still prerolling
                return False
        else:  # start preroll
            # self.logger.info("Preroll started.")
            self.preroll(wait=False)
            return False

    def has_prerolled(self):
        return self._preroll_completed

    def engaged(self):
        if self.driver and hasattr(self, "axes"):
            return (
                self.axes[0].requested_state == AXIS_STATE_CLOSED_LOOP_CONTROL
                and self.axes[1].requested_state == AXIS_STATE_CLOSED_LOOP_CONTROL
            )
        else:
            return False

    def idle(self):
        if self.driver and hasattr(self, "axes"):
            return (
                self.axes[0].requested_state == AXIS_STATE_IDLE
                and self.axes[1].requested_state == AXIS_STATE_IDLE
            )
        else:
            return False

    def engage(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False

        self.profile.verify(self.driver)
        try:
            for axis in self.axes:
                axis.controller.input_vel = 0.0
                axis.controller.vel_integrator_torque = 0.0
                axis.watchdog_feed()
                axis.config.enable_watchdog = True
                if not axis.config.enable_watchdog:
                    raise SafetyFault('Watchdog enable was not confirmed')
                axis.watchdog_feed()
                axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            deadline = self.clock() + 1.0
            while any(axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL for axis in self.axes):
                for axis in self.axes:
                    check_errors(axis)
                    axis.watchdog_feed()
                if self.clock() >= deadline:
                    raise SafetyFault('CLOSED_LOOP was not confirmed on both axes')
                time.sleep(0.01)
            self._guard = MotionGuard(self.profile.circumference, self.encoder_cpr,
                                      self.profile.motor_signs)
            self._commands = (0.0, 0.0)
            self._straight_origin = self._straight_direction = None
            self._active = True
            self.check_health()
        except Exception:
            self.release()
            raise
        return True

    def release(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        failures = []
        # Attempt every operation for BOTH axes even when one USB write fails.
        for axis in self.axes:
            try:
                axis.controller.input_vel = 0.0
            except Exception as error:
                failures.append(str(error))
        for axis in self.axes:
            try:
                axis.requested_state = AXIS_STATE_IDLE
            except Exception as error:
                failures.append(str(error))
        for axis in self.axes:
            try:
                deadline = self.clock() + 1.0
                while axis.current_state != AXIS_STATE_IDLE:
                    if self.clock() >= deadline:
                        raise SafetyFault('IDLE was not confirmed; watchdog remains enabled')
                    time.sleep(0.01)
                axis.config.enable_watchdog = False
            except Exception as error:
                failures.append(str(error))
        self._active = self._stop_confirmed = False
        self._commands = (0.0, 0.0)
        if failures:
            raise SafetyFault('; '.join(failures))
        return True

    def drive(self, left_motor_val, right_motor_val):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        values = [float(left_motor_val), float(right_motor_val)]
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Motor references must be finite')
        if any(values) and not self._active:
            raise SafetyFault('Velocity requested outside an active session')
        # Common scaling preserves the commanded curvature at the wheel limit.
        scale = max(1.0, max(abs(v) for v in values) / self.profile.velocity_limit)
        values = [v / scale for v in values]
        if self._active:
            self.check_health()
            # Only equal signed wheel requests receive straight-line correction.
            if values[0] and math.isclose(values[0], values[1], abs_tol=1e-8):
                direction = 1 if values[0] < 0 else -1
                positions = (self.left_forward_sign * self.left_axis.encoder.pos_estimate,
                             self.right_forward_sign * self.right_axis.encoder.pos_estimate)
                if self._straight_origin is None or self._straight_direction != direction:
                    self._straight_origin, self._straight_direction = positions, direction
                travel = [direction * (p - origin) for p, origin in zip(positions, self._straight_origin)]
                leading = 0 if travel[0] > travel[1] else 1
                correction = min(0.08, abs(travel[0] - travel[1]))
                values[leading] = math.copysign(max(0.0, abs(values[leading]) - correction), values[leading])
            else:
                self._straight_origin = self._straight_direction = None
        try:
            self.left_axis.controller.input_vel = -self.left_forward_sign * values[0]
            self.right_axis.controller.input_vel = -self.right_forward_sign * values[1]
            # Store references in physical axis order for stop detection.
            self._commands = tuple(float(axis.controller.input_vel) for axis in self.axes)
            return True
        except Exception:
            self.release()
            raise
        # except (fibre.protocol.ChannelBrokenException, AttributeError) as e:
        #    raise ODriveFailure(str(e))

    def feed_watchdog(self):
        if not self._active:
            return
        self.left_axis.watchdog_feed()
        self.right_axis.watchdog_feed()

    def check_health(self):
        if not self.driver:
            return
        started = self.clock()
        voltage = finite(self.driver.vbus_voltage, 'vbus_voltage')
        if not 30 < voltage < 50:
            raise SafetyFault('Bus voltage outside the commissioned 30–50 V envelope')
        rows = []
        for axis in self.axes:
            check_errors(axis)
            if self._active and (axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL or
                                 not axis.config.enable_watchdog):
                raise SafetyFault('Active axis left CLOSED_LOOP or watchdog was disabled')
            temperature = finite(axis.fet_thermistor.temperature, 'fet temperature')
            if temperature > 60:
                raise SafetyFault('Inverter temperature exceeds 60 C')
            row = {key: finite(property_value(axis, path), path) for key, path in (
                ('position', 'encoder.pos_estimate'), ('velocity', 'encoder.vel_estimate'),
                ('setpoint', 'controller.vel_setpoint'),
                ('iq_setpoint', 'motor.current_control.Iq_setpoint'),
                ('iq', 'motor.current_control.Iq_measured'))}
            if self._active:
                if abs(row['iq']) > 2.4:
                    raise SafetyFault('Measured Iq exceeds 2.4 A')
                if abs(row['velocity']) > self.profile.velocity_limit * 1.15:
                    raise SafetyFault('Measured velocity exceeds the commissioned envelope')
            rows.append(row)
        if self.clock() - started > 0.5:
            raise SafetyFault('Telemetry acquisition exceeded 0.5 s')
        self.telemetry = {'time': self.clock(), 'voltage': voltage, 'axes': rows}
        if self._active:
            self._stop_confirmed = self._guard.update(self.clock(), rows, self._commands)

    def ready_to_release(self):
        return self._stop_confirmed

    def get_errors(self, clear=False):
        # TODO: add error parsing, see: https://github.com/madcowswe/ODrive/blob/master/tools/odrive/utils.py#L34
        if not self.driver:
            return None

        axis_error = self.axes[0].error or self.axes[1].error

        if axis_error:
            error_string = "Errors(hex): L: a%x m%x e%x c%x, R: a%x m%x e%x c%x" % (
                self.left_axis.error,
                self.left_axis.motor.error,
                self.left_axis.encoder.error,
                self.left_axis.controller.error,
                self.right_axis.error,
                self.right_axis.motor.error,
                self.right_axis.encoder.error,
                self.right_axis.controller.error,
            )

        if clear:
            for axis in self.axes:
                axis.error = 0
                axis.motor.error = 0
                axis.encoder.error = 0
                axis.controller.error = 0

        if axis_error:
            return error_string

    @property
    def left_forward_sign(self):
        return self.profile.motor_signs[1-self.right_axis_index]

    @property
    def right_forward_sign(self):
        return self.profile.motor_signs[self.right_axis_index]

    def left_vel_estimate(self):
        return (
            self.left_forward_sign * self.left_axis.encoder.vel_estimate if self.left_axis else 0
        )  # ODrive 0.5.x: turns/s (not encoder counts/s or m/s).

    def right_vel_estimate(self):
        return (
            -self.right_forward_sign * self.right_axis.encoder.vel_estimate if self.right_axis else 0
        )  # neg is forward for right

    def left_pos(self):
        return (
            self.left_forward_sign * self.left_axis.encoder.pos_cpr_counts if self.left_axis else 0
        )  # units: encoder counts

    def right_pos(self):
        return -self.right_forward_sign * self.right_axis.encoder.pos_cpr_counts if self.right_axis else 0

    # TODO check these match the right motors, but it doesn't matter for now
    def left_temperature(self):
        return 0  # self.left_axis.motor.motor_thermistor.temperature  if self.left_axis  else 0.

    def right_temperature(self):
        return 0  # self.right_axis.motor.motor_thermistor.temperature if self.right_axis else 0.

    # def left_current(self):       return self.left_axis.motor.current_control.power  if self.left_axis and self.left_axis.current_state > 1 else 0.
    # def right_current(self):      return self.right_axis.motor.current_control.power if self.right_axis and self.right_axis.current_state > 1 else 0.
    def left_current(self):
        return (
            self.left_axis.motor.current_control.Ibus
            if self.left_axis and self.left_axis.current_state > 1
            else 0.0
        )

    def right_current(self):
        return (
            self.right_axis.motor.current_control.Ibus
            if self.right_axis and self.right_axis.current_state > 1
            else 0.0
        )

    # from axis.hpp: https://github.com/madcowswe/ODrive/blob/767a2762f9b294b687d761029ef39e742bdf4539/Firmware/MotorControl/axis.hpp#L26
    MOTOR_STATES = [
        "UNDEFINED",  # <! will fall through to idle
        "IDLE",  # <! disable PWM and do nothing
        "STARTUP_SEQUENCE",  # <! the actual sequence is defined by the config.startup_... flags
        "FULL_CALIBRATION_SEQUENCE",  # <! run all calibration procedures, then idle
        "MOTOR_CALIBRATION",  # //<! run motor calibration
        "SENSORLESS_CONTROL",  # //<! run sensorless control
        "ENCODER_INDEX_SEARCH",  # //<! run encoder index search
        "ENCODER_OFFSET_CALIBRATION",  # //<! run encoder offset calibration
        "CLOSED_LOOP_CONTROL",  # //<! run closed loop control
        "LOCKIN_SPIN",  # //<! run lockin spin
        "ENCODER_DIR_FIND",
    ]

    def left_state(self):
        return (
            self.MOTOR_STATES[self.left_axis.current_state]
            if self.left_axis
            else "NOT_CONNECTED"
        )

    def right_state(self):
        return (
            self.MOTOR_STATES[self.right_axis.current_state]
            if self.right_axis
            else "NOT_CONNECTED"
        )

    def bus_voltage(self):
        return self.driver.vbus_voltage if self.left_axis else 0.0
