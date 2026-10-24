"""Lifecycle and commands shared by the real and simulated adapters."""

import time
from .command_watchdog import CommandWatchdog
from .drive_math import EncoderSample, positive


class DriveController:
    def __init__(self, driver, odometry, command_timeout_sec=0.5,
                 idle_release_sec=2.0, axis_for_right=0, connection_timeout_sec=10.0,
                 calibration_timeout_sec=60.0, use_preroll=False, clock=time.monotonic):
        if axis_for_right not in (0, 1):
            raise ValueError("axis_for_right must be 0 or 1")
        self.driver, self.odometry, self.clock = driver, odometry, clock
        self.watchdog = CommandWatchdog(command_timeout_sec, clock)
        self.idle_release_sec = positive(idle_release_sec, "idle_release_sec")
        self.connection_timeout_sec = positive(connection_timeout_sec, "connection_timeout_sec")
        self.calibration_timeout_sec = positive(calibration_timeout_sec, "calibration_timeout_sec")
        self.axis_for_right, self.use_preroll = axis_for_right, use_preroll
        self.connected = self.engaged = False
        self.last_command = (0.0, 0.0)
        self.idle_since = self.last_time = self.last_sample = None
        self._connection_attempted = self._shutdown_complete = False

    def startup(self, connect=False, calibrate=False, engage=False):
        if not connect and (calibrate or engage):
            raise ValueError("calibrate/engage_on_startup require connect_on_startup")
        try:
            if connect:
                self.connect()
                if calibrate:
                    self.calibrate()
                if engage:
                    self.engage()
        except Exception:
            self.shutdown()
            raise

    def connect(self):
        if self.connected:
            return
        self._connection_attempted = True
        if not self.driver.connect(right_axis=self.axis_for_right, timeout=self.connection_timeout_sec):
            raise RuntimeError("Não foi possível conectar à ODrive")
        self.connected = True
        self._shutdown_complete = False
        self.watchdog.reset()
        self.odometry.rebase()
        self.last_time = None

    def calibrate(self):
        if not self.connected:
            raise RuntimeError("Conecte a ODrive antes de calibrar")
        if self.engaged:
            raise RuntimeError("Libere os motores antes de calibrar")
        if not self.driver.calibrate(timeout=self.calibration_timeout_sec):
            raise RuntimeError("Calibração da ODrive falhou")
        self.odometry.rebase()

    def engage(self):
        if not self.connected:
            raise RuntimeError("Conecte a ODrive antes de engatar")
        if self.engaged:
            return
        if self.use_preroll and not self.driver.preroll(timeout=self.calibration_timeout_sec):
            raise RuntimeError("Busca de índice da ODrive falhou")
        if not self.driver.engage():
            raise RuntimeError("Não foi possível engatar os motores")
        self.engaged = True
        self.idle_since = self.clock()

    def command(self, forward, yaw, time_sec):
        values = self.odometry.geometry.motor_commands(forward, yaw)
        if not self.connected:
            return False
        self.driver.update_time(time_sec)
        if any(values) and not self.engaged:
            self.engage()
        if self.driver.drive(*values) is False:
            raise RuntimeError("Comando da ODrive falhou")
        self.last_command = values
        if any(values):
            self.idle_since = None
        elif self.idle_since is None:
            self.idle_since = self.clock()
        self.watchdog.received()
        return True

    def stop(self):
        if self.connected:
            if self.driver.drive(0.0, 0.0) is False:
                raise RuntimeError("Comando de parada da ODrive falhou")
        self.last_command = (0.0, 0.0)
        if self.idle_since is None:
            self.idle_since = self.clock()

    def release(self):
        if not self.connected:
            return
        self.stop()
        if self.driver.release() is False:
            raise RuntimeError("Não foi possível liberar os motores")
        self.engaged = False
        self.watchdog.reset()

    def disconnect(self):
        try:
            self.release()
        finally:
            try:
                self.driver.disconnect()
            finally:
                self.connected = self.engaged = False
                self.watchdog.reset()
                self.odometry.rebase()
                self.last_time = None

    def step(self, time_sec):
        if not self.connected:
            return None, False
        self.driver.update_time(time_sec)
        self.driver.check_health()
        if self.last_time is not None and time_sec < self.last_time:
            self.odometry.rebase()
        self.last_time = time_sec
        sample = EncoderSample(
            self.driver.left_pos(), -self.driver.right_pos(),
            self.driver.left_vel_estimate(), -self.driver.right_vel_estimate(),
            self.driver.encoder_cpr,
        )
        self.last_sample = sample
        state = self.odometry.update(sample)
        expired = self.watchdog.expired()
        if expired:
            self.stop()
        if self.engaged and self.idle_since is not None and (
                self.driver.ready_to_release() or self.clock() - self.idle_since >= self.idle_release_sec):
            self.release()
        if self.engaged:
            self.driver.feed_watchdog()
        return state, expired

    def reset_odometry(self, time_sec):
        self.step(time_sec)
        self.odometry.reset()

    def shutdown(self):
        if self._shutdown_complete:
            return
        self._shutdown_complete = True
        if not self._connection_attempted:
            return
        try:
            self.driver.drive(0.0, 0.0)
        finally:
            try:
                self.driver.release()
            finally:
                try:
                    self.driver.disconnect()
                finally:
                    self.connected = self.engaged = False
                    self.last_command = (0.0, 0.0)
                    self.watchdog.reset()
                    self.odometry.rebase()
