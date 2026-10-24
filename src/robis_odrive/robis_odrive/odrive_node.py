"""ROS topics and services for the ODrive base; math/lifecycle live separately."""

import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
import tf2_ros

from .drive_controller import DriveController
from .drive_math import DriveGeometry, EncoderOdometry, positive


class OdriveNode(Node):
    def __init__(self, *, driver=None, parameter_overrides=None):
        super().__init__('odrive_node', parameter_overrides=parameter_overrides)
        self.controller = None
        try:
            self.init_params()
            if driver is None:
                if self.sim_mode:
                    from .odrive_simulator import ODriveInterfaceSimulator
                    driver = ODriveInterfaceSimulator(logger=self.get_logger())
                else:
                    from .odrive_interface import ODriveInterfaceAPI
                    from .drive_safety import SafetyProfile
                    from ament_index_python.packages import get_package_share_directory
                    from pathlib import Path
                    profile_path = self.safety_profile or str(
                        Path(get_package_share_directory('robis_odrive')) / 'params/ground_05mps.json')
                    driver = ODriveInterfaceAPI(logger=self.get_logger(),
                                               profile=SafetyProfile.load(profile_path))
            odometry = EncoderOdometry(
                DriveGeometry(self.wheel_track, self.tyre_circumference, self.encoder_yaw_sign),
                self.odometry_mode, self.encoder_velocity_units, self.legacy_velocity_scale,
            )
            self.controller = DriveController(
                driver, odometry, self.command_timeout_sec, self.idle_release_sec,
                self.axis_for_right, self.connection_timeout_sec,
                self.calibration_timeout_sec, self.use_preroll,
            )
            self.controller.startup(self.connect_on_startup, self.calibrate_on_startup,
                                    self.engage_on_startup)
            self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
            self.tf_publisher = tf2_ros.TransformBroadcaster(self)
            self.raw_publishers = {
                key: self.create_publisher(Float64, f'odrive/{side}/raw_odom/{kind}', 10)
                for key, side, kind in (
                    ('left_count', 'left', 'encoder'), ('right_count', 'right', 'encoder'),
                    ('left_velocity', 'left', 'velocity'), ('right_velocity', 'right', 'velocity'),
                )
            }
            self.current_publishers = {
                side: self.create_publisher(Float64, f'odrive/{side}/current', 10)
                for side in ('left', 'right')
            }
            self.vel_subscriber = self.create_subscription(Twist, self.cmd_vel_topic,
                                                          self.cmd_vel_callback, 1)
            self.control_services = [self.create_service(Trigger, 'reset_odometry', self.reset_odometry)]
            for name in ('connect', 'disconnect', 'calibrate', 'engage', 'release'):
                self.control_services.append(self.create_service(
                    Trigger, f'~/{name}',
                    lambda request, response, action=name: self.control_service(action, response),
                ))
            from rclpy.clock import Clock, ClockType
            self.timer_recalc_odom = self.create_timer(
                1.0 / self.odom_calc_hz, self.fast_timer,
                clock=Clock(clock_type=ClockType.STEADY_TIME))
            if self.odometry_mode == 'legacy':
                self.get_logger().warning('Odometria legada selecionada; escala e sinal angular aguardam validação física')
            self.get_logger().info('ODrive pronta; serviços em /odrive_node e comando em ' + self.cmd_vel_topic)
        except Exception:
            try:
                self.shutdown()
            finally:
                self.destroy_node()
            raise

    def init_params(self):
        defaults = {
            'simulation_mode': False,
            'safety_profile': '', 'allow_angular_motion': False,
            'wheel_track': 0.36, 'tyre_circumference': 0.5366,
            'encoder_yaw_sign': 1.0, 'odometry_mode': 'encoder',
            'encoder_velocity_units': 'turns_per_second', 'legacy_velocity_scale': 100.0,
            'connect_on_startup': False, 'calibrate_on_startup': False,
            'engage_on_startup': False, 'axis_for_right': 0, 'use_preroll': False,
            'connection_timeout_sec': 10.0, 'calibration_timeout_sec': 60.0,
            'command_timeout_sec': 0.5, 'idle_release_sec': 2.0,
            'publish_current': True, 'publish_raw_odom': True,
            'publish_odom': True, 'publish_odom_tf': True,
            'odom_topic': '/odom', 'cmd_vel_topic': '/cmd_vel',
            'odom_frame': 'odom', 'base_frame': 'base_footprint', 'odom_calc_hz': 20,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
            setattr(self, name, self.get_parameter(name).value)
        self.sim_mode = self.simulation_mode
        positive(self.odom_calc_hz, 'odom_calc_hz')
        for name in ('odom_topic', 'cmd_vel_topic', 'odom_frame', 'base_frame'):
            if not getattr(self, name):
                raise ValueError(f'{name} cannot be empty')
        # Compatibility for existing callers, with an explicit warning rather
        # than advertising fabricated temperatures or unverified wheel angles.
        for name in ('publish_joint_angles', 'publish_temperatures'):
            self.declare_parameter(name, False, ParameterDescriptor(read_only=True))
            if self.get_parameter(name).value:
                self.get_logger().warning(f'{name} ainda não é suportado; nenhuma mensagem será inventada')

    def time_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def control_service(self, action, response):
        try:
            getattr(self.controller, action)()
            response.success, response.message = True, f'ODrive: {action} concluído'
        except Exception as error:
            response.success, response.message = False, str(error)
            self.handle_driver_failure(error)
        return response

    def reset_odometry(self, request, response):
        try:
            self.controller.reset_odometry(self.time_seconds())
            response.success, response.message = True, 'Odometry reset.'
        except Exception as error:
            response.success, response.message = False, str(error)
            self.handle_driver_failure(error)
        return response

    def cmd_vel_callback(self, msg):
        try:
            if not self.allow_angular_motion and msg.angular.z != 0.0:
                raise ValueError('Comando angular bloqueado até confirmar os lados das rodas')
            if any(value != 0.0 for value in (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)):
                raise ValueError('Somente velocidade linear x e angular z são suportadas')
            accepted = self.controller.command(msg.linear.x, msg.angular.z, self.time_seconds())
            if not accepted:
                self.get_logger().warning('ODrive desconectada; comando descartado', throttle_duration_sec=5.0)
        except ValueError as error:
            # An invalid command must neither refresh the watchdog nor leave
            # an earlier movement running indefinitely.
            try:
                self.controller.stop()
            except Exception as stop_error:
                self.handle_driver_failure(stop_error)
            self.get_logger().error(str(error))
        except Exception as error:
            self.handle_driver_failure(error)

    def fast_timer(self):
        try:
            state, expired = self.controller.step(self.time_seconds())
            if state is None:
                return
            if expired:
                self.get_logger().warning('Comandos /cmd_vel interrompidos; motores parados')
            stamp = self.get_clock().now().to_msg()
            self.publish_odometry(state, stamp)
            if self.publish_raw_odom:
                for name, publisher in self.raw_publishers.items():
                    publisher.publish(Float64(data=float(getattr(self.controller.last_sample, name))))
            if self.publish_current:
                self.current_publishers['left'].publish(Float64(data=float(self.controller.driver.left_current())))
                self.current_publishers['right'].publish(Float64(data=float(self.controller.driver.right_current())))
        except Exception as error:
            self.handle_driver_failure(error)

    def publish_odometry(self, state, stamp):
        z, w = math.sin(state.yaw / 2), math.cos(state.yaw / 2)
        if self.publish_odom:
            msg = Odometry()
            msg.header.stamp, msg.header.frame_id = stamp, self.odom_frame
            msg.child_frame_id = self.base_frame
            msg.pose.pose.position.x, msg.pose.pose.position.y = state.x, state.y
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = z, w
            msg.twist.twist.linear.x = state.linear_velocity
            msg.twist.twist.angular.z = state.angular_velocity
            self.odom_publisher.publish(msg)
        if self.publish_odom_tf:
            transform = TransformStamped()
            transform.header.stamp, transform.header.frame_id = stamp, self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x, transform.transform.translation.y = state.x, state.y
            transform.transform.rotation.z, transform.transform.rotation.w = z, w
            self.tf_publisher.sendTransform(transform)

    def handle_driver_failure(self, error):
        self.get_logger().error(f'Falha da ODrive: {error}')
        try:
            self.shutdown()
        except Exception as cleanup_error:
            self.get_logger().error(f'Falha ao encerrar ODrive: {cleanup_error}')

    def shutdown(self):
        if self.controller is not None:
            self.controller.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OdriveNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                try:
                    node.shutdown()
                finally:
                    node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
