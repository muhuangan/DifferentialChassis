"""Subscribe to /cmd_vel and send signed int32 wheel rpm at a fixed rate."""

import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist

from .protocol import CommandState, map_speeds, pack_speeds


class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        defaults = {
            'port': '/dev/ttyS1', 'baudrate': 115200,
            'kv': 200.0, 'kw': 100.0, 'rpm_limit': 100,
            'left_direction': 1, 'right_direction': 1,
            'send_hz': 50.0, 'command_timeout': 0.3,
            'write_timeout': 0.01, 'dry_run': False,
        }
        self.cfg = {
            name: self.declare_parameter(name, value).value
            for name, value in defaults.items()
        }
        for name in ('kv', 'kw', 'send_hz', 'command_timeout', 'write_timeout'):
            if not math.isfinite(self.cfg[name]) or self.cfg[name] <= 0:
                raise ValueError(name + ' must be finite and positive')
        if not 0 < self.cfg['rpm_limit'] <= 2147483647:
            raise ValueError('rpm_limit must be in 1..2147483647')
        if self.cfg['baudrate'] <= 0:
            raise ValueError('baudrate must be positive')
        for name in ('left_direction', 'right_direction'):
            if self.cfg[name] not in (-1, 1):
                raise ValueError(name + ' must be +1 or -1')
        self.command = CommandState(self.cfg['command_timeout'])
        self.port = None
        self.last_log = -float('inf')
        self.last_warning = -float('inf')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 1)
        # Hardware sending uses a steady clock even if use_sim_time is enabled.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0 / self.cfg['send_hz'], self.on_timer, clock=self.steady_clock)
        if not self.cfg['dry_run']:
            self.port = serial.Serial(
                port=self.cfg['port'], baudrate=self.cfg['baudrate'],
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0,
                write_timeout=self.cfg['write_timeout'], exclusive=True)
        self.get_logger().info(
            f"12-byte int32 LE; port={self.cfg['port']}, "
            f"baud={self.cfg['baudrate']}, dry_run={self.cfg['dry_run']}")

    def on_cmd(self, msg):
        now = time.monotonic()
        try:
            speeds = map_speeds(
                msg.linear.x, msg.angular.z, self.cfg['kv'], self.cfg['kw'],
                self.cfg['rpm_limit'], self.cfg['left_direction'],
                self.cfg['right_direction'])
        except ValueError:
            speeds = (0, 0)
            if now - self.last_warning >= 1.0:
                self.get_logger().warning('Invalid cmd_vel: sending zero rpm')
                self.last_warning = now
        self.command.update(speeds, now)

    def on_timer(self):
        now = time.monotonic()
        left, right = self.command.current(now)
        packet = pack_speeds(left, right)
        if self.cfg['dry_run']:
            if now - self.last_log >= 0.5:
                self.get_logger().info(
                    f'L={left} R={right} rpm | {packet.hex(" ").upper()}')
                self.last_log = now
            return
        # Fail explicitly on UART errors; never silently keep replaying commands.
        if self.port.write(packet) != len(packet):
            raise serial.SerialException('Incomplete UART write')

    def close_port(self):
        self.timer.cancel()
        if self.port is not None and self.port.is_open:
            try:
                packet = pack_speeds(0, 0)
                if self.port.write(packet) != len(packet):
                    self.get_logger().error('Final zero packet was incomplete')
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'Could not send final zero packet: {exc}')
            finally:
                self.port.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    failed = False
    try:
        node = SerialNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (serial.SerialException, OSError, ValueError) as exc:
        failed = True
        print(f'serial_node: {exc}', flush=True)
    finally:
        try:
            if node is not None:
                node.close_port()
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
