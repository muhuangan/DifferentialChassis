"""Interactive Linux terminal keyboard publisher; run in its own terminal."""

import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .keyboard_state import KeyboardState


class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        defaults = {
            'linear_speed': 0.30, 'angular_speed': 0.50,
            'linear_step': 0.05, 'angular_step': 0.10,
            'max_linear': 0.80, 'max_angular': 1.00,
            'publish_hz': 50.0, 'stop_delay': 0.15,
        }
        cfg = {
            name: self.declare_parameter(name, value).value
            for name, value in defaults.items()
        }
        for name, value in cfg.items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be finite and positive')
        if cfg['linear_speed'] > cfg['max_linear'] or cfg['angular_speed'] > cfg['max_angular']:
            raise ValueError('Initial speeds must not exceed their maximums')
        self.state = KeyboardState(**{k: v for k, v in cfg.items() if k != 'publish_hz'})
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0 / cfg['publish_hz'], self.publish_velocity, clock=self.steady_clock)
        self.last_display = 0.0

    def publish_velocity(self):
        now = time.monotonic()
        linear, angular = self.state.velocity(now)
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)
        if now - self.last_display >= 0.1:
            sys.stdout.write(
                f'\r v={linear:+.2f} m/s  w={angular:+.2f} rad/s | '
                f'target={self.state.linear_speed:.2f}/{self.state.angular_speed:.2f}    ')
            sys.stdout.flush()
            self.last_display = now


def main(args=None):
    if not sys.stdin.isatty():
        raise SystemExit('keyboard_node needs an interactive terminal')
    rclpy.init(args=args)
    node = None
    old_settings = None
    fd = sys.stdin.fileno()
    try:
        node = KeyboardNode()
        old_settings = termios.tcgetattr(fd)
        print('W/S: forward/back | A/D: turn | Q/E: speed +/- | Space: stop | Ctrl+C: exit')
        print('Terminal repeat timeout approximates release; this is not true key-up detection.')
        tty.setraw(fd)
        running = True
        while running and rclpy.ok():
            if select.select([fd], [], [], 0)[0]:
                data = os.read(fd, 64)
                if not data:
                    break
                for key in data.decode('ascii', errors='ignore'):
                    if not node.state.feed(key, time.monotonic()):
                        running = False
                        break
            if running:
                rclpy.spin_once(node, timeout_sec=0.005)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Restore the terminal even if ROS or serial-related startup fails.
        if old_settings is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        try:
            if node is not None and rclpy.ok():
                node.publisher.publish(Twist())
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            print('\nKeyboard stopped.')


if __name__ == '__main__':
    main()
