"""Offline node behavior tests with ROS/UART doubles, NOT hardware validation."""

import importlib.util
from pathlib import Path
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.log = Mock()

    def declare_parameter(self, name, default):
        return SimpleNamespace(value=self.overrides.get(name, default))

    def create_subscription(self, *args):
        return Mock()

    def create_timer(self, *args, **kwargs):
        return Mock()

    def get_logger(self):
        return self.log


class FakePort:
    def __init__(self, **kwargs):
        self.is_open = True
        self.packets = []

    def write(self, packet):
        self.packets.append(packet)
        return len(packet)

    def close(self):
        self.is_open = False


def load_node_with_doubles():
    modules = {name: ModuleType(name) for name in (
        'rclpy', 'rclpy.clock', 'rclpy.executors', 'rclpy.node',
        'serial', 'geometry_msgs', 'geometry_msgs.msg')}
    modules['rclpy.node'].Node = FakeNode
    modules['rclpy.clock'].Clock = lambda **kwargs: object()
    modules['rclpy.clock'].ClockType = SimpleNamespace(STEADY_TIME=1)
    modules['rclpy.executors'].ExternalShutdownException = type('Shutdown', (Exception,), {})
    modules['geometry_msgs.msg'].Twist = object
    uart = modules['serial']
    uart.Serial = FakePort
    uart.SerialException = type('SerialException', (Exception,), {})
    uart.EIGHTBITS, uart.PARITY_NONE, uart.STOPBITS_ONE = 8, 'N', 1
    path = Path(__file__).resolve().parents[1] / 'chassis_communication' / 'serial_node.py'
    spec = importlib.util.spec_from_file_location('chassis_communication._offline_node', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict('sys.modules', modules):
        spec.loader.exec_module(module)
    return module


class SerialNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_node_with_doubles()

    def setUp(self):
        FakeNode.overrides = {}
        self.node = self.module.SerialNode()

    def tearDown(self):
        self.node.close_port()
        FakeNode.overrides = {}

    @staticmethod
    def cmd(v, w):
        return SimpleNamespace(linear=SimpleNamespace(x=v), angular=SimpleNamespace(z=w))

    def test_received_twist_to_wire_and_stale_zero(self):
        with patch.object(self.module.time, 'monotonic', return_value=1.0):
            self.node.on_cmd(self.cmd(0, 0.6))
            self.node.on_timer()
        self.assertEqual(self.node.port.packets[-1],
                         bytes.fromhex('AA 55 C4 FF FF FF 3C 00 00 00 0D 0A'))
        with patch.object(self.module.time, 'monotonic', return_value=1.31):
            self.node.on_timer()
        self.assertEqual(self.node.port.packets[-1],
                         bytes.fromhex('AA 55 00 00 00 00 00 00 00 00 0D 0A'))

    def test_invalid_command_cancels_previous_motion(self):
        with patch.object(self.module.time, 'monotonic', return_value=1.0):
            self.node.on_cmd(self.cmd(0.3, 0))
            self.node.on_cmd(self.cmd(float('nan'), 0))
            self.node.on_timer()
        self.assertEqual(self.node.port.packets[-1][2:10], bytes(8))

    def test_partial_write_and_disconnect_raise(self):
        with patch.object(self.node.port, 'write', return_value=4):
            with self.assertRaises(self.module.serial.SerialException):
                self.node.on_timer()
        with patch.object(self.node.port, 'write', side_effect=OSError('Disconnected')):
            with self.assertRaises(OSError):
                self.node.on_timer()

    def test_shutdown_attempts_zero_and_closes(self):
        self.node.close_port()
        self.assertEqual(self.node.port.packets[-1][2:10], bytes(8))
        self.assertFalse(self.node.port.is_open)

    def test_dry_run_never_opens_port(self):
        FakeNode.overrides = {'dry_run': True}
        with patch.object(self.module.serial, 'Serial', side_effect=AssertionError('Opened')):
            node = self.module.SerialNode()
            node.on_timer()
            node.close_port()
        self.assertIsNone(node.port)


if __name__ == '__main__':
    unittest.main()
