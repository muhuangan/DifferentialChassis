import unittest

from chassis_communication.keyboard_state import KeyboardState


class KeyboardTests(unittest.TestCase):
    def setUp(self):
        self.state = KeyboardState(0.3, 0.5, 0.05, 0.1, 0.8, 1.0, 0.15)

    def test_direction_replaces_both_axes(self):
        self.state.feed('w', 0)
        self.assertEqual(self.state.velocity(0.01), (0.3, 0.0))
        self.state.feed('A', 0.02)
        self.assertEqual(self.state.velocity(0.03), (0.0, 0.5))
        self.state.feed('s', 0.04)
        self.assertEqual(self.state.velocity(0.05), (-0.3, 0.0))

    def test_release_timeout(self):
        self.state.feed('w', 1)
        self.assertEqual(self.state.velocity(1.2), (0.0, 0.0))

    def test_speed_keys_and_noise_do_not_extend_motion(self):
        self.state.feed('w', 1)
        self.state.feed('q', 1.1)
        self.state.feed('x', 1.14)
        self.assertEqual(self.state.velocity(1.2), (0.0, 0.0))

    def test_space_and_ctrl_c(self):
        self.state.feed('w', 1)
        self.state.feed(' ', 1.01)
        self.assertEqual(self.state.velocity(1.02), (0.0, 0.0))
        self.assertFalse(self.state.feed('\x03', 1.03))

    def test_arrow_sequences_do_not_trigger_uppercase_wasd(self):
        for key in '\x1b[A\x1b[D\x1bOB':
            self.state.feed(key, 1)
        self.assertEqual(self.state.velocity(1.01), (0.0, 0.0))
        self.state.feed('w', 1.02)
        self.assertEqual(self.state.velocity(1.03), (0.3, 0.0))

    def test_speed_limits(self):
        for _ in range(100):
            self.state.feed('q', 0)
        self.assertEqual((self.state.linear_speed, self.state.angular_speed), (0.8, 1.0))
        for _ in range(100):
            self.state.feed('e', 0)
        self.assertEqual((self.state.linear_speed, self.state.angular_speed), (0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
