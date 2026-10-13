import struct
import unittest

from chassis_communication.protocol import CommandState, map_speeds, pack_speeds


class ProtocolTests(unittest.TestCase):
    def test_known_signed_packet(self):
        self.assertEqual(
            pack_speeds(60, -60),
            bytes.fromhex('AA 55 3C 00 00 00 C4 FF FF FF 0D 0A'))

    def test_full_int32_range_and_length(self):
        packet = pack_speeds(-2147483648, 2147483647)
        self.assertEqual(len(packet), 12)
        self.assertEqual(packet[2:10], bytes.fromhex('00 00 00 80 FF FF FF 7F'))
        with self.assertRaises(struct.error):
            pack_speeds(2147483648, 0)

    def test_tail_inside_payload_does_not_change_frame_length(self):
        packet = pack_speeds(0x0A0D, 0x55AA)
        self.assertEqual(len(packet), 12)
        self.assertEqual(packet[2:4], b'\x0d\x0a')
        self.assertEqual(packet[10:], b'\x0d\x0a')

    def test_motion_mapping(self):
        for v, w, expected in [
            (0.3, 0.0, (60, 60)), (-0.3, 0.0, (-60, -60)),
            (0.0, 0.5, (-50, 50)), (0.0, -0.5, (50, -50)),
            (0.3, 0.5, (10, 100)), (0.0, 0.0, (0, 0)),
        ]:
            with self.subTest(v=v, w=w):
                self.assertEqual(map_speeds(v, w, 200, 100, 100, 1, 1), expected)

    def test_limit_direction_and_rounding(self):
        self.assertEqual(map_speeds(2, 0, 200, 100, 100, -1, 1), (-100, 100))
        self.assertEqual(map_speeds(0.303, 0, 200, 100, 100, 1, 1), (61, 61))

    def test_nonfinite_commands(self):
        for v, w in [(float('nan'), 0), (0, float('inf')), (1e308, 0)]:
            with self.assertRaises(ValueError):
                map_speeds(v, w, 200, 100, 100, 1, 1)

    def test_startup_timeout_and_fresh_command(self):
        state = CommandState(0.3)
        self.assertEqual(state.current(0), (0, 0))
        state.update((60, -60), 1.0)
        self.assertEqual(state.current(1.2), (60, -60))
        self.assertEqual(state.current(1.31), (0, 0))
        state.update((-20, -20), 2.0)
        self.assertEqual(state.current(2.01), (-20, -20))


if __name__ == '__main__':
    unittest.main()
