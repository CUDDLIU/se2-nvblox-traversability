import json
import unittest
from motion_status import HEADER, MAGIC, NAMES, heartbeat, parse_packet, split_joints


class ProtocolTests(unittest.TestCase):
    def packet(self, value):
        data = json.dumps(value).encode()
        return HEADER.pack(MAGIC, len(data), 7, 1) + data

    def report(self):
        values = [float(i)/10 for i in range(16)]
        return {'Type': 1002, 'Command': 4,
                'Items': {'MotorStatus': {'Joint': values, **dict(zip(NAMES, values))}}}

    def test_only_heartbeat_and_wrapped_sequence(self):
        sequence, patrol = parse_packet(heartbeat(65537))
        self.assertEqual(sequence, 1)
        self.assertEqual((patrol['Type'], patrol['Command'], patrol['Items']), (100, 100, {}))

    def test_mixed_units_are_separated_without_wheel_positions(self):
        _, patrol = parse_packet(self.packet({'PatrolDevice': self.report()}))
        angles, speeds = split_joints(patrol)
        self.assertEqual(len(angles), 12)
        self.assertEqual([n for n, _ in speeds], [NAMES[i] for i in (3, 7, 11, 15)])
        self.assertEqual([v for _, v in speeds], [.3, .7, 1.1, 1.5])
        self.assertNotIn('LeftFrontWheel', [n for n, _ in angles])

    def test_named_fields_supported_but_contradictions_rejected(self):
        p = self.report()
        expected = split_joints(p)
        del p['Items']['MotorStatus']['Joint']
        self.assertEqual(split_joints(p), expected)
        p = self.report()
        p['Items']['MotorStatus']['LeftFrontHipX'] = 99
        with self.assertRaises(ValueError):
            split_joints(p)

    def test_invalid_envelopes_and_nonfinite_values_rejected(self):
        packet = self.packet({'PatrolDevice': self.report()})
        for bad in (packet[:8], packet[:-1], packet+b'x', b'bad!' + packet[4:],
                    packet[:8]+b'\x00'+packet[9:], self.packet([]),
                    self.packet({'PatrolDevice': {'Items': {'x': float('nan')}}})):
            with self.subTest(packet=bad[:24]), self.assertRaises(ValueError):
                parse_packet(bad)
        for bad in ([0]*15, [0]*15+[True], [0]*15+[float('inf')]):
            p = self.report()
            p['Items']['MotorStatus'] = {'Joint': bad}
            with self.assertRaises(ValueError):
                split_joints(p)


if __name__ == '__main__':
    unittest.main()
