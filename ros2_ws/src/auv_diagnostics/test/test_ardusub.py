import json
import math
import struct
import unittest

from auv_diagnostics.ardusub import (
    MotorWatchdog, normalize_mavlink, physics_packet, pwm_to_commands, rotate_body_to_ned, servo_packet,
)


class ArduSubTests(unittest.TestCase):
    def test_servo_protocol_16_and_32(self):
        for magic, channels in ((18458, 16), (29569, 32)):
            packet = struct.pack(f'<HHI{channels}H', magic, 400, 123, *([1500]*channels))
            result = servo_packet(packet)
            self.assertEqual(result['frame'], 123)
            self.assertEqual(len(result['pwm_us']), channels)
        for broken in (b'', struct.pack('<HHI', 1, 400, 1),
                       struct.pack('<HHI16H', 18458, 0, 1, *([1500]*16))):
            with self.assertRaises(ValueError):
                servo_packet(broken)

    def test_pwm_units_neutral_reverse_disabled_and_range(self):
        self.assertEqual(pwm_to_commands([1100, 1500, 1900, 0]), [-1, 0, 1, 0])
        self.assertEqual(pwm_to_commands([1300, 1700, 1500, 1500]), [-0.5, 0.5, 0, 0])
        for invalid in ([1000]*4, [2000]*4, [float('nan')]*4, [1500]):
            with self.assertRaises(ValueError):
                pwm_to_commands(invalid)

    def test_physics_world_velocity_rotation_and_quaternion_order(self):
        q = [0, 0, math.sqrt(0.5), math.sqrt(0.5)]
        velocity = rotate_body_to_ned([2, 0, 0], q)
        self.assertAlmostEqual(velocity[0], 0)
        self.assertAlmostEqual(velocity[1], 2)
        raw = physics_packet(2.5, [10, 20, 5], [2, 0, 0], q, [0]*3, [0, 0, -9.81])
        self.assertTrue(raw.startswith(b'\n') and raw.endswith(b'\n'))
        state = json.loads(raw)
        self.assertEqual(state['position'], [10, 20, 5])
        self.assertAlmostEqual(state['quaternion'][0], q[3])
        self.assertEqual(state['imu']['accel_body'][2], -9.81)
        with self.assertRaises(ValueError):
            physics_packet(float('nan'), [0]*3, [0]*3, q, [0]*3, [0]*3)

    def test_mavlink_pressure_units_and_no_invented_rpm(self):
        pressure = normalize_mavlink({'mavpackettype': 'SCALED_PRESSURE2',
                                      'press_abs': 1500.25, 'press_diff': 100.0})
        self.assertEqual(pressure['absolute_pressure_pa'], 150025)
        self.assertEqual(pressure['sensor_index'], 2)
        self.assertIsNone(pressure['depth_m'])
        servo = normalize_mavlink({'mavpackettype': 'SERVO_OUTPUT_RAW',
                                   **{f'servo{i}_raw': 1500 for i in range(1, 5)}})
        self.assertEqual(servo['motor_pwm_us'], [1500]*4)
        self.assertIsNone(servo['rpm'])

    def test_stale_duplicate_and_sensor_loss_stop_motors(self):
        guard = MotorWatchdog()
        self.assertEqual(guard.output(1), [0]*4)
        packet = {'frame': 100, 'pwm_us': [1700]*4}
        self.assertTrue(guard.receive(packet, 1))
        self.assertEqual(guard.output(1.1), [0.5]*4)
        self.assertEqual(guard.output(1.1, sensors_fresh=False), [0]*4)
        self.assertFalse(guard.receive(packet, 1.4))
        self.assertEqual(guard.output(1.6), [0]*4)
        self.assertFalse(guard.receive({'frame': 99, 'pwm_us': [1900]*4}, 1.7))
        self.assertTrue(guard.receive({'frame': 101, 'pwm_us': [1300]*4}, 1.8))
        self.assertEqual(guard.output(1.9), [-0.5]*4)


if __name__ == '__main__':
    unittest.main()
