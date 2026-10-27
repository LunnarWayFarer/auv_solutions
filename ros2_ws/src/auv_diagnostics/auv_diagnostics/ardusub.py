"""Обмен с JSON backend ArduSub SITL и преобразование PWM."""

import json
import math
import struct


def servo_packet(data):
    if len(data) < 8:
        raise ValueError('Short servo packet')
    magic, rate, frame = struct.unpack_from('<HHI', data)
    channels = {18458: 16, 29569: 32}.get(magic)
    if channels is None or len(data) != 8+2*channels or not 1 <= rate <= 2000:
        raise ValueError('Invalid servo packet')
    pwm = list(struct.unpack_from(f'<{channels}H', data, 8))
    return {'frame': frame, 'rate_hz': rate, 'pwm_us': pwm}


def pwm_to_commands(pwm, minimum=1100, neutral=1500, maximum=1900):
    if not minimum < neutral < maximum or len(pwm) < 4:
        raise ValueError('Invalid PWM configuration')
    result = []
    for value in pwm[:4]:
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError('Invalid PWM value')
        if value == 0:
            result.append(0.0)
        elif not minimum <= value <= maximum:
            raise ValueError('PWM outside configured motor range')
        else:
            result.append((value-neutral)/(maximum-neutral if value >= neutral
                                          else neutral-minimum))
    return result


def rotate_body_to_ned(vector, xyzw):
    if len(vector) != 3 or len(xyzw) != 4:
        raise ValueError('Invalid vector or quaternion')
    if not all(type(v) in (int, float) and math.isfinite(v) for v in [*vector, *xyzw]):
        raise ValueError('Nonfinite orientation or velocity')
    norm = math.sqrt(sum(v*v for v in xyzw))
    if norm < 1e-9:
        raise ValueError('Zero quaternion')
    x, y, z, w = [v/norm for v in xyzw]
    a, b, c = vector
    return [(1-2*(y*y+z*z))*a+2*(x*y-z*w)*b+2*(x*z+y*w)*c,
            2*(x*y+z*w)*a+(1-2*(x*x+z*z))*b+2*(y*z-x*w)*c,
            2*(x*z-y*w)*a+2*(y*z+x*w)*b+(1-2*(x*x+y*y))*c]


def physics_packet(timestamp, position, velocity_body, xyzw, gyro, accel):
    values = [timestamp, *position, *velocity_body, *xyzw, *gyro, *accel]
    if (len(position) != 3 or len(gyro) != 3 or len(accel) != 3
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)
            or timestamp < 0):
        raise ValueError('Invalid physics state')
    velocity = rotate_body_to_ned(velocity_body, xyzw)
    norm = math.sqrt(sum(v*v for v in xyzw))
    x, y, z, w = [v/norm for v in xyzw]
    state = {'timestamp': timestamp, 'imu': {'gyro': gyro, 'accel_body': accel},
             'position': position, 'velocity': velocity, 'quaternion': [w, x, y, z],
             'no_time_sync': True}
    return ('\n'+json.dumps(state, allow_nan=False, separators=(',', ':'))+'\n').encode()


class MotorWatchdog:
    def __init__(self, timeout=0.5):
        self.timeout = timeout
        self.frame = None
        self.received_at = None
        self.commands = [0.0]*4

    def receive(self, packet, now):
        if self.frame is not None and packet['frame'] <= self.frame:
            return False
        commands = pwm_to_commands(packet['pwm_us'])
        self.frame = packet['frame']
        self.received_at = now
        self.commands = commands
        return True

    def output(self, now, sensors_fresh=True):
        if (not sensors_fresh or self.received_at is None
                or now-self.received_at > self.timeout):
            return [0.0]*4
        return self.commands[:]


def normalize_mavlink(message):
    kind = message['mavpackettype']
    if kind == 'HEARTBEAT':
        return {'armed': bool(message['base_mode'] & 128),
                'mode_id': message['custom_mode'], 'system_status': message['system_status']}
    if kind == 'ATTITUDE':
        return {'attitude_rpy_rad': [message[k] for k in ('roll', 'pitch', 'yaw')],
                'angular_velocity_rad_s': [message[k] for k in ('rollspeed', 'pitchspeed', 'yawspeed')]}
    if kind == 'LOCAL_POSITION_NED':
        return {'position_ned_m': [message[k] for k in ('x', 'y', 'z')],
                'velocity_ned_m_s': [message[k] for k in ('vx', 'vy', 'vz')],
                'source': 'autopilot_navigation_estimate'}
    if kind in ('SCALED_PRESSURE', 'SCALED_PRESSURE2'):
        return {'absolute_pressure_pa': message['press_abs']*100,
                'differential_pressure_pa': message['press_diff']*100,
                'sensor_index': 1 if kind == 'SCALED_PRESSURE' else 2,
                'depth_m': None}
    if kind == 'SERVO_OUTPUT_RAW':
        return {'motor_pwm_us': [message[f'servo{i}_raw'] for i in range(1, 5)],
                'rpm': None, 'rpm_source': 'stonefish_thruster_state'}
    raise ValueError('Unsupported MAVLink telemetry message')
