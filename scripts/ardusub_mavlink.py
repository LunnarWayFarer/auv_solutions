"""MAVLink-телеметрия ArduSub в ROS2. Движение включается только --exercise."""

import argparse
import json
import socket
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_ws/src/auv_diagnostics'))

import rclpy  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from pymavlink import mavutil  # noqa: E402
from auv_diagnostics.monitoring import write_json  # noqa: E402
from auv_diagnostics.ardusub import normalize_mavlink  # noqa: E402


MESSAGES = {'HEARTBEAT': 0, 'ATTITUDE': 30, 'LOCAL_POSITION_NED': 32,
            'SCALED_PRESSURE': 29, 'SCALED_PRESSURE2': 137, 'SERVO_OUTPUT_RAW': 36}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--connection', default='tcp:127.0.0.1:5760')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--exercise', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    deadline = time.monotonic()+20
    while True:
        try:
            mav = mavutil.mavlink_connection(args.connection, source_system=255,
                                            source_component=190, autoreconnect=False)
            break
        except OSError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.2)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('ardusub_mavlink_telemetry')
    publisher = node.create_publisher(String, '/auv_01/autopilot/telemetry', 10)
    received, counts, acknowledgements = {}, {}, []
    started = time.monotonic()
    last_control = last_heartbeat = last_request = 0.0
    armed_at = None
    exercise_done = False
    disconnect_reason = None
    target_system, target_component = 1, 1
    log = (args.output/'mavlink.jsonl').open('w')
    try:
        while rclpy.ok():
            now = time.monotonic()
            try:
                if mav.port.recv(1, socket.MSG_PEEK) == b'':
                    raise ConnectionError('ArduSub TCP connection closed')
            except BlockingIOError:
                pass
            for _ in range(300):
                msg = mav.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_type() == 'BAD_DATA' or msg.get_srcSystem() != target_system:
                    continue
                kind = msg.get_type()
                record = msg.to_dict()
                counts[kind] = counts.get(kind, 0)+1
                if kind in MESSAGES:
                    received[kind] = (record, now)
                if kind == 'COMMAND_ACK':
                    acknowledgements.append(record)
                if kind in MESSAGES or kind in ('STATUSTEXT', 'COMMAND_ACK'):
                    log.write(json.dumps({'elapsed_s': now-started, 'message': record})+'\n')
            if now-last_heartbeat >= 1:
                mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                      mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_heartbeat = now
            heartbeat = received.get('HEARTBEAT')
            fresh = heartbeat is not None and now-heartbeat[1] < 2
            if fresh and now-last_request > 5:
                for kind, ident in MESSAGES.items():
                    if kind != 'HEARTBEAT':
                        mav.mav.command_long_send(target_system, target_component, 511, 0,
                                                  ident, 100000, 0, 0, 0, 0, 0)
                last_request = now
            armed = fresh and bool(heartbeat[0]['base_mode'] & 128)
            if args.exercise and fresh and not exercise_done and now-last_control >= 0.1:
                if heartbeat[0]['custom_mode'] != 19:
                    mav.mav.set_mode_send(target_system, 1, 19)
                elif not armed:
                    mav.mav.manual_control_send(target_system, 0, 0, 500, 0, 0)
                    if now-last_control >= 0.1:
                        mav.mav.command_long_send(target_system, target_component, 400, 0,
                                                  1, 0, 0, 0, 0, 0, 0)
                else:
                    if armed_at is None:
                        armed_at = now
                    elapsed = now-armed_at
                    x, z, yaw = 0, 500, 0
                    if 2 <= elapsed < 6:
                        x = 600
                    elif 6 <= elapsed < 10:
                        x = -600
                    elif 10 <= elapsed < 12:
                        yaw = 300
                    elif 12 <= elapsed < 14:
                        z = 650
                    elif elapsed >= 16:
                        mav.mav.command_long_send(target_system, target_component, 400, 0,
                                                  0, 0, 0, 0, 0, 0, 0)
                        exercise_done = True
                    mav.mav.manual_control_send(target_system, x, 0, z, yaw, 0)
                last_control = now
            state = {'connected': fresh, 'armed': bool(armed), 'exercise': args.exercise,
                     'messages': {kind: {'age_s': now-stamp, 'fields': fields,
                                         'normalized': normalize_mavlink(fields)}
                                  for kind, (fields, stamp) in received.items()}}
            publisher.publish(String(data=json.dumps(state)))
            rclpy.spin_once(node, timeout_sec=0.02)
    except KeyboardInterrupt:
        pass
    except OSError as error:
        disconnect_reason = str(error)
    finally:
        if args.exercise:
            try:
                mav.mav.manual_control_send(target_system, 0, 0, 500, 0, 0)
                mav.mav.command_long_send(target_system, target_component, 400, 0,
                                          0, 0, 0, 0, 0, 0, 0)
            except OSError:
                pass
        write_json(args.output/'mavlink_summary.json', {'message_counts': counts, 'disconnect_reason': disconnect_reason,
                   'exercise_requested': args.exercise, 'armed_observed': armed_at is not None,
                   'exercise_completed': exercise_done, 'command_acknowledgements': acknowledgements})
        log.close()
        mav.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
