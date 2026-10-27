"""Двусторонний мост Stonefish ROS2 и ArduSub SITL (UDP JSON)."""

import argparse
import json
from pathlib import Path
import socket
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_ws/src/auv_diagnostics'))

import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from std_msgs.msg import Float64, Float64MultiArray, String  # noqa: E402
from auv_diagnostics.ardusub import MotorWatchdog, physics_packet, servo_packet  # noqa: E402
from auv_diagnostics.monitoring import write_json  # noqa: E402


def vec(v):
    return [v.x, v.y, v.z]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, default=9002)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', args.port))
    sock.setblocking(False)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('ardusub_physics_bridge')
    cache = {}
    watchdog = MotorWatchdog()
    commands = node.create_publisher(Float64MultiArray, '/auv_01/thruster_command', 10)
    status = node.create_publisher(String, '/auv_01/autopilot/bridge_status', 10)
    counters = {'servo_packets': 0, 'physics_replies': 0, 'invalid_packets': 0,
                'duplicate_or_old_packets': 0, 'sensor_timeouts': 0}
    peer, last_time = None, None
    last_fresh = False
    log = (args.output/'bridge.jsonl').open('w')

    def remember(key, message):
        cache[key] = (message, time.monotonic())

    subscriptions = [node.create_subscription(kind, topic,
                     lambda msg, k=key: remember(k, msg), qos_profile_sensor_data)
                     for key, topic, kind in [
                         ('imu', '/auv_01/imu', Imu),
                         ('truth', '/evaluation/auv_01/ground_truth', Odometry),
                         ('clock', '/stonefish/physics_time', Float64)]]

    def tick():
        nonlocal peer, last_time, last_fresh
        now = time.monotonic()
        fresh = all(k in cache and now-cache[k][1] < 0.3 for k in ('imu', 'truth', 'clock'))
        if last_fresh and not fresh:
            counters['sensor_timeouts'] += 1
        last_fresh = fresh
        for _ in range(500):
            try:
                raw, remote = sock.recvfrom(512)
            except BlockingIOError:
                break
            if peer is not None and peer != remote:
                counters['invalid_packets'] += 1
                continue
            try:
                packet = servo_packet(raw)
                peer = remote
                accepted = watchdog.receive(packet, now)
            except ValueError:
                counters['invalid_packets'] += 1
                continue
            peer = remote
            counters['servo_packets'] += 1
            if accepted:
                log.write(json.dumps({'received_monotonic_s': now, **packet})+'\n')
            else:
                counters['duplicate_or_old_packets'] += 1
        commands.publish(Float64MultiArray(data=watchdog.output(now, fresh)))
        if not fresh or peer is None:
            return
        stamp = cache['clock'][0].data
        if last_time is not None and stamp <= last_time:
            return
        imu = cache['imu'][0]
        odom = cache['truth'][0]
        q = odom.pose.pose.orientation
        payload = physics_packet(stamp, vec(odom.pose.pose.position),
                                 vec(odom.twist.twist.linear), [q.x, q.y, q.z, q.w],
                                 vec(imu.angular_velocity), vec(imu.linear_acceleration))
        sock.sendto(payload, peer)
        last_time = stamp
        counters['physics_replies'] += 1
        status.publish(String(data=json.dumps({'physics_time_s': stamp,
                       'frame': watchdog.frame, 'command': watchdog.output(now, fresh),
                       'sensors_fresh': fresh,
                       'command_timeout': watchdog.received_at is None or now-watchdog.received_at > watchdog.timeout,
                       **counters})))

    timer = node.create_timer(0.005, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        commands.publish(Float64MultiArray(data=[0.0]*4))
        write_json(args.output/'bridge_summary.json', {**counters,
                   'last_physics_time_s': last_time, 'last_servo_frame': watchdog.frame})
        log.close()
        sock.close()
        node.destroy_timer(timer)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
