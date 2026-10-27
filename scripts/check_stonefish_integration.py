"""Сбор реальных показаний Stonefish и запись файлов первого агента."""

import argparse
import math
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from uuid import uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_ws/src/auv_diagnostics'))

import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import FluidPressure, Imu  # noqa: E402
from std_msgs.msg import Float64MultiArray, String  # noqa: E402
from stonefish_ros2.msg import DVL, ThrusterState  # noqa: E402

from auv_diagnostics.monitoring import (  # noqa: E402
    THRUSTER_IDS, ned_to_enu, orientation_to_enu, summarize, write_json,
)
from auv_diagnostics.network import replay  # noqa: E402


def vector(v):
    return [v.x, v.y, v.z]


def quaternion(q):
    return [q.x, q.y, q.z, q.w]


def command(elapsed):
    if elapsed < 2:
        return [0.0]*4
    if elapsed < 9:
        return [0.4, 0.4, 0.0, 0.0]
    if elapsed < 13:
        return [0.2, 0.5, 0.0, 0.0]
    if elapsed < 17:
        return [-0.35, -0.35, 0.0, 0.0]
    if elapsed < 20:
        return [0.0, 0.0, 0.2, 0.2]
    return [0.0]*4


def window(rows, run_id, index):
    start = index*5.0
    return {'schema_version': '1.0', 'run_id': run_id, 'vehicle_id': 'auv_01',
            'window_id': f'w{index:04}', 'frame_id': 'map_enu', 'body_frame_id': 'body_frd',
            'window': {'start_s': start, 'end_s': start+5, 'clock_domain': 'ros_wall'},
            'clock_origin_ros_s': rows[0]['stamp_ros_s']-rows[0]['stamp_s'] if rows else None,
            'complete': True, 'acceleration_includes_gravity': True,
            'source_rates_hz': {'pressure': 10, 'imu': 50, 'dvl': 10,
                                'navigation': 10, 'thruster_state': 100},
            'samples': [s for s in rows if start <= s['stamp_s'] < start+5]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', type=Path, required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--external-control', action='store_true',
                        help='Команды подаёт мост ArduSub, проверка их не публикует')
    parser.add_argument('--finish-grace', type=float, default=0.0)
    args = parser.parse_args()
    scenario = args.scenario.resolve(strict=True)
    run_id = 'integration-'+uuid4().hex[:12]
    output = args.output_root/run_id
    output.mkdir(parents=True)
    cache, counts, rows, truth, commands = {}, {}, [], [], []
    rclpy.init()
    node = rclpy.create_node('stonefish_integration_check')

    def remember(key, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec/1e9
        cache[key] = (message, stamp)
        counts[key] = counts.get(key, 0)+1

    subscriptions = []
    for key, topic, kind in [
        ('pressure', '/auv_01/pressure', FluidPressure), ('imu', '/auv_01/imu', Imu),
        ('dvl', '/auv_01/dvl', DVL), ('navigation', '/auv_01/navigation', Odometry),
        ('thruster_state', '/auv_01/thruster_state', ThrusterState),
        ('truth', '/evaluation/auv_01/ground_truth', Odometry),
    ]:
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, k=key: remember(k, msg), qos_profile_sensor_data))
    autopilot = {}

    def receive_autopilot(message):
        autopilot['state'] = json.loads(message.data)
        autopilot['received_at'] = time.monotonic()

    if args.external_control:
        subscriptions.append(node.create_subscription(
            String, '/auv_01/autopilot/telemetry', receive_autopilot, 10))
    publisher = (None if args.external_control else node.create_publisher(
        Float64MultiArray, '/auv_01/thruster_command', 10))
    window_publisher = node.create_publisher(String, '/auv_01/telemetry_window', 10)
    summary_publisher = node.create_publisher(String, '/auv_01/monitoring_summary', 10)
    published_windows = set()
    process = None
    try:
        with (output/'simulator.log').open('w') as log:
            process = subprocess.Popen([
                'ros2', 'run', 'stonefish_ros2', 'stonefish_simulator_nogpu',
                str(scenario.parent)+'/', str(scenario), '100.0',
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+30
            while len(cache) < 6 and time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError('Simulator stopped: see simulator.log')
                rclpy.spin_once(node, timeout_sec=0.05)
            if len(cache) != 6:
                raise RuntimeError(f'Missing sources: {set(counts)}')
            epoch = node.get_clock().now().nanoseconds/1e9
            start = time.monotonic()
            next_sample = 0.0
            while time.monotonic()-start < 25:
                rclpy.spin_once(node, timeout_sec=0.005)
                if process.poll() is not None:
                    raise RuntimeError('Simulator stopped during capture')
                elapsed = time.monotonic()-start
                if elapsed < next_sample:
                    continue
                next_sample += 0.1
                cmd = command(elapsed)
                if publisher is not None:
                    publisher.publish(Float64MultiArray(data=cmd))
                now_ros = node.get_clock().now().nanoseconds/1e9
                elapsed_ros = now_ros-epoch
                if not args.external_control:
                    commands.append({'stamp_s': elapsed_ros, 'command': cmd})
                imu, nav, dvl = (cache[k][0] for k in ('imu', 'navigation', 'dvl'))
                state = cache['thruster_state'][0]
                if any(len(getattr(state, k)) != 4 for k in ('rpm', 'setpoint', 'thrust', 'torque')):
                    raise RuntimeError('Unexpected thruster mapping')
                row = {'stamp_s': elapsed_ros, 'stamp_ros_s': now_ros, 'sequence': len(rows),
                       'sources_stamp_ros_s': {k: v[1] for k, v in cache.items() if k != 'truth'},
                       'sources_age_s': {k: max(0, now_ros-v[1]) for k, v in cache.items()
                                         if k != 'truth'},
                       'pressure_pa': cache['pressure'][0].fluid_pressure,
                       'depth_m': cache['pressure'][0].fluid_pressure/(1031*9.81),
                       'orientation_xyzw': orientation_to_enu(quaternion(imu.orientation)),
                       'angular_velocity_rad_s': vector(imu.angular_velocity),
                       'linear_acceleration_m_s2': vector(imu.linear_acceleration),
                       'dvl_velocity_body_m_s': vector(dvl.velocity),
                       'dvl_altitude_m': dvl.altitude,
                       'dvl_valid': dvl.altitude >= 0 and all(math.isfinite(x)
                                                            for x in vector(dvl.velocity)),
                       'dvl_validity_method': 'bottom_range_available',
                       'position_estimate_m': ned_to_enu(vector(nav.pose.pose.position)),
                       'velocity_estimate_body_m_s': vector(nav.twist.twist.linear),
                       'thrusters': [
                           {'id': ident, 'command': state.setpoint[i]/400.0,
                            'setpoint_rad_s': state.setpoint[i],
                            'command_sent': None if args.external_control else cmd[i], 'command_unit': 'normalized',
                            'rpm': state.rpm[i], 'thrust_n': state.thrust[i],
                            'torque_nm': state.torque[i]}
                           for i, ident in enumerate(THRUSTER_IDS)],
                       'expected_motion': None}
                if args.external_control:
                    row['command_source'] = 'ardusub_sitl'
                    state = dict(autopilot.get('state', {}))
                    state['receiver_age_s'] = (time.monotonic()-autopilot['received_at']
                                                if 'received_at' in autopilot else None)
                    row['autopilot'] = state
                rows.append(row)
                for index in range(min(5, int(elapsed_ros//5))):
                    if index not in published_windows:
                        incoming = window(rows, run_id, index)
                        summary = summarize(incoming)
                        write_json(output/f'telemetry_window_{index}.json', incoming)
                        write_json(output/f'monitoring_summary_{index}.json', summary)
                        window_publisher.publish(String(data=json.dumps(incoming)))
                        summary_publisher.publish(String(data=json.dumps(summary)))
                        published_windows.add(index)
                ground = cache['truth'][0]
                truth.append({'stamp_s': elapsed_ros, 'stamp_ros_s': cache['truth'][1],
                              'position_m': ned_to_enu(vector(ground.pose.pose.position)),
                              'orientation_xyzw': orientation_to_enu(
                                  quaternion(ground.pose.pose.orientation)),
                              'frame_id': 'map_enu'})
            for index in range(5):
                if index not in published_windows:
                    incoming = window(rows, run_id, index)
                    window_publisher.publish(String(data=json.dumps(incoming)))
                    summary_publisher.publish(String(data=json.dumps(summarize(incoming))))
            if args.finish_grace > 0:
                write_json(args.output_root/'capture_ready.json', {'run_id': run_id})
                deadline = time.monotonic()+args.finish_grace
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.01)
            if publisher is not None:
                publisher.publish(Float64MultiArray(data=[0.0]*4))
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.shutdown()
        write_json(output/'capture.json', {'samples': rows, 'source_counts': counts})
        write_json(output/'evaluation_ground_truth.json', truth)
        write_json(output/'commands.json', commands)
    for index in range(5):
        incoming = window(rows, run_id, index)
        write_json(output/f'telemetry_window_{index}.json', incoming)
        write_json(output/f'monitoring_summary_{index}.json', summarize(incoming))
    packets = [{'sequence': s['sequence'], 'sent_s': s['stamp_s'], 'sample': s} for s in rows]
    network_results = {}
    for name, parameters in [
        ('normal', {}), ('delay', {'delay_s': 0.7}), ('loss', {'loss': 0.2}),
        ('node_outage', {'node_outage': (5, 9)}),
        ('link_outage', {'link_outage': (12, 16)}),
    ]:
        outcome = replay(packets, **parameters)
        network_results[name] = outcome
        evaluation = outcome.pop('evaluation_events')
        write_json(output/f'network_{name}.json', outcome)
        write_json(output/f'evaluation_network_{name}.json', evaluation)
        delivered = []
        for delivery in outcome['deliveries']:
            sample = dict(delivery['packet']['sample'])
            latency = delivery['received_s']-delivery['packet']['sent_s']
            sample['received_s'] = delivery['received_s']
            sample['sources_age_s'] = {k: age+latency for k, age
                                       in sample['sources_age_s'].items()}
            delivered.append(sample)
        delivered.sort(key=lambda s: s['stamp_s'])
        folder = output/'network'/name
        folder.mkdir(parents=True)
        for index in range(5):
            incoming = window(delivered, run_id, index)
            a, b = index*5, (index+1)*5
            sent = sum(a <= p['sent_s'] < b for p in packets)
            received = len(incoming['samples'])
            delays = [s['received_s']-s['stamp_s'] for s in incoming['samples']]
            incoming['communication'] = {
                'sent_count': sent, 'received_unique_count': received,
                'loss_pct': 100*(sent-received)/sent if sent else None,
                'latency_mean_ms': 1000*sum(delays)/len(delays) if delays else None,
                'observed_events': [e for e in outcome['observed_events']
                                    if a <= e['stamp_s'] < b]}
            write_json(folder/f'telemetry_window_{index}.json', incoming)
            write_json(folder/f'monitoring_summary_{index}.json', summarize(incoming))
    rpms = [t['rpm'] for s in rows for t in s['thrusters'][:2]]
    checks = {'sources': all(counts[k] >= 10 for k in counts),
              'frames': len(rows) >= 240,
              'forward_rpm': max(rpms) > 1000, 'reverse_rpm': min(rpms) < -1000,
              'movement': math.dist(truth[0]['position_m'], truth[-1]['position_m']) > 0.1,
              'dvl_bottom_available': sum(s['dvl_valid'] for s in rows) >= len(rows)*0.9,
              'normalized_commands': all(abs(t['command']) <= 1 for s in rows
                                          for t in s['thrusters']),
              'network_normal_delivery': network_results['normal']['loss_pct'] == 0,
              'network_delay': abs(network_results['delay']['latency_mean_ms']-700) < 0.01,
              'network_packet_loss': 0 < network_results['loss']['loss_pct'] < 100,
              'network_outage_recovery': all(
                  [e['event'] for e in network_results[k]['observed_events']]
                  == ['node_unreachable', 'node_reachable']
                  for k in ('node_outage', 'link_outage')),
              'monitoring_windows': all(summarize(window(rows, run_id, i))['status'] == 'ok'
                                        for i in range(5))}
    if args.external_control:
        checks['forward_rpm'] = max(rpms) > 100
        checks['reverse_rpm'] = min(rpms) < -100
        checks['movement'] = max(math.dist(truth[0]['position_m'], s['position_m'])
                                 for s in truth) > 0.1
        checks['autopilot_heartbeat'] = any(s.get('autopilot', {}).get('connected') for s in rows)
        checks['autopilot_armed'] = any(s.get('autopilot', {}).get('armed') for s in rows)
        checks['autopilot_servo_telemetry'] = any(
            'SERVO_OUTPUT_RAW' in s.get('autopilot', {}).get('messages', {}) for s in rows)
    write_json(output/'verification.json', {'run_id': run_id, 'checks': checks,
               'source_counts': counts, 'clock_domain': 'ros_wall',
               'clock_origin_ros_s': epoch, 'diagnostic_agent_connected': False,
               'notes': ['Окна измерены по часам ROS, а не по времени физического решателя.',
                         'Навигация получена из отдельной одометрии с заданным шумом.',
                         'Сетевые нарушения воспроизведены программно на записанной телеметрии.']})
    if not all(checks.values()):
        raise RuntimeError(f'Integration check failed: {checks}; output={output}')
    print(f'STONEFISH_INTEGRATION_PASS {output}')


if __name__ == '__main__':
    main()
