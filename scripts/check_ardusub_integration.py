"""Запуск и проверка Stonefish, ArduSub, ROS2 и JSON первого агента."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros2_ws/src/auv_diagnostics'))
from auv_diagnostics.monitoring import write_json  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--binary', type=Path, default=Path('/opt/ardupilot/build/sitl/bin/ardusub'))
    args = parser.parse_args()
    output = args.output_root/('ardusub-'+uuid4().hex[:12])
    output.mkdir(parents=True)
    processes, logs = [], []

    def start(name, command, cwd=None):
        log = (output/f'{name}.log').open('w')
        logs.append(log)
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True, cwd=cwd)
        processes.append((name, proc))
        return proc

    import rclpy
    from std_msgs.msg import Float64MultiArray, String
    rclpy.init()
    node = rclpy.create_node('ardusub_integration_observer')
    observed, bridge_states, summaries = [], [], []
    node.create_subscription(Float64MultiArray, '/auv_01/thruster_command',
        lambda msg: observed.append((time.monotonic(), list(msg.data))), 10)
    node.create_subscription(String, '/auv_01/autopilot/bridge_status',
        lambda msg: bridge_states.append((time.monotonic(), json.loads(msg.data))), 10)
    node.create_subscription(String, '/auv_01/monitoring_summary',
        lambda msg: summaries.append(json.loads(msg.data)), 10)
    verification = {'checks': {'bridge_summary_written': False,
                              'mavlink_summary_written': False},
                    'diagnostic_agent_connected': False}
    stopped_at = None
    try:
        start('physics_bridge', [sys.executable, str(ROOT/'scripts/ardusub_bridge.py'),
                                 '--output', str(output)])
        autopilot = start('ardusub', [str(args.binary.resolve()), '--model', 'JSON:127.0.0.1',
                          '--speedup', '1', '--defaults', str(ROOT/'scenarios/ardusub.parm'),
                          '--home', '59,30,0,0', '--wipe'], cwd=output)
        time.sleep(1)
        start('mavlink_bridge', [sys.executable, str(ROOT/'scripts/ardusub_mavlink.py'),
                                '--output', str(output), '--exercise'])
        capture = start('capture', [sys.executable, str(ROOT/'scripts/check_stonefish_integration.py'),
                        '--scenario', str(ROOT/'scenarios/integration.scn'),
                        '--output-root', str(output), '--external-control', '--finish-grace', '4'])
        deadline = time.monotonic()+70
        while capture.poll() is None:
            if time.monotonic() > deadline:
                raise RuntimeError('Capture timeout')
            for name, process in processes:
                if (name != 'capture' and process.poll() is not None
                        and not (stopped_at is not None and name in ('ardusub', 'mavlink_bridge'))):
                    raise RuntimeError(f'{name} stopped; see {output}')
            if stopped_at is None and (output/'capture_ready.json').exists():
                autopilot.send_signal(signal.SIGINT)
                autopilot.wait(timeout=5)
                stopped_at = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.02)
        if capture.returncode != 0:
            raise RuntimeError(f'Capture failed; see {output}/capture.log')
        capture_dir = next(output.glob('integration-*'))
        captured = json.loads((capture_dir/'verification.json').read_text())
        verification['checks'].update(captured['checks'])
        verification['capture_directory'] = capture_dir.name
        after_stop = [(stamp, state) for stamp, state in bridge_states
                      if stopped_at is not None and stamp > stopped_at+0.6]
        neutral = [cmd for stamp, cmd in observed
                   if stopped_at is not None and stamp > stopped_at+0.6]
        verification['checks']['stopped_autopilot_neutral'] = (
            len(neutral) >= 10 and all(cmd == [0.0]*4 for cmd in neutral))
        verification['checks']['watchdog_with_live_physics'] = (
            len(after_stop) >= 10 and any(s['sensors_fresh'] and s['command_timeout']
                                         for _, s in after_stop)
            and after_stop[-1][1]['physics_time_s'] > after_stop[0][1]['physics_time_s'])
        verification['checks']['live_monitoring_summaries'] = (
            len(summaries) == 5 and all(s['status'] == 'ok' for s in summaries))
        write_json(output/'observed_monitoring_summaries.json', summaries)
    finally:
        for name, process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
        if (output/'bridge_summary.json').exists():
            verification['checks']['bridge_summary_written'] = True
            bridge = json.loads((output/'bridge_summary.json').read_text())
            verification['bridge'] = bridge
            verification['checks']['physics_exchange'] = bridge['physics_replies'] > 100
            verification['checks']['servo_exchange'] = bridge['servo_packets'] > 100
        if (output/'mavlink_summary.json').exists():
            verification['checks']['mavlink_summary_written'] = True
            mav = json.loads((output/'mavlink_summary.json').read_text())
            verification['mavlink'] = mav
            verification['checks']['exercise_completed'] = mav['exercise_completed']
        write_json(output/'verification.json', verification)
    if not verification['checks'] or not all(verification['checks'].values()):
        raise RuntimeError(f'Integration check failed; see {output}/verification.json')
    print(f'ARDUSUB_STONEFISH_PASS {output}')


if __name__ == '__main__':
    main()
