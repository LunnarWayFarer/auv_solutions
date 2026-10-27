"""Проверка окна телеметрии и подготовка JSON-сводки первого агента."""

import json
import math
from pathlib import Path


THRUSTER_IDS = ('horizontal_left', 'horizontal_right', 'vertical_front', 'vertical_rear')


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)
                          + '\n', encoding='utf-8')


def ned_to_enu(vector):
    if len(vector) != 3 or not all(finite(x) for x in vector):
        raise ValueError('Expected a finite NED vector')
    return [vector[1], vector[0], -vector[2]]


def orientation_to_enu(q):
    if len(q) != 4 or not all(finite(x) for x in q):
        raise ValueError('Expected quaternion x,y,z,w')
    norm = math.sqrt(sum(x*x for x in q))
    if norm < 1e-9:
        raise ValueError('Zero quaternion')
    x, y, z, w = (x/norm for x in q)
    a = math.sqrt(0.5)
    return [a*(w+z), a*(w-z), a*(y-x), -a*(x+y)]


def stats(values, unit):
    values = [x for x in values if finite(x)]
    if not values:
        return {'valid': False, 'reason': 'no_valid_measurements', 'mean': None,
                'min': None, 'max': None, 'change': None, 'unit': unit}
    return {'valid': True, 'mean': sum(values)/len(values), 'min': min(values),
            'max': max(values), 'change': values[-1]-values[0], 'unit': unit}


def validate_window(window):
    if window.get('schema_version') != '1.0':
        raise ValueError('Unsupported schema_version')
    for key in ('run_id', 'vehicle_id', 'window_id', 'frame_id', 'body_frame_id'):
        if not isinstance(window.get(key), str) or not window[key]:
            raise ValueError(f'Missing {key}')
    clock = window['window']
    if clock.get('clock_domain') not in ('ros_wall', 'simulation'):
        raise ValueError('Unknown clock domain')
    if not all(finite(clock.get(k)) for k in ('start_s', 'end_s')):
        raise ValueError('Invalid window bounds')
    if clock['end_s'] <= clock['start_s']:
        raise ValueError('Empty time interval')
    if not isinstance(window.get('samples'), list):
        raise ValueError('samples must be an array')
    for source in ('pressure', 'imu', 'dvl', 'navigation', 'thruster_state'):
        rate = window.get('source_rates_hz', {}).get(source)
        if not finite(rate) or rate <= 0:
            raise ValueError(f'Invalid source rate: {source}')
    for key in ('ground_truth', 'expected_class', 'fault_label'):
        if key in window:
            raise ValueError('Evaluation fields cannot enter monitoring')
    previous = None
    for sample in window['samples']:
        stamp = sample.get('stamp_s')
        if not finite(stamp) or not clock['start_s'] <= stamp < clock['end_s']:
            raise ValueError('Sample outside half-open time window')
        if previous is not None and stamp <= previous:
            raise ValueError('Samples must have increasing stamps')
        previous = stamp
        for forbidden in ('ground_truth', 'expected_class', 'fault_label'):
            if forbidden in sample:
                raise ValueError('Evaluation fields cannot enter monitoring')
        if 'thrusters' in sample and sample['thrusters'] is not None:
            ids = [t['id'] for t in sample['thrusters']]
            if len(ids) != len(set(ids)):
                raise ValueError('Duplicate thruster ids')
            for thruster in sample['thrusters']:
                for key in ('command', 'rpm', 'thrust_n', 'torque_nm'):
                    value = thruster.get(key)
                    if value is not None and not finite(value):
                        raise ValueError(f'Invalid thruster {key}')
                if thruster.get('command') is not None and abs(thruster['command']) > 1:
                    raise ValueError('Command must be normalized')
        for key in ('pressure_pa', 'depth_m'):
            if sample.get(key) is not None and not finite(sample[key]):
                raise ValueError(f'Invalid {key}')
    return window


def summarize(window):
    validate_window(window)
    samples = window['samples']
    required = ('pressure', 'imu', 'dvl', 'navigation', 'thruster_state')
    missing, stale = set(), set()
    for sample in samples:
        for source in required:
            age = sample.get('sources_age_s', {}).get(source)
            if age is None:
                missing.add(source)
            elif not finite(age) or age > 2/window['source_rates_hz'][source] + 0.2:
                stale.add(source)
        if sample.get('dvl_valid') is not True:
            missing.add('dvl_valid_velocity')
        if {t['id'] for t in (sample.get('thrusters') or [])} != set(THRUSTER_IDS):
            missing.add('thruster_mapping')
    if not samples:
        missing.update(required)
    thrusters = {}
    for ident in THRUSTER_IDS:
        rows = [t for s in samples for t in (s.get('thrusters') or []) if t['id'] == ident]
        thrusters[ident] = {key: stats([t.get(key) for t in rows], unit)
                           for key, unit in (('rpm', 'rpm'), ('command', 'normalized'),
                                             ('thrust_n', 'N'), ('torque_nm', 'N m'))}
        thrusters[ident]['command_steady_rpm'] = stats(
            [t['command']*400*60/(2*math.pi) for t in rows if finite(t.get('command'))],
            'rpm')
    residuals = []
    for sample in samples:
        forecast = sample.get('expected_motion')
        pose = sample.get('position_estimate_m')
        if forecast and forecast.get('valid') and pose:
            residuals.append(math.dist(pose, forecast['position_m']))
    complete = window.get('complete', False) and len(samples) >= 45
    depth = stats([s.get('depth_m') for s in samples], 'm')
    q_last = next((s.get('orientation_xyzw') for s in reversed(samples)
                   if s.get('orientation_xyzw')), None)
    q_first = next((s.get('orientation_xyzw') for s in samples
                    if s.get('orientation_xyzw')), None)
    angle_change = None
    if q_first and q_last:
        norm = math.sqrt(sum(v*v for v in q_first)*sum(v*v for v in q_last))
        if norm > 0:
            angle_change = 2*math.acos(min(1, abs(sum(a*b for a, b
                                      in zip(q_first, q_last)))/norm))
    poses = [s['position_estimate_m'] for s in samples if s.get('position_estimate_m')]
    speeds = [math.sqrt(sum(v*v for v in s['dvl_velocity_body_m_s'])) for s in samples
              if s.get('dvl_valid') and s.get('dvl_velocity_body_m_s')]
    result = {k: window[k] for k in ('schema_version', 'run_id', 'vehicle_id', 'window_id',
                                    'window', 'frame_id', 'body_frame_id')}
    result.update({
        'status': 'ok' if complete and not missing and not stale else 'insufficient_data',
        'data_quality': {'sample_count': len(samples), 'window_complete': complete,
                         'missing_sources': sorted(missing), 'stale_sources': sorted(stale)},
        'features': {'depth_m': depth, 'thrusters': thrusters,
                     'orientation_last_xyzw': q_last,
                     'orientation_change_rad': angle_change,
                     'navigation_displacement_m': math.dist(poses[0], poses[-1])
                     if len(poses) > 1 else None,
                     'dvl_speed_m_s': stats(speeds, 'm/s'),
                     'position_prediction_residual_m': stats(residuals, 'm'),
                     'communication': window.get('communication', {})},
        'observations': [], 'summary': '',
        'limitations': ['Диагноз формирует отдельный диагностический агент.',
                        'Прогноз движения отсутствует, если нет независимого эталона.',
                        'Расчётные установившиеся RPM относятся к модели 400 рад/с. '
                        'Переходные процессы оцениваются отдельно.']})
    if depth['valid']:
        result['observations'].append({'feature': 'depth_m', 'value': depth['mean'],
                                       'unit': 'm', 'description': 'Средняя глубина за окно.'})
    autopilot_samples = [s['autopilot'] for s in samples if 'autopilot' in s]
    if autopilot_samples:
        fresh_autopilot = [s for s in autopilot_samples
                           if finite(s.get('receiver_age_s')) and s['receiver_age_s'] < 0.5]
        result['features']['autopilot'] = {
            'sample_count': len(autopilot_samples),
            'fresh_sample_count': len(fresh_autopilot),
            'heartbeat_seen': any(s.get('connected') for s in fresh_autopilot),
            'armed_seen': any(s.get('armed') for s in fresh_autopilot),
            'message_types': sorted({k for s in fresh_autopilot for k in s.get('messages', {})}),
            'rpm_source': 'stonefish_thruster_state',
            'command_source': 'ardusub_sitl_json_servo_packet'}
    result['summary'] = (f"Обработано {len(samples)} записей. "
                         + (f"Средняя глубина {depth['mean']:.2f} м. " if depth['valid'] else '')
                         + ('Обязательные источники доступны.' if result['status'] == 'ok'
                            else 'Данных недостаточно: проверить data_quality.'))
    return result


def main():
    import argparse
    from .core import decode

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    write_json(args.output, summarize(decode(args.input.read_text(encoding='utf-8'))))


if __name__ == '__main__':
    main()
