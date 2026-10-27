import copy
import math
import unittest

from auv_diagnostics.monitoring import THRUSTER_IDS, ned_to_enu, orientation_to_enu, summarize
from auv_diagnostics.network import Channel, Observer, replay


def sample_window():
    return {'schema_version': '1.0', 'run_id': 'r1', 'vehicle_id': 'auv_01',
            'window_id': 'w0', 'frame_id': 'map_enu', 'body_frame_id': 'body_frd',
            'window': {'start_s': 0.0, 'end_s': 5.0, 'clock_domain': 'ros_wall'},
            'source_rates_hz': {k: 10 for k in ('pressure', 'imu', 'dvl', 'navigation',
                                               'thruster_state')}, 'complete': True,
            'samples': [{'stamp_s': i/10, 'depth_m': 5.0, 'dvl_valid': True,
                         'sources_age_s': {k: 0.1 for k in ('pressure', 'imu', 'dvl',
                                                           'navigation', 'thruster_state')},
                         'thrusters': [{'id': ident, 'command': 0.0, 'rpm': 0.0,
                                        'thrust_n': 0.0, 'torque_nm': 0.0}
                                       for ident in THRUSTER_IDS],
                         'expected_motion': None} for i in range(50)]}


class MonitoringTests(unittest.TestCase):
    def test_world_transform_and_identity_orientation(self):
        self.assertEqual(ned_to_enu([10, 20, 5]), [20, 10, -5])
        q = orientation_to_enu([0, 0, 0, 1])
        self.assertAlmostEqual(sum(x*x for x in q), 1)
        self.assertAlmostEqual(q[0], math.sqrt(0.5))
        self.assertAlmostEqual(q[1], math.sqrt(0.5))
        with self.assertRaises(ValueError):
            orientation_to_enu([0, 0, 0, 0])

    def test_missing_measurements_and_no_fabricated_forecast(self):
        data = sample_window()
        data['samples'] = []
        summary = summarize(data)
        self.assertEqual(summary['status'], 'insufficient_data')
        self.assertIsNone(summary['features']['depth_m']['mean'])
        self.assertFalse(summary['features']['position_prediction_residual_m']['valid'])

    def test_partial_dvl_is_not_a_zero_speed(self):
        data = sample_window()
        data['samples'][4]['dvl_valid'] = False
        self.assertIn('dvl_valid_velocity', summarize(data)['data_quality']['missing_sources'])

    def test_stale_measurement(self):
        data = sample_window()
        data['samples'][2]['sources_age_s']['imu'] = 3
        self.assertIn('imu', summarize(data)['data_quality']['stale_sources'])

    def test_signed_rpm_and_command_scaling(self):
        data = sample_window()
        for s in data['samples']:
            s['thrusters'] = [{'id': 'horizontal_left', 'command': -0.5, 'rpm': -1900.0,
                              'thrust_n': -10.0, 'torque_nm': 0.0}]
        result = summarize(data)['features']['thrusters']['horizontal_left']
        self.assertEqual(result['rpm']['mean'], -1900)
        self.assertAlmostEqual(result['command_steady_rpm']['mean'], -200*60/(2*math.pi))
        data['samples'][0]['thrusters'][0]['command'] = 200
        with self.assertRaises(ValueError):
            summarize(data)

    def test_labels_and_invalid_time_rejected(self):
        data = sample_window()
        data['expected_class'] = 'none'
        with self.assertRaises(ValueError):
            summarize(data)
        data.pop('expected_class')
        data['samples'][1]['stamp_s'] = 0
        with self.assertRaises(ValueError):
            summarize(data)

    def test_repeatable_numeric_summary(self):
        data = sample_window()
        self.assertEqual(summarize(data), summarize(copy.deepcopy(data)))
        self.assertEqual(summarize(data)['features']['depth_m']['mean'], 5)


class NetworkTests(unittest.TestCase):
    def packets(self):
        return [{'sequence': i, 'sent_s': i/10} for i in range(160)]

    def test_delay_and_loss_measured_at_receiver(self):
        result = replay(self.packets(), delay_s=0.7)
        self.assertEqual(result['received_unique_count'], 160)
        self.assertAlmostEqual(result['latency_mean_ms'], 700)
        lost = replay(self.packets(), loss=1)
        self.assertEqual(lost['received_unique_count'], 0)
        self.assertIsNone(lost['latency_mean_ms'])

    def test_node_outage_and_recovery(self):
        result = replay(self.packets(), node_outage=(4, 8))
        events = [x['event'] for x in result['observed_events']]
        self.assertEqual(events, ['node_unreachable', 'node_reachable'])
        self.assertTrue(all(x.get('reason') == 'message_timeout' for x
                            in result['observed_events'] if x['event'] == 'node_unreachable'))

    def test_link_outage_and_separate_evaluation(self):
        result = replay(self.packets(), link_outage=(4, 8))
        self.assertIn('link_disconnected', [x['event'] for x in result['evaluation_events']])
        self.assertNotIn('link_disconnected', [x['event'] for x in result['observed_events']])
        self.assertGreater(result['loss_pct'], 0)

    def test_seed_reproduces_drops_and_jitter(self):
        args = {'seed': 42, 'loss': 0.2, 'jitter_s': 0.1}
        self.assertEqual(replay(self.packets(), **args), replay(self.packets(), **args))

    def test_duplicates_do_not_restore_or_inflate_count(self):
        observer = Observer()
        observer.expect('auv_01', 0)
        item = {'sender': 'auv_01', 'received_s': 0.1,
                'packet': {'sequence': 1, 'sent_s': 0}}
        self.assertTrue(observer.accept(item))
        observer.tick(3)
        item['received_s'] = 4
        self.assertFalse(observer.accept(item))
        self.assertIn('auv_01', observer.unreachable)
        self.assertEqual(len(observer.seen), 1)

    def test_two_nodes_are_independent(self):
        channel = Channel(loss=0, delay_s=0)
        channel.set_node('auv_02', False, 0)
        for node in ('auv_01', 'auv_02'):
            channel.send(node, 'monitor', {'sequence': 0}, 0)
        self.assertEqual([x['sender'] for x in channel.receive(0)], ['auv_01'])


if __name__ == '__main__':
    unittest.main()
