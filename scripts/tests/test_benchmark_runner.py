"""Verify trial pairing and isolation without launching Docker or calling a model."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import copy
import yaml
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('benchmark_runner', Path(__file__).parents[1] / 'run_agentic_benchmark.py')
runner_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner_module)


class RunnerTest(unittest.TestCase):
    def test_initialization_yaw_scan_config(self):
        runner = self.runner()
        self.assertFalse(runner.pci_config()['init_motion']['yaw_scan_enable'])
        runner.manifest['runtime'].update(initialization_yaw_scan=True, initialization_yaw_rate=0.4)
        self.assertEqual(runner.pci_config()['init_motion']['yaw_scan_rate'], 0.4)
        self.assertTrue(runner.pci_config()['init_motion']['yaw_scan_enable'])
        for rate in (0, -1, float('nan'), True):
            runner.manifest['runtime']['initialization_yaw_rate'] = rate
            with self.assertRaises(ValueError):
                runner.pci_config()

    def test_agent_and_planner_share_bounds_for_every_method(self):
        runner = self.runner(repetitions=1)
        for trial in runner.trials():
            with self.subTest(world=trial['world_id'], method=trial['method']):
                profile = runner.agent_config(trial)
                expected = trial['world']['gbplanner_bounds']
                self.assertEqual(profile['goal_bounds'], dict(enabled=True, **expected))
                planner = runner.planner_config(trial['world'])['BoundedSpaceParams']['Global']
                self.assertEqual(profile['goal_bounds']['min'], planner['min_val'])
                self.assertEqual(profile['goal_bounds']['max'], planner['max_val'])
        self.assertFalse(runner.profile['goal_bounds']['enabled'])

    def test_world_planner_bounds_override_only_global_space(self):
        runner = self.runner()
        source = runner.planner_config({})
        world = runner.manifest['worlds']['office']
        config = runner.planner_config(world)
        self.assertEqual(config['BoundedSpaceParams']['Global']['min_val'], world['gbplanner_bounds']['min'])
        self.assertEqual(config['BoundedSpaceParams']['Global']['max_val'], world['gbplanner_bounds']['max'])
        self.assertEqual(config['BoundedSpaceParams']['Global']['type'], 'kCuboid')
        config['BoundedSpaceParams']['Global'] = source['BoundedSpaceParams']['Global']
        self.assertEqual(config, source)
        self.assertEqual(runner.planner_config({}), source)

    def test_invalid_planner_bounds_fail_before_launch(self):
        runner = self.runner()
        for bounds in ({'min': [0, 0, 0]}, {'min': [0, 0], 'max': [1, 1, 1]},
                       {'min': [0, 0, 0], 'max': [1, float('nan'), 1]},
                       {'min': [0, 0, 0], 'max': [1, 0, 1]},
                       {'min': [False, 0, 0], 'max': [1, 1, 1]}):
            with self.subTest(bounds=bounds), self.assertRaisesRegex(ValueError, 'gbplanner_bounds'):
                runner.planner_config({'gbplanner_bounds': bounds})
        runner.manifest['worlds']['office']['gbplanner_bounds']['max'][0] = -100
        with self.assertRaisesRegex(ValueError, 'gbplanner_bounds'):
            runner.trials()

    def runner(self, **overrides):
        settings = dict(manifest=runner_module.MANIFEST, model=None, output=Path('/tmp/eval-tests'),
                        campaign='test', domain=218, master_port=11321, worlds=None,
                        methods=None, repetitions=None, tasks=None, duration=None,
                        max_runs=None, max_model_iterations=None, seed=42, rviz=False)
        settings.update(overrides)
        return runner_module.BenchmarkRunner(argparse.Namespace(**settings))

    def test_matrix_pairs_world_task_seed_across_methods(self):
        trials = self.runner().trials()
        self.assertEqual(len(trials), 162)
        combinations = {(t['world_id'], t['task_id'], t['method'], t['seed']) for t in trials}
        self.assertEqual(len(combinations), 162)
        self.assertTrue(all(t['simulation_only'] for t in trials))
        self.assertEqual({t['seed'] for t in trials}, {42, 43, 44})

    def test_unknown_method_rejected_and_pilot_budget_explicit(self):
        with self.assertRaises(ValueError):
            self.runner(methods=['not-a-method']).trials()
        trials = self.runner(worlds=['office'], tasks=['search'], methods=['frontier'],
                             repetitions=1, duration=17).trials()
        self.assertEqual(len(trials), 1)
        self.assertEqual(trials[0]['task']['duration_sec'], 17)

    def test_model_iteration_limit(self):
        self.assertEqual(self.runner().trials()[0]['max_model_iterations'], 0)
        self.assertEqual(self.runner(max_model_iterations=5).trials()[0]['max_model_iterations'], 5)
        with self.assertRaises(ValueError):
            self.runner(max_model_iterations=-1)

    def test_resume_skips_completed_and_archives_failed(self):
        with tempfile.TemporaryDirectory() as directory:
            runner = self.runner(output=Path(directory), worlds=['office'], tasks=['search'], repetitions=1)
            trial = runner.trials()[0]
            folder = runner.trial_folder(trial)
            folder.mkdir(parents=True)
            (folder / 'trial.json').write_text(json.dumps(trial))
            profile = runner.agent_config(trial)
            (folder / 'agent.yaml').write_text(yaml.safe_dump(profile))
            outcome_path = folder / 'runner_outcome.json'
            outcome_path.write_text(json.dumps({'outcome': 'iteration_budget', 'container_exit_code': 0}))
            self.assertIsNotNone(runner.resume_outcome(trial))
            changed = copy.deepcopy(trial)
            changed['task']['duration_sec'] += 1
            with self.assertRaises(ValueError):
                runner.resume_outcome(changed)
            outcome_path.write_text(json.dumps({'outcome': 'storage_limit', 'container_exit_code': 1}))
            self.assertIsNone(runner.resume_outcome(trial))
            (folder / 'partial.bag').write_text('preserve me')
            runner.archive_partial(trial)
            self.assertFalse(folder.exists())
            archives = list((Path(directory) / '.interrupted').rglob('partial.bag'))
            self.assertEqual(len(archives), 1)
            self.assertEqual(archives[0].read_text(), 'preserve me')

    def test_compose_isolated_without_archiving_credentials(self):
        runner = self.runner(worlds=['office'], tasks=['search'], repetitions=1)
        services = {}
        for name in ('ros1_launch_roscore', 'ros1_launch_gbplanner', 'ros2_launch_uav_sim',
                     'ros2_launch_agentic_uas', 'ros2_launch_scene_graph'):
            services[name] = {'image': 'test-image', 'profiles': ['launch'],
                              'command': ['bash', '-c', 'exec roslaunch planner.launch'],
                              'environment': {'OPENAI_API_KEY': 'secret-sentinel'},
                              'volumes': [{'type': 'bind', 'source': '/tmp/ws', 'target': '/workspace'}]}
        services['disabled'] = {'profiles': ['cbf']}
        original = {'services': services, 'name': 'normal-user-stack'}
        with patch.object(runner_module, 'command', return_value=subprocess.CompletedProcess([], 0, json.dumps(original))):
            config = runner.compose_config(runner.trials()[0], Path('/tmp/trial'), 'isolated-test')
            runner.args.rviz = True
            visible_config = runner.compose_config(runner.trials()[0], Path('/tmp/trial'), 'visible-test')
        self.assertTrue(visible_config['services']['ros1_launch_gbplanner']['command'][-1].endswith(' rviz:=true'))
        self.assertNotIn('secret-sentinel', json.dumps(config))
        self.assertNotIn('disabled', config['services'])
        self.assertEqual(config['services']['ros2_launch_scene_graph']['environment']['ROS_DOMAIN_ID'], '219')
        self.assertEqual(config['services']['ros2_launch_agentic_uas']['environment']['ROS_DOMAIN_ID'], '218')
        self.assertIn('world:=rmf_office', config['services']['ros2_launch_uav_sim']['command'])
        self.assertEqual(config['services']['ros1_launch_roscore']['command'], ['roscore', '-p', '11321'])
        self.assertTrue(config['services']['ros1_launch_gbplanner']['command'][-1].endswith(' rviz:=false'))
        self.assertIn('gbplanner_config_file:=/evaluation/run/gbplanner.yaml',
                      config['services']['ros1_launch_gbplanner']['command'][-1])
        self.assertIn({'type': 'bind', 'source': '/tmp/trial', 'target': '/evaluation/run', 'read_only': True},
                      config['services']['ros1_launch_gbplanner']['volumes'])


if __name__ == '__main__':
    unittest.main()
