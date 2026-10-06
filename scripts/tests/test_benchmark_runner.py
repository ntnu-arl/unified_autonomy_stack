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
            profile = copy.deepcopy(runner.profile)
            profile.update(model=runner.model, method={'type': trial['method']},
                           max_model_iterations=trial['max_model_iterations'])
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


if __name__ == '__main__':
    unittest.main()
