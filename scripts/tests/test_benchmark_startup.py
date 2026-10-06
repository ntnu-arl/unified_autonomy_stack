"""Startup supervision must expose dead services instead of waiting for ROS timeouts."""
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import MagicMock, patch

SPEC = importlib.util.spec_from_file_location('benchmark_startup', Path(__file__).parents[1] / 'run_agentic_benchmark.py')
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)


class StartupTests(unittest.TestCase):
    def test_parameter_loader_is_only_successful_exit_allowed(self):
        states = [{'Service': 'ros1_launch_bridge_params', 'State': 'exited', 'ExitCode': 0},
                  {'Service': 'evaluation', 'State': 'exited', 'ExitCode': 0},
                  {'Service': 'ros2_launch_uav_sim', 'State': 'running', 'Health': 'starting'}]
        self.assertIsNone(runner.failed_service(states))
        states.append({'Service': 'ros1_launch_gbplanner', 'State': 'exited', 'ExitCode': 0})
        self.assertEqual(runner.failed_service(states)['Service'], 'ros1_launch_gbplanner')
        self.assertIsNotNone(runner.failed_service([{'Service': 'ros1_launch_bridge_params', 'State': 'exited', 'ExitCode': 1}]))
        self.assertIsNotNone(runner.failed_service([{'Service': 'sim', 'State': 'running', 'Health': 'unhealthy'}]))

    def test_dead_planner_logs_are_printed_and_wait_process_is_reaped(self):
        state = {'Service': 'ros1_launch_gbplanner', 'State': 'exited', 'ExitCode': 1}
        process = MagicMock()
        process.poll.return_value = None
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(runner.subprocess, 'Popen', return_value=process), \
                patch.object(runner.time, 'monotonic', side_effect=[0, 6]), \
                patch.object(runner, 'command', side_effect=[subprocess.CompletedProcess([], 0, json.dumps(state)), subprocess.CompletedProcess([], 0, 'Missing voxblox_config.yaml', '')]), \
                patch('builtins.print') as output:
            with self.assertRaisesRegex(RuntimeError, 'ros1_launch_gbplanner failed'):
                runner.monitored_command(['docker', 'wait', 'container'], ['docker', 'compose'], {}, Path(folder), 100)
        self.assertIn('Missing voxblox_config.yaml', output.call_args.args[0])
        process.terminate.assert_called_once()
        process.wait.assert_called_once()


if __name__ == '__main__':
    unittest.main()
