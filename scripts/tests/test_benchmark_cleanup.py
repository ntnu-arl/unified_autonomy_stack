"""Benchmark cleanup must preserve unrelated stacks and live benchmark ownership."""
import fcntl
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).parents[1] / 'run_agentic_benchmark.py'
spec = importlib.util.spec_from_file_location('benchmark_cleanup', SCRIPT)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class CleanupTests(unittest.TestCase):
    def labels(self, service='ros1_launch_roscore'):
        return {'com.docker.compose.project': 'agentic-eval-123-0',
                'com.docker.compose.service': service,
                runner.OWNER_LABEL: str(runner.ROOT)}

    def test_scope_and_legacy_deleted_directory(self):
        self.assertTrue(runner.owned_benchmark(self.labels()))
        self.assertFalse(runner.owned_benchmark(dict(self.labels(), **{runner.OWNER_LABEL: '/some/other/checkout'})))
        self.assertFalse(runner.owned_benchmark({'com.docker.compose.project': 'normal-stack'}))
        legacy = self.labels()
        del legacy[runner.OWNER_LABEL]
        legacy['com.docker.compose.project.working_dir'] = str(runner.ROOT / 'evaluation_results/deleted-trial')
        self.assertTrue(runner.owned_benchmark(legacy))
        legacy['com.docker.compose.project.working_dir'] = '/unrelated/project'
        self.assertFalse(runner.owned_benchmark(legacy))

    def test_recorder_stops_first_without_touching_unrelated_containers(self):
        metadata = [
            {'Id': 'ros', 'Config': {'Labels': self.labels()}, 'State': {'Running': True}},
            {'Id': 'bag', 'Config': {'Labels': self.labels('evaluation')}, 'State': {'Running': True}},
            {'Id': 'other', 'Config': {'Labels': {'com.docker.compose.project': 'another-stack'}}, 'State': {'Running': True}},
        ]
        responses = [subprocess.CompletedProcess([], 0, 'ros\nbag\nother\n'),
                     subprocess.CompletedProcess([], 0, json.dumps(metadata))]
        responses.extend(subprocess.CompletedProcess([], 0, '') for _ in range(3))
        with patch.object(runner, 'command', side_effect=responses) as command:
            runner.cleanup_benchmarks()
        calls = [call.args[0] for call in command.call_args_list]
        self.assertEqual(calls[2][-1], 'bag')
        self.assertEqual(calls[3][-1], 'ros')
        self.assertEqual(calls[4], ['docker', 'rm', 'ros', 'bag'])

    def test_active_runner_blocks_cleanup_before_docker(self):
        with open('/tmp/agentic-benchmark.lock', 'a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                pass  # An actual active benchmark already owns the lock.
            result = subprocess.run([sys.executable, str(SCRIPT), '--stop-existing'], text=True, capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('runner is still active', result.stderr)


if __name__ == '__main__':
    unittest.main()
