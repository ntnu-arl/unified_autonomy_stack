"""Check that campaign analysis preserves missing evidence and surfaces failed replays."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('benchmark_analysis', Path(__file__).parents[1] / 'analyze_agentic_benchmark.py')
analysis = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analysis)


class AnalysisTests(unittest.TestCase):
    def test_missing_bag_reported_without_hiding_other_reports(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'trial.json').write_text('{}')
            with patch.object(sys, 'argv', ['analyze', folder]), patch.object(analysis.subprocess, 'run') as run:
                with self.assertRaisesRegex(SystemExit, '1 trials could not be replayed'):
                    analysis.main()
            self.assertEqual(len(run.call_args_list), 2)
            self.assertTrue(all(call.args[0][0] != 'docker' for call in run.call_args_list))
            self.assertEqual(json.loads((root / 'analysis_failures.json').read_text())[0]['error'], 'No closed bag metadata')

    def test_skip_replay_preserves_prior_failures(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'trial.json').write_text('{}')
            previous = [{'trial': '.', 'error': 'Previous replay failed'}]
            (root / 'analysis_failures.json').write_text(json.dumps(previous))
            with patch.object(sys, 'argv', ['analyze', folder, '--skip-replay']), patch.object(analysis.subprocess, 'run'):
                with self.assertRaises(SystemExit):
                    analysis.main()
            self.assertEqual(json.loads((root / 'analysis_failures.json').read_text()), previous)

    def test_failed_replay_still_generates_review_and_plot_outputs(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'trial.json').write_text('{}')
            (root / 'bag').mkdir()
            (root / 'bag/metadata.yaml').write_text('')
            with patch.object(sys, 'argv', ['analyze', folder]), patch.object(analysis.subprocess, 'run', return_value=subprocess.CompletedProcess([], 2)) as run:
                with self.assertRaises(SystemExit):
                    analysis.main()
            self.assertEqual(len(run.call_args_list), 3)
            self.assertEqual(json.loads((root / 'analysis_failures.json').read_text())[0]['exit_code'], 2)


if __name__ == '__main__':
    unittest.main()
