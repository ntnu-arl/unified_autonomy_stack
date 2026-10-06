"""Evidence review integrity, missing grades and report artifact tests."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
import review_agentic_benchmark as review  # noqa: E402
import plot_agentic_benchmark as plots  # noqa: E402
import summarize_agentic_benchmark as summary  # noqa: E402


class ReportingTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.campaign = Path(self.tmp.name)
        self.folder = self.campaign / 'trial'
        self.folder.mkdir()
        for name, value in {
            'trial.json': {'model': 'test', 'method': 'graph_sample', 'world_id': 'office', 'task_id': 'search', 'task': {'prompt': '<Find chair>'}},
            'outcome.json': {'outcome': 'model_completed'},
            'metrics.json': {'model_declared_complete': True, 'elapsed_sim_sec': 5, 'path_length_m': 2},
        }.items():
            (self.folder / name).write_text(json.dumps(value))
        (self.folder / 'iterations.jsonl').write_text('')
        (self.folder / 'events.jsonl').write_text('')

    def test_review_unknown_is_not_failure_and_hash_prevents_stale_grade(self):
        review.save_review(self.folder, 'unknown', 'tester', 'Cannot identify object')
        row = summary.summarize_trial(self.folder, self.campaign)
        self.assertEqual(row['semantic_task_success'], 'unknown')
        with self.assertRaises(FileExistsError):
            review.save_review(self.folder, 'success', 'tester', 'Image 1')
        review.save_review(self.folder, 'failure', 'tester', 'Image 1 wrong object', 'incorrect', True)
        row = summary.summarize_trial(self.folder, self.campaign)
        self.assertTrue(row['false_completion'])
        self.assertFalse(row['semantic_task_success'])
        with (self.folder / 'trial.json').open('a') as stream:
            stream.write('\n')
        self.assertEqual(summary.summarize_trial(self.folder, self.campaign)['semantic_task_success'], 'unknown')

    def test_reject_inconsistent_and_unsubstantiated_reviews(self):
        with self.assertRaises(ValueError):
            review.save_review(self.folder, 'success', 'tester', 'frame 1', 'incorrect')
        with self.assertRaises(ValueError):
            review.save_review(self.folder, 'success', ' ', 'frame 1')
        (self.folder / 'metrics.json').write_text('{}')
        (self.folder / 'outcome.json').write_text('{"outcome":"time_budget"}')
        with self.assertRaises(ValueError):
            review.save_review(self.folder, 'failure', 'tester', 'frame 1', 'incorrect')

    def test_packet_escapes_content_and_defaults_to_partial_blinding(self):
        path = review.packet(self.folder, self.campaign / 'packet.html')
        content = path.read_text()
        self.assertIn('&lt;Find chair&gt;', content)
        self.assertNotIn('graph_sample', content)
        self.assertIn('only partial blinding', content)
        self.assertIn('has not been run', content)

    def test_visibility_remains_separate_from_reviewed_success(self):
        trial = json.loads((self.folder / 'trial.json').read_text())
        trial['task']['stages'] = ['chair', 'table']
        (self.folder / 'trial.json').write_text(json.dumps(trial))
        (self.folder / 'visibility.json').write_text(json.dumps({
            'status': 'evaluated', 'ordered_stages_completed': 1,
            'targets': {'chair': {'status': 'visible', 'first_visible_elapsed_sec': 2.5},
                        'table': {'status': 'unknown'}}}))
        row = summary.summarize_trial(self.folder, self.campaign)
        self.assertEqual(row['visibility_targets_observed'], 1)
        self.assertEqual(row['visibility_targets_unknown'], 1)
        self.assertEqual(row['visibility_first_visible_elapsed_sec'], 2.5)
        self.assertEqual(row['visibility_ordered_stages_total'], 2)
        self.assertEqual(row['semantic_task_success'], 'unknown')
        self.assertIsNone(row['false_completion'])

    def test_wilson_small_sample_and_missing(self):
        lower, upper = plots.wilson(1, 1)
        self.assertAlmostEqual(lower, .206549, places=5)
        self.assertAlmostEqual(upper, 1)
        self.assertTrue(__import__('math').isnan(plots.wilson(0, 0)[0]))

    def test_wilson_bounds_contain_observed_rate(self):
        for count in range(1, 101):
            for successes in range(count + 1):
                low, high = plots.wilson(successes, count)
                rate = successes / count
                self.assertLessEqual(low, rate)
                self.assertGreaterEqual(high, rate)
                self.assertGreaterEqual(low, 0)
                self.assertLessEqual(high, 1)
        self.assertEqual(plots.wilson(0, 3)[0], 0)
        self.assertEqual(plots.wilson(3, 3)[1], 1)

    @unittest.skipUnless(importlib.util.find_spec('matplotlib'), 'matplotlib optional report dependency')
    def test_measurement_bars_show_mean_median_sd_and_samples(self):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axis = plt.subplots()
        self.addCleanup(lambda: plt.close(fig))
        plots.plot_measurements(axis, 0, [1., 2., 9.])
        self.assertAlmostEqual(axis.patches[0].get_height(), 4.)
        self.assertEqual(len(axis.patches), 1)
        median_segment = axis.collections[1].get_segments()[0]
        self.assertEqual(median_segment[:, 1].tolist(), [2., 2.])
        self.assertEqual(len(axis.collections[-1].get_offsets()), 3)
        segments = axis.collections[0].get_segments()[0]
        self.assertAlmostEqual(segments[1, 1] - 4., __import__('statistics').stdev([1., 2., 9.]))
        plots.plot_measurements(axis, 1, [7.])
        self.assertEqual(axis.patches[-1].get_height(), 7.)

    @unittest.skipUnless(importlib.util.find_spec('matplotlib'), 'matplotlib optional report dependency')
    def test_report_outputs_unknown_grades_and_real_figures(self):
        paths = plots.create_report(self.campaign, self.campaign / 'report')
        self.assertEqual(len(paths), 5)
        self.assertTrue(all(path.stat().st_size > 100 for path in paths))
        self.assertIn('unknown', paths[-1].read_text())
        self.assertEqual(paths[0].read_bytes()[:8], b'\x89PNG\r\n\x1a\n')


if __name__ == '__main__':
    unittest.main()
