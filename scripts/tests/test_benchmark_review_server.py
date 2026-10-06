"""Exercise browser review persistence through the actual HTTP application."""
import http.client
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
import review_agentic_benchmark as review  # noqa: E402
import summarize_agentic_benchmark as summary  # noqa: E402


class ReviewServerTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.campaign = Path(self.temp.name)
        self.folder = self.campaign / 'trial'
        self.folder.mkdir()
        (self.folder / 'trial.json').write_text(json.dumps({'model': 'test', 'method': 'graph_sample',
            'world_id': 'office', 'task_id': 'search', 'task': {'prompt': 'Find chair'}}))
        (self.folder / 'outcome.json').write_text('{"outcome":"model_completed"}')
        (self.folder / 'metrics.json').write_text('{"model_declared_complete":true}')
        review.prepare(self.campaign)
        self.server = review.ReviewServer(self.campaign, 0)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.close_server)
        self.code = review.trial_digest(self.folder)[:12]
        self.path = '/api/reviews/' + self.code

    def close_server(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def request(self, method, path, value=None, token=True):
        connection = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
        headers = {'Content-Type': 'application/json'}
        if token:
            headers['X-Review-Token'] = self.server.token
        connection.request(method, path, json.dumps(value) if value is not None else None, headers)
        response = connection.getresponse()
        payload = response.read()
        status = response.status
        connection.close()
        return status, payload

    def value(self, **changes):
        return dict({'trial_sha256': review.trial_digest(self.folder), 'reviewer': 'Reviewer A',
                     'result': 'success', 'evidence': 'Iteration 000003/image_000.jpg shows the chair and the applied finish identifies it.',
                     'completion_claim': 'correct', 'overwrite': False}, **changes)

    def test_browser_save_reloads_and_is_used_in_summary(self):
        status, page = self.request('GET', '/' + self.code + '.html')
        self.assertEqual(status, 200)
        self.assertIn(b'id="review-form"', page)
        status, state = self.request('GET', self.path)
        self.assertEqual(status, 200)
        self.assertTrue(json.loads(state)['declared_complete'])
        self.assertEqual(self.request('POST', self.path, self.value())[0], 200)
        self.assertTrue(summary.summarize_trial(self.folder, self.campaign)['semantic_task_success'])
        self.assertEqual(self.request('POST', self.path, self.value())[0], 409)
        self.assertEqual(self.request('POST', self.path, self.value(result='failure', completion_claim='incorrect', overwrite=True))[0], 200)
        self.assertTrue(json.loads(self.request('GET', self.path)[1])['review']['false_completion'])

    def test_reject_bad_session_stale_trial_invalid_fields_and_paths(self):
        self.assertEqual(self.request('POST', self.path, self.value(), token=False)[0], 403)
        self.assertEqual(self.request('POST', self.path, self.value(trial_sha256='old'))[0], 409)
        self.assertEqual(self.request('POST', self.path, self.value(reviewer={}))[0], 400)
        self.assertEqual(self.request('POST', self.path, self.value(result='failure'))[0], 400)
        self.assertEqual(self.request('GET', '/../trial/trial.json')[0], 404)
        self.assertFalse((self.folder / 'review.json').exists())

    def test_no_declaration_cannot_receive_correct_or_incorrect_grade(self):
        (self.folder / 'metrics.json').write_text('{}')
        (self.folder / 'outcome.json').write_text('{"outcome":"time_budget"}')
        self.assertEqual(self.request('POST', self.path, self.value())[0], 400)
        self.assertEqual(self.request('POST', self.path, self.value(result='unknown', completion_claim='unknown'))[0], 200)
        self.assertIsNone(json.loads((self.folder / 'review.json').read_text())['semantic_task_success'])

    def test_embedded_existing_review_cannot_inject_script(self):
        review.save_review(self.folder, 'unknown', 'Reviewer A', '</script><script>alert(1)</script>')
        packet = review.packet(self.folder, self.campaign / 'packet.html').read_text()
        self.assertNotIn('</script><script>alert(1)</script>', packet)
        self.assertIn('\\u003c/script', packet)


if __name__ == '__main__':
    unittest.main()
