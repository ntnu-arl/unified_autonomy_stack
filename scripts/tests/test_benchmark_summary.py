"""Synthetic evidence checks for offline benchmark reports; no ROS is required."""

import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    "benchmark_summary", Path(__file__).resolve().parents[1] / "summarize_agentic_benchmark.py")
summary = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(summary)


class BenchmarkSummaryTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.campaign = Path(self.temporary.name)

    def trial(self, name="first", outcome="model_completed", distance=10):
        folder = self.campaign / name
        folder.mkdir()
        (folder / "trial.json").write_text(json.dumps({
            "model": "test-model", "world_id": "office", "task_id": "search",
            "method": "graph_sample", "seed": 42, "repetition": 0}))
        (folder / "outcome.json").write_text(json.dumps({"outcome": outcome}))
        (folder / "metrics.json").write_text(json.dumps({
            "path_length_m": distance, "elapsed_sim_sec": 10,
            "model_declared_complete": outcome == "model_completed",
            "semantic_task_success": None, "ordered_stages_completed": 1,
            "target_region_visit_times_sec": {"target": 4}}))
        (folder / "iterations.jsonl").write_text("")
        (folder / "events.jsonl").write_text("")
        return folder

    def write_records(self, folder, name, records):
        (folder / name).write_text("".join(json.dumps(record) + "\n" for record in records))

    def test_iteration_budget_is_valid_without_completion_claim(self):
        folder = self.trial("limited", "iteration_budget")
        row = summary.summarize_trial(folder, self.campaign)
        self.assertTrue(row["valid_performance_run"])
        self.assertEqual(row["semantic_task_success"], "unknown")

    def test_counts_unique_calls_including_validation_errors_and_discarded_prefetch(self):
        folder = self.trial()
        records = [
            {"kind": "started", "iteration_id": 1, "monotonic_ns": 1_000_000_000,
             "context": {"prefetch": True}},
            {"kind": "request", "iteration_id": 1},
            {"kind": "response", "iteration_id": 1, "monotonic_ns": 3_000_000_000,
             "usage": {"input_tokens": 100, "output_tokens": 20, "total_tokens": 120}},
            {"kind": "error", "iteration_id": 1, "error": "validation failed"},
            {"kind": "error", "iteration_id": 1, "error": "duplicate delivery"},
            {"kind": "started", "iteration_id": 2, "monotonic_ns": 4_000_000_000,
             "context": {"prefetch": True}},
            {"kind": "request", "iteration_id": 2},
            {"kind": "response", "iteration_id": 2, "monotonic_ns": 8_000_000_000,
             "usage": {"input_tokens": 60, "output_tokens": 40, "total_tokens": 100}},
            {"kind": "validated", "iteration_id": 2},
            {"kind": "request", "iteration_id": 3},
        ]
        records.append(records[2])
        self.write_records(folder, "iterations.jsonl", records)
        self.write_records(folder, "events.jsonl", [
            {"kind": "discarded", "iteration_id": 1},
            {"kind": "discarded", "iteration_id": 1},
            {"kind": "applied", "iteration_id": 2},
            {"kind": "state", "iteration_id": None}])
        row = summary.summarize_trial(folder, self.campaign)
        self.assertEqual(row["model_calls"], 3)
        self.assertEqual(row["model_responses"], 2)
        self.assertEqual(row["model_errors"], 1)
        self.assertEqual(row["model_calls_incomplete"], 1)
        self.assertEqual(row["model_latency_mean_sec"], 3)
        self.assertEqual(row["input_tokens"], 160)
        self.assertEqual(row["total_tokens"], 220)
        self.assertEqual(row["prefetch_applied"], 1)
        self.assertEqual(row["prefetch_discarded"], 1)
        self.assertEqual(row["semantic_task_success"], "unknown")

    def test_aggregates_exclude_failed_partial_performance_and_keep_denominators(self):
        first = self.trial("first", distance=10)
        second = self.trial("second", "time_budget", distance=20)
        failed = self.trial("failure", "runtime_error", distance=999)
        rows = [summary.summarize_trial(folder, self.campaign) for folder in (first, second, failed)]
        aggregate = summary.aggregate_trials(rows)[0]
        self.assertEqual(aggregate["runs"], 3)
        self.assertEqual(aggregate["infrastructure_failures"], 1)
        self.assertEqual(aggregate["valid_performance_runs"], 2)
        self.assertEqual(aggregate["path_length_m_mean"], 15)
        self.assertAlmostEqual(aggregate["path_length_m_std"], 7.0710678118654755)
        self.assertEqual(aggregate["path_length_m_n"], 2)
        self.assertEqual(aggregate["model_calls_n"], 3)
        self.assertEqual(aggregate["semantic_success_known_runs"], 0)

    def test_missing_and_truncated_logs_are_explicit_not_zero_success(self):
        folder = self.trial()
        (folder / "events.jsonl").unlink()
        (folder / "iterations.jsonl").write_text('{"kind":"request","iteration_id":1}\n{"kind":')
        row = summary.summarize_trial(folder, self.campaign)
        self.assertEqual(row["model_calls"], 1)
        self.assertIsNone(row["prefetch_discarded"])
        self.assertEqual(row["evidence_issue_count"], 2)
        (folder / "iterations.jsonl").unlink()
        row = summary.summarize_trial(folder, self.campaign)
        self.assertIsNone(row["model_calls"])

    def test_runner_failure_overrides_nominal_evaluator_completion(self):
        folder = self.trial()
        (folder / "runner_outcome.json").write_text(json.dumps({
            "outcome": "model_completed", "container_exit_code": 1}))
        row = summary.summarize_trial(folder, self.campaign)
        self.assertTrue(row["infrastructure_failure"])
        self.assertFalse(row["valid_performance_run"])

    def test_unknown_token_usage_and_bad_clocks_are_not_fabricated(self):
        metrics = summary.decision_metrics([
            {"kind": "started", "iteration_id": 1, "monotonic_ns": 99},
            {"kind": "request", "iteration_id": 1},
            {"kind": "response", "iteration_id": 1, "monotonic_ns": 2, "usage": None}], [])
        self.assertIsNone(metrics["model_latency_mean_sec"])
        self.assertIsNone(metrics["input_tokens"])
        self.assertEqual(metrics["usage_responses"], 0)

    def test_independent_review_and_fixture_invalidation(self):
        folder = self.trial()
        (folder / "review.json").write_text(json.dumps({
            "semantic_task_success": True, "reviewer": "independent", "evidence": "frame 4"}))
        row = summary.summarize_trial(folder, self.campaign)
        aggregate = summary.aggregate_trials([row])[0]
        self.assertEqual(aggregate["semantic_success_known_runs"], 1)
        self.assertEqual(aggregate["semantic_success_rate"], 1)
        (folder / "validation.json").write_text(json.dumps({"valid_trial": False, "reason": "invalid start"}))
        row = summary.summarize_trial(folder, self.campaign)
        self.assertFalse(row["valid_performance_run"])
        self.assertTrue(row["infrastructure_failure"])
        self.assertEqual(summary.aggregate_trials([row])[0]["semantic_success_known_runs"], 0)

    def test_offline_metrics_replace_live_values(self):
        folder = self.trial()
        (folder / "replay_metrics.json").write_text(json.dumps({"path_length_m": 8, "replay": {"failures": {}}}))
        row = summary.summarize_trial(folder, self.campaign)
        self.assertEqual(row["path_length_m"], 8)
        self.assertEqual(row["metric_source"], "replay")

    def test_atomic_csv_reports_and_group_separation(self):
        self.trial()
        other = self.trial("other")
        trial = json.loads((other / "trial.json").read_text())
        trial["method"] = "image_bearing"
        (other / "trial.json").write_text(json.dumps(trial))
        per_trial, aggregate = summary.summarize_campaign(self.campaign)
        with per_trial.open() as stream:
            rows = list(csv.DictReader(stream))
        with aggregate.open() as stream:
            groups = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 2)
        self.assertEqual(len(groups), 2)
        self.assertTrue(all(row["semantic_task_success"] == "unknown" for row in rows))
        original = per_trial.read_bytes()
        with patch.object(summary.os, "replace", side_effect=OSError("interrupted")):
            with self.assertRaises(OSError):
                summary.write_csv_atomic(per_trial, [{"trial_id": "changed"}])
        self.assertEqual(per_trial.read_bytes(), original)
        self.assertEqual(list(self.campaign.glob(".*.tmp")), [])

    def test_empty_campaign_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "No trial.json"):
            summary.summarize_campaign(self.campaign)


if __name__ == "__main__":
    unittest.main()
