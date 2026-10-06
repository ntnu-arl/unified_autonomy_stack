#!/usr/bin/env python3
"""Summarize recorded trials without treating model completion as semantic success.

Performance means use only time_budget/model_completed/iteration_budget trials. Resource counts use
all trials with available logs. Every numeric aggregate has its own sample count;
standard deviations are sample deviations and blank for fewer than two samples.
A response followed by a validation error counts once in both response and error
columns. Token totals include only reported usage, with coverage counts alongside.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
from typing import Any

GROUP_FIELDS = ("model", "world_id", "task_id", "method")
VALID_OUTCOMES = {"time_budget", "model_completed", "iteration_budget"}
INFRASTRUCTURE_OUTCOMES = {
    "infrastructure_error", "invalid_start", "startup_error", "runtime_error", "recorder_failed", "startup_timeout",
    "storage_limit", "service_timeout", "service_rejected", "service_failed", "wall_timeout",
}
PERFORMANCE_FIELDS = (
    "elapsed_sim_sec", "task_wall_elapsed_sec", "startup_wall_sec", "path_length_m", "moving_sim_sec", "idle_fraction",
    "ordered_stages_completed", "ordered_stages_total", "all_target_regions_visited",
    "ordered_regions_completed", "observed_occupied_voxels_proxy", "voxel_size_m",
    "odom_samples", "lidar_samples", "clock_resets", "model_declared_complete",
)
RESOURCE_FIELDS = (
    "model_calls", "model_responses", "model_errors", "model_validated",
    "model_calls_incomplete", "model_latency_mean_sec", "model_latency_samples",
    "input_tokens", "output_tokens", "total_tokens", "usage_responses",
    "prefetch_started", "prefetch_applied", "prefetch_discarded", "prefetch_rejected",
)


def read_object(path: Path, issues: list[str]) -> dict[str, Any]:
    """Read a JSON object and retain missing/corrupt evidence as an explicit issue."""
    try:
        value = json.loads(path.read_text())
        if not isinstance(value, dict):
            raise ValueError("expected a JSON object")
        return value
    except (OSError, ValueError) as error:
        issues.append(f"{path.name}: {error}")
        return {}


def read_records(path: Path, issues: list[str]) -> list[dict[str, Any]] | None:
    """Read intact JSONL records, reporting damaged lines from interrupted writes."""
    try:
        lines = path.read_text().splitlines()
    except OSError as error:
        issues.append(f"{path.name}: {error}")
        return None
    records = []
    for number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError("expected a JSON object")
            records.append(value)
        except ValueError as error:
            issues.append(f"{path.name}:{number}: {error}")
    return records


def finite_number(value: Any) -> bool:
    """Accept finite metric scalars, including Boolean rates."""
    return isinstance(value, (int, float)) and math.isfinite(value)


def decision_metrics(records: list[dict[str, Any]] | None,
                     events: list[dict[str, Any]] | None) -> dict[str, Any]:
    """Count unique iterations and match monotonic start/response timestamps."""
    result: dict[str, Any] = dict.fromkeys(RESOURCE_FIELDS)
    if records is None:
        return result
    by_kind: dict[str, dict[int, dict[str, Any]]] = {}
    for record in records:
        iteration = record.get("iteration_id")
        if type(iteration) is int:
            # One logical call per iteration; repeated transports must not inflate counts.
            by_kind.setdefault(record.get("kind", ""), {}).setdefault(iteration, record)
    starts = by_kind.get("started", {})
    requests = by_kind.get("request", {})
    responses = by_kind.get("response", {})
    errors = by_kind.get("error", {})
    latency = []
    for iteration, response in responses.items():
        start = starts.get(iteration, {}).get("monotonic_ns")
        end = response.get("monotonic_ns")
        if finite_number(start) and finite_number(end) and end >= start:
            latency.append((end - start) / 1e9)
    result.update(model_calls=len(requests), model_responses=len(responses),
                  model_errors=len(errors), model_validated=len(by_kind.get("validated", {})),
                  model_calls_incomplete=len(set(requests) - set(responses) - set(errors)),
                  model_latency_mean_sec=statistics.mean(latency) if latency else None,
                  model_latency_samples=len(latency))
    usages = [record["usage"] for record in responses.values() if isinstance(record.get("usage"), dict)]
    result["usage_responses"] = len(usages)
    for field in ("input_tokens", "output_tokens", "total_tokens"):
        values = [usage[field] for usage in usages if finite_number(usage.get(field))]
        result[field] = sum(values) if values else (0 if not responses else None)
    prefetch = {iteration for iteration, record in starts.items()
                if record.get("context", {}).get("prefetch") is True}
    result["prefetch_started"] = len(prefetch)
    for kind in ("applied", "discarded", "rejected"):
        result[f"prefetch_{kind}"] = (len({record.get("iteration_id") for record in events
                                           if record.get("kind") == kind} & prefetch)
                                          if events is not None else None)
    return result


def summarize_trial(folder: Path, campaign: Path) -> dict[str, Any]:
    """Read a trial, retaining partial metrics and distinguishing infrastructure failure."""
    issues: list[str] = []
    trial = read_object(folder / "trial.json", issues)
    metrics = read_object(folder / "metrics.json", issues)
    metric_source = "live"
    if (folder / "replay_metrics.json").exists():
        replay = read_object(folder / "replay_metrics.json", issues)
        if replay:
            metrics.update(replay)
            metric_source = "replay"
            if replay.get("replay", {}).get("failures"):
                issues.append("Replay reported transform/decode failures; inspect bag_validation.json")
    outcome_path = folder / "runner_outcome.json"
    outcome = read_object(outcome_path if outcome_path.exists() else folder / "outcome.json", issues)
    status = outcome.get("outcome", "unknown")
    exit_code = outcome.get("container_exit_code")
    infrastructure_failure = status in INFRASTRUCTURE_OUTCOMES or (exit_code is not None and exit_code != 0)
    valid = status in VALID_OUTCOMES and not infrastructure_failure and bool(metrics)
    if (folder / "validation.json").exists():
        validation = read_object(folder / "validation.json", issues)
        if validation.get("valid_trial") is False:
            valid = False
            infrastructure_failure = True
            issues.append("Invalid trial: " + str(validation.get("reason", "independent validation failed")))
    row = {"trial_id": str(folder.relative_to(campaign)),
           **{field: trial.get(field) for field in GROUP_FIELDS},
           "seed": trial.get("seed"), "repetition": trial.get("repetition"),
           "outcome": status, "container_exit_code": exit_code,
           "infrastructure_failure": infrastructure_failure, "valid_performance_run": valid,
           "semantic_task_success": "unknown", "false_completion": None,
           "visibility_targets_unknown": None, "visibility_targets_total": None, "visibility_targets_observed": None,
           "visibility_first_visible_elapsed_sec": None, "visibility_ordered_stages_total": None,
           "visibility_ordered_stages_completed": None, "metric_source": metric_source}
    row.update({field: metrics.get(field) if finite_number(metrics.get(field)) else None
                for field in PERFORMANCE_FIELDS})
    for field in ("target_region_visit_times_sec", "ordered_stage_visit_times_sec"):
        row[field] = json.dumps(metrics[field], sort_keys=True) if field in metrics else None
    evaluation_path = folder / "evaluation_events.jsonl"
    if evaluation_path.exists():
        observations = read_records(evaluation_path, issues) or []
        boundaries = {event.get("event"): event for event in observations
                      if event.get("event") in {"task_started", "trial_finished", "recorder_started"}}
        for field, first, last in (("task_wall_elapsed_sec", "task_started", "trial_finished"),
                                   ("startup_wall_sec", "recorder_started", "task_started")):
            before, after = boundaries.get(first, {}), boundaries.get(last, {})
            clock = "monotonic_time" if "monotonic_time" in before and "monotonic_time" in after else "wall_time"
            if finite_number(before.get(clock)) and finite_number(after.get(clock)) and after[clock] >= before[clock]:
                row[field] = after[clock] - before[clock]
    # An independent human review can supply the semantic result after inspecting
    # the recorded evidence. Model completion is never promoted to this field.
    review_path = folder / "review.json"
    if review_path.exists():
        review = read_object(review_path, issues)
        digest = review.get("trial_sha256")
        matches = digest is None or digest == hashlib.sha256((folder / "trial.json").read_bytes()).hexdigest()
        if matches and review.get("reviewer") and review.get("evidence"):
            value = review.get("semantic_task_success")
            if type(value) is bool:
                row["semantic_task_success"] = value
            elif value is not None:
                issues.append("review.json semantic_task_success must be Boolean or null")
            if type(review.get("false_completion")) is bool and row.get("model_declared_complete"):
                row["false_completion"] = review["false_completion"]
        else:
            issues.append("review.json requires matching trial hash, reviewer and evidence")
    if (folder / "visibility.json").exists():
        visibility = read_object(folder / "visibility.json", issues)
        targets = visibility.get("targets", {})
        known = [item for item in targets.values() if item.get("status") in ("visible", "not_observed")]
        if visibility.get("status") == "evaluated":
            row["visibility_targets_total"] = len(targets)
            times = [item.get("first_visible_elapsed_sec") for item in targets.values()
                     if finite_number(item.get("first_visible_elapsed_sec"))]
            row["visibility_first_visible_elapsed_sec"] = min(times) if times else None
            row["visibility_ordered_stages_total"] = len(trial.get("task", {}).get("stages", []))
            row["visibility_targets_observed"] = sum(item.get("status") == "visible" for item in known)
            row["visibility_ordered_stages_completed"] = visibility.get("ordered_stages_completed")
        row["visibility_targets_unknown"] = len(targets) - len(known)
    records = read_records(folder / "iterations.jsonl", issues)
    events = read_records(folder / "events.jsonl", issues)
    row.update(decision_metrics(records, events))
    row["evidence_issue_count"] = len(issues)
    row["evidence_issues"] = " | ".join(issues)
    return row


def aggregate_trials(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Group paired settings; expose denominators rather than filling unknowns with zero."""
    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        groups.setdefault(tuple(row[field] for field in GROUP_FIELDS), []).append(row)
    aggregates = []
    for key, trials in sorted(groups.items(), key=lambda item: str(item[0])):
        valid = [row for row in trials if row["valid_performance_run"]]
        aggregate = {**dict(zip(GROUP_FIELDS, key)), "runs": len(trials),
                     "valid_performance_runs": len(valid),
                     "infrastructure_failures": sum(row["infrastructure_failure"] for row in trials),
                     "unknown_outcomes": sum(row["outcome"] == "unknown" for row in trials),
                     "semantic_task_success": "unknown", "semantic_success_known_runs": 0}
        reviewed = [row["semantic_task_success"] for row in valid
                    if type(row["semantic_task_success"]) is bool]
        aggregate["semantic_success_known_runs"] = len(reviewed)
        aggregate["semantic_success_rate"] = statistics.mean(reviewed) if reviewed else None
        claims = [row["false_completion"] for row in valid if type(row.get("false_completion")) is bool]
        aggregate["false_completion_reviewed_claims"] = len(claims)
        aggregate["false_completion_rate"] = statistics.mean(claims) if claims else None
        aggregate["semantic_success_unknown_runs"] = len(valid) - len(reviewed)
        for field in PERFORMANCE_FIELDS + RESOURCE_FIELDS + ("visibility_targets_total", "visibility_targets_observed", "visibility_targets_unknown", "visibility_ordered_stages_completed", "visibility_first_visible_elapsed_sec", "visibility_ordered_stages_total"):
            pool = trials if field in RESOURCE_FIELDS else valid
            values = [float(row[field]) for row in pool if finite_number(row.get(field))]
            aggregate[f"{field}_n"] = len(values)
            aggregate[f"{field}_mean"] = statistics.mean(values) if values else None
            aggregate[f"{field}_std"] = statistics.stdev(values) if len(values) > 1 else None
        aggregates.append(aggregate)
    return aggregates


def write_csv_atomic(path: Path, rows: list[dict[str, Any]]) -> None:
    """Replace a complete CSV atomically, leaving no partial report on interruption."""
    fields = list(rows[0]) if rows else ["trial_id"]
    temporary: str | None = None
    try:
        with tempfile.NamedTemporaryFile("w", dir=path.parent, prefix=f".{path.name}.",
                                         suffix=".tmp", newline="", delete=False) as output:
            temporary = output.name
            writer = csv.DictWriter(output, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            Path(temporary).unlink(missing_ok=True)


def summarize_campaign(campaign: Path) -> tuple[Path, Path]:
    """Create per_trial.csv and aggregate.csv from all discovered trial directories."""
    campaign = campaign.expanduser().resolve()
    if not campaign.is_dir():
        raise ValueError(f"Campaign directory does not exist: {campaign}")
    trials = [path.parent for path in sorted(campaign.rglob("trial.json"))
              if "provenance" not in path.relative_to(campaign).parts]
    if not trials:
        raise ValueError(f"No trial.json files found in {campaign}")
    rows = [summarize_trial(folder, campaign) for folder in trials]
    per_trial, aggregate = campaign / "per_trial.csv", campaign / "aggregate.csv"
    write_csv_atomic(per_trial, rows)
    write_csv_atomic(aggregate, aggregate_trials(rows))
    return per_trial, aggregate


def main() -> None:
    """Summarize one campaign without ROS, model calls or external dependencies."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign", type=Path, help="Campaign folder containing recorded trials")
    args = parser.parse_args()
    try:
        paths = summarize_campaign(args.campaign)
    except ValueError as error:
        parser.error(str(error))
    for path in paths:
        print(path)


if __name__ == "__main__":
    main()
