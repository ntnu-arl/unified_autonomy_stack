# Running and reviewing agentic autonomy benchmarks

The benchmark runs all six configurable methods in the Unipilot simulation, records
model inputs/outputs and ROS bags, and evaluates motion and object visibility offline.
**Object visibility is not task success.** A separate reviewer checks whether the
agent correctly recognized the requested objects and completed the instruction.

## 1. Prepare the environment

Run commands from the `unified_autonomy_stack` repository root. Build the simulation,
planner, controller, scene graph and agent images/workspaces first. After updating
the evaluation code, rebuild the affected workspace:

```bash
make build-agentic_uas
```

Set `OPENAI_API_KEY` in your shell using your normal secret-management procedure.
The benchmark makes real model calls. Stop an existing simulation before starting:
the scene graph endpoints on ports 8002–8004 cannot be shared. The runner checks
those ports and uses an isolated Compose project, ROS domains 218/219, ROS master
port 11321 and Gazebo partition. It tears down only its own containers.

Install the host analysis dependencies in a virtual environment:

```bash
python3 -m venv .venv-evaluation
source .venv-evaluation/bin/activate
python -m pip install numpy PyYAML matplotlib
```

Keep this environment active for the commands below. ROS bag replay runs inside
the existing agent Docker image; host ROS installation is unnecessary.

## Stopping a run and recovering after an interruption

Press **Ctrl-C once in the benchmark terminal** to stop an active campaign. The
runner finishes recording cleanup before exiting; SIGTERM also follows this path.
Do not close the terminal repeatedly while containers are shutting down.

If a killed process or interrupted Docker startup leaves containers behind:

```bash
make stop-benchmark
```

This stops only abandoned benchmark containers belonging to this checkout, even
when their saved Compose file has been deleted. Recordings and model-cache volumes
are preserved. `make stop` also invokes this cleanup after stopping the normal
stack. An active benchmark holding the runner lock is protected: stop it with
Ctrl-C first. New benchmark runs automatically clean abandoned owned containers
before checking ports. Other stacks and unrelated processes are never stopped;
remaining port conflicts include an `ss` diagnostic command.

## 2. Start with one short trial

```bash
# Inspect the matrix without starting containers or making model calls.
python scripts/run_agentic_benchmark.py --list

# One 35-simulated-second pilot, with the normal ROS 1 planner RViz visible.
python scripts/run_agentic_benchmark.py \
  --campaign office-check --methods graph_sample --worlds office --tasks search \
  --repetitions 1 --duration 35 --rviz
```

Omit `--rviz` for headless benchmarks. `--rviz` requires the same working X display
as your normal simulation launch; it shows ROS 1 RViz, not the Gazebo GUI. RViz adds
rendering load, so use the same setting across compared trials. Do not use its
planner/agent controls during a measured trial: the benchmark handles takeoff,
assigning the task, starting the agent and stopping it.

The terminal prints container health and evaluator progress every five seconds,
including missing sensors/services, takeoff and task time. A required container
exit or unhealthy state prints its recent logs and stops the trial. The trial
folder printed at startup contains `runner.log` and `containers.log`. Missing
Unipilot planner YAML files are rejected before launching containers.

Each campaign name must be new. A retry cannot overwrite an earlier run. Use
`--output /path/to/large/disk` to change storage location. `--model MODEL_ID`
overrides the common model; otherwise the normal agent profile's model is used.
`--dry-run` prints the selected cases without launching them. `--max-runs 1` limits
the matrix to its first case.

## 3. Run paired comparisons

```bash
# All six methods, one world/task, five repetitions each.
python scripts/run_agentic_benchmark.py \
  --campaign office-search-comparison --worlds office --tasks search --repetitions 5

# Full supplied matrix: 6 methods × 3 worlds × 3 tasks × 3 repeats = 162 trials.
python scripts/run_agentic_benchmark.py \
  --campaign full-study --worlds office hotel airport \
  --tasks exploration search sequential --repetitions 3
```

Default episode budgets are 300 s exploration, 180 s search and 300 s sequential.
The full matrix therefore has 11.7 hours of simulated task budgets, plus startup,
model latency and reset overhead. Raw depth bags require substantial disk space.
The runner refuses new trials below 3 GiB free; the observer stops below 2 GiB to
leave room to flush recordings. Begin with a pilot, not the full matrix.

All methods share the starting pose, task, budgets, controller settings and model
profile. Each trial starts fresh processes, maps and agent memory; model caches
persist. Method order rotates by repetition. Gazebo receives `--seed` plus the
repetition index (default seed 42). This does not seed GBPlanner or make remote
model inference deterministic. Record failures; do not silently replace them.

| Method | CLI/config identifier | Input → output |
| --- | --- | --- |
| 1 | `image_bearing` | Current image → optical bearing + distance |
| 2 | `graph_bearing` | Sparse graph + current image → optical bearing + distance |
| 3 | `graph_sample` | Sparse graph + posed current image/keyviews → sample ID |
| 4 | `frontier` | Frontiers + posed current image/keyviews → frontier ID |
| 5 | `graph_scene_graph` | Method 3 + scene graph → sample ID |
| 6 | `frontier_scene_graph` | Method 4 + scene graph → frontier ID |

Frontier methods retain their normal graph/imaginary-sample fallbacks. Actual input
sources are logged per call. Hydra runs and warms up for every method, although a
received graph can still contain no objects. These are comparisons of complete
strategies, not isolated changes to one input variable.

## 4. Replay, inspect evidence, and produce figures

After a campaign finishes:

```bash
python scripts/analyze_agentic_benchmark.py evaluation_results/office-check
```

This command replays each closed bag using exact recorded TF, evaluates geometric
visibility, creates portable HTML review packets, and generates PNG/PDF figures,
Markdown tables and CSV summaries. It starts no simulation and makes no model calls.
Failures are listed in `analysis_failures.json` and cause a nonzero exit status;
missing evidence stays unknown. Use `--skip-replay` to regenerate packets and plots
from existing replay results.

Open `evaluation_results/office-check/review_packets/index.html` in a browser.
Each packet includes the task, independent visibility images, the actual images
submitted to the model, its full structured outputs and the decision application
records. Method names are hidden by default, but overlays can reveal the method:
this is partial blinding. Keep `review_packet_mapping.json` with the experiment
coordinator when distributing the packets to reviewers.

To save judgments directly from the HTML pages, start the local review application:

```bash
python scripts/review_agentic_benchmark.py serve evaluation_results/office-check
```

Open **http://127.0.0.1:8765/** and select a trial. The server regenerates its review
packets automatically, so older campaigns receive the new form. Use **Go to judgment
form** to jump to the form, inspect the evidence above it, and click **Save judgment
to trial**. This writes the trial's `review.json` directly; no per-trial Python command
is needed. Keep the server terminal open while reviewing; Ctrl-C stops it. Use
`--port 8766` if the default port is occupied. No ROS or simulator is needed.

Fill the fields as follows:

| Field | What to enter |
| --- | --- |
| **Reviewer name or identifier** (required) | Your name or a stable alias, such as Albert or Reviewer A. Use the same identifier across trials. It identifies who reviewed the evidence. |
| **Task result** (required) | **Success** if the recorded evidence establishes that the agent satisfied the full instruction; **Failure** if sufficient evidence establishes that it did not satisfy the task by trial end; **Unknown** when the evidence or task criterion is insufficient. For sequential tasks, success requires every stage in order. Seeing a target or entering its region alone is insufficient. |
| **Evidence and justification** (required) | Exact iteration/image identifiers or simulation timestamps, what they show, and why that supports your result. For sequential tasks, cite each stage and its order. For Unknown, explain what evidence or criterion is missing. Example: “Iteration 000004/image_000.jpg shows the requested chair; the applied answer identifies it and finishes the task.” |
| **Correctness of final completion claim** | **Correct** if the agent declared completion and the evidence supports it; **Incorrect** if it declared completion but the evidence contradicts it; **Not assessed / no final declaration** if no declaration exists or you cannot judge it. This field is disabled when the recording has no final declaration. A timeout is not a false completion claim. |
| **Replace the existing saved judgment** | Leave unchecked for a first review. Check only when intentionally revising an existing review. Existing judgments are loaded into the form; saving without this checkbox protects them. |

The form and server reject contradictory judgments, such as Success with an
Incorrect completion claim. Unknown results are stored as null and excluded from
the success-rate denominator. Reviews include the trial hash, reviewer, evidence
and save time. The summarizer reads the saved review on its next run.

**Offline/shared packets:** if you open an HTML file directly, **Download review.json**
exports the same judgment without a server. Put that file in the corresponding
trial directory to include it in summaries. Downloading does not automatically
change the trial; the direct-save button requires the local review server. Existing
CLI `review` commands remain available for scripted workflows.

Use the same rubric for every method:

- **Search:** the requested instance is supported by image evidence and the agent
  correctly identifies it. Merely flying near it or incidentally seeing it is insufficient.
- **Sequential:** all requested instances are identified and inspected in the required
  order. Check timestamps and applied decisions; a discarded prefetched answer does
  not establish completion.
- **Exploration:** review whether the report accurately describes observed areas and
  whether the preregistered task criterion was met. The current open-ended prompt
  does not define exhaustive exploration. Do not grade “fully explored” without a
  separately established coverage criterion.

If the packet is insufficient, inspect the bag/recording and cite that evidence, or
leave the result unknown. Review every valid trial before comparing success rates.

Refresh figures after reviewing:

```bash
python scripts/plot_agentic_benchmark.py evaluation_results/office-check
```

Outputs are `per_trial.csv`, `aggregate.csv` and `report/`. Figures are separated by
model, world and task: individual trial dots, means, reviewed success with Wilson
95% intervals, runtime, distance, model calls/latency, coverage proxy, and visibility
metrics. Tables show denominators, ungraded trials and false completion judgments.
Runtime includes timeouts; it is not automatically time-to-success. No arbitrary
weighted overall ranking is produced. Small pilot runs do not establish a ranking.

## 5. Change or add tasks

The manifest is
`workspaces/robot_bringup/config/evaluation/agentic_benchmark.yaml`.
Copy it for a separate experiment rather than changing a running campaign:

```bash
cp workspaces/robot_bringup/config/evaluation/agentic_benchmark.yaml /tmp/my-benchmark.yaml
# Edit /tmp/my-benchmark.yaml, then inspect the selected cases.
python scripts/run_agentic_benchmark.py --manifest /tmp/my-benchmark.yaml \
  --worlds office --tasks search --list
```

Within `worlds.<world>.tasks.<task>` configure:

```yaml
prompt: Find the Coke can and report visual evidence that you found it.
duration_sec: 180
targets:
  - id: coke
    label: Coke can
    position: [17.766, -5.507, 1.20]  # Approximate center for region-visit metrics only.
    radius: 2.0                    # Region radius; does not establish visibility.
    visibility:
      policy: any                  # any: one listed instance; all: all in the same frame.
      instances: ["Coke"]           # Exact <include><name> in the world SDF.
stages: [coke]                     # Ordered target IDs; later stages are not pre-credited.
```

For a sequential task, add targets and list their IDs in the required order in
`stages`. `policy: all` is appropriate when the instruction requires seeing a group,
such as the suitcase pair; it requires simultaneous visibility under the current
rubric. For exploration without specific targets, use `targets: []`, `stages: []`;
object visibility is then not applicable.

World settings select the SDF filename, its internal world name (`cosmos` for the
supplied worlds), start `[x,y,z,yaw]`, and metric ROI
`[xmin,ymin,zmin,xmax,ymax,zmax]`. Keep targets and coordinates evaluator-only:
the agent receives just the natural-language task prompt. Validate reachability,
visibility and wording before comparing methods. Static doors/lifts cannot be
operated; current hotel tasks stay on the ground floor.

Validate target geometry before running a new task:

```bash
python scripts/ground_truth_assets.py --manifest /tmp/my-benchmark.yaml \
  --worlds-root workspaces/ws_sim/src/gz_sim_worlds \
  --world office --task search --output /tmp/my-task-geometry
```

The extractor uses actual static visual triangles and SDF transforms, not approximate
region centers. Current OBJ and Z-up COLLADA assets are supported. Unsupported
geometry/transforms fail explicitly. Adding a new object format requires extending
and testing the extractor. Dynamic objects need timestamped ground-truth poses and
are not supported by this evaluator.

## Visibility definition and limitations

At a fixed simulated-time interval, replay pairs RGB and metric depth by identical
header timestamps, checks calibration/alignment, and queries exact-time camera TF.
It rasterizes the target's true visual mesh with perspective-correct depth and
compares it to measured depth. Surfaces behind nearer depth are occluded. This is
independent of Hydra detections and the agent's scene graph.

Shared YAML `visibility` defaults:

| Setting | Default | Meaning |
| --- | --- | --- |
| `interval_sec` | 0.5 s | Sampling interval in simulated time |
| `pixel_stride` | 2 | Evaluate every second pixel in both dimensions |
| `minimum_pixels` | 25 | Minimum supported area, estimated in full-resolution pixels |
| `minimum_visible_fraction` | 0.1 | Supported fraction of the projected mesh surface |
| `depth_tolerance_m` | 0.05 m | Allowed mesh-versus-measured depth difference |

At least three sampled pixels must also support the target. Small objects can fall
below these thresholds even when a person notices them. Freeze thresholds across
methods; use stride 1 for more precise small-object measurements. Depth tolerance
and sampling make geometric visibility approximate, not a semantic identity oracle.
Missing depth/TF/calibration is reported, not treated as successful observation.

`visibility.json` records first-visible times, per-target evidence and ordered
visibility stages. `visibility_evidence/` stores annotated images. These measure
viewing opportunities, not recognition or understanding. Exploration still uses an
occupied-LiDAR-endpoint voxel proxy; ground-truth coverage, optimal-path efficiency
and collision rates are not implemented.

## Artifacts and reproducibility

Trial folders are grouped as
`evaluation_results/<campaign>/<model>/<world>/<task>/<method>/seed-<seed>/`:

- `trial.json`, `agent.yaml`, `config.json`, `system_prompt.txt`: frozen task/settings.
- `world.sdf`, `ground_truth.json`, `ground_truth_meshes.npz`: evaluated world and target
  surfaces with source hashes. The harness normalizes physics to 1 ms in a saved world
  copy; source worlds remain unchanged.
- `iterations/`, `iterations.jsonl`, `events.jsonl`: exact model JPEGs/requests, full
  responses, structured explanations, usage and applied/discarded decisions.
- `bag/`: compressed RGB, metric depth, camera calibration, TF, clock, odometry,
  LiDAR, GBPlanner paths/status, agent visualization/status/decisions, NMPC paths,
  Hydra markers and native Spark-DSG snapshots.
- `metrics.json`, `outcome.json`, `runner_outcome.json`: online measurements/outcome.
- `replay_metrics.json`, `bag_validation.json`, `visibility.json`: offline results.
- `review.json`: independent review; never inferred from model self-report.

Campaign provenance archives source revisions, dirty diffs/new source files, image
identities and the manifest. Nested repositories are versioned separately. API keys
are not archived. Recordings are Git-ignored. Infrastructure failures and independent
`validation.json` exclusions remain in summaries with reasons.

For older recordings without frozen target geometry, explicit enrichment is available:

```bash
python scripts/ground_truth_assets.py \
  --manifest workspaces/robot_bringup/config/evaluation/agentic_benchmark.yaml \
  --worlds-root workspaces/ws_sim/src/gz_sim_worlds --trial /path/to/trial
```

It checks the saved world hash and task agreement and leaves `trial.json` untouched.
Historical asset equality is not established by a world-file hash alone; retrospective
sidecars explicitly mark that limitation. Prefer new trials with meshes frozen at
capture time for formal comparisons. See [validation results](agentic_evaluation_results.md)
for what has actually been tested.

### Model iteration budget

Use `--max-model-iterations 20` with `scripts/run_agentic_benchmark.py` to cap each trial at 20 model decision calls. The default is `max_model_iterations` in the benchmark manifest; `0` means unlimited. The resolved limit is saved in `trial.json` and `agent.yaml`. For ordinary launches, set `max_model_iterations` in the agent YAML.

Calls include prefetch and failed decisions, but not individual SDK transport retries. The final permitted decision is processed normally; a waypoint can finish executing before the next reasoning attempt ends the trial with `iteration_budget`. The limit does not imply task success. Changing the task resets the counter; stopping and restarting the same task does not. Duration and wall-clock limits still apply.

### Resume after interruption or disk exhaustion

Free disk space, then repeat the original benchmark command with `--resume`. Use the same campaign, model, duration, iteration budget and trial selection. Completed trials (`model_completed`, `time_budget`, `iteration_budget` with exit code 0) are skipped. Failed or unfinished trials restart from a fresh simulation; their original files are moved to `evaluation_results/.interrupted/<campaign>/<timestamp>/...`, outside the active campaign so reporting does not count them twice. Nothing is deleted. Resuming a simulation halfway through a trial is not supported.

Use `--resume --dry-run` to inspect what will be skipped or restarted without launching containers or moving recordings. Changed task or agent settings are rejected to avoid silently mixing experiments. Original provenance is retained, and the resumed checkout is recorded under `resume_provenance/`. A storage-limit outcome stops the campaign until space is freed.

Measurement plots overlay individual trials on mean bars, with horizontal lines marking the median. Whiskers show ±1 sample standard deviation across valid trials, not confidence intervals; SD is undefined and omitted for a single trial. Reviewed-success plots retain Wilson 95% confidence intervals. The metric table includes mean, median and SD.
