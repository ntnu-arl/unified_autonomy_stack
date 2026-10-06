# Evaluation pipeline validation — 2026-10-05

These are short real-model pipeline trials using `gpt-6.1-sol`, not a statistically meaningful ranking or the full benchmark. The configured study has 162 runs with 180–300 second task budgets; these checks used 30–35 seconds. Semantic task success remains ungraded.

| World / task | Method | Calls | Travel (m) | Observed occupied voxels |
| --- | --- | ---: | ---: | ---: |
| office / search | `frontier` | 4 | 13.54 | 554 |
| office / search | `frontier_scene_graph` | 3 | 12.17 | 571 |
| office / search | `graph_bearing` | 5 | 14.08 | 579 |
| office / search | `graph_sample` | 4 | 16.47 | 625 |
| office / search | `graph_scene_graph` | 3 | 16.06 | 538 |
| office / search | `image_bearing` | 3 | 6.24 | 498 |
| hotel / sequential | `graph_scene_graph` | 3 | 18.94 | 624 |
| airport / exploration | `frontier_scene_graph` | 4 | 29.47 | 1137 |

All eight trials reached their time budget and stopped cleanly. All 29 API calls have recorded responses, with zero API/validation errors and zero incomplete calls. Offline bag audits read 684,525 messages, matching metadata, with no missing required topics and no transform/decoding failures. Each bag's first native graph snapshot decoded successfully.

Travel and occupied-voxel counts above come from offline replay with exact timestamped TF and uniform 0.5-second simulated-time LiDAR sampling. Coverage is a proxy, not a percentage of reachable free space. These measurements do not demonstrate object recognition or ordered task completion.

## Artifacts

- `evaluation_results/pilot-six-methods/`: all six methods on office object search.
- `evaluation_results/validated-hotel-sequential/`: method 5 on hotel ordered inspection.
- `evaluation_results/validated-airport-exploration/`: method 6 on airport exploration.

Each directory contains `per_trial.csv`, `aggregate.csv`, source provenance, exact model inputs/outputs and per-trial bags/audit reports. The recordings are intentionally Git-ignored. Other `pilot-*` directories contain development diagnostics and invalid runs; they are not part of the table.

## Issues found and addressed

- Stock hotel and airport worlds use a 10 ms physics step. The hotel initially drifted to roughly 8 m before its task. The runner now saves a derived world using 1 ms physics and the evaluator requires bounded, settled takeoff before starting. Source worlds are preserved.
- Live zstd file compression produced malformed SQLite data on this installation. The final recorder uses raw SQLite storage with a post-close integrity check; RGB remains JPEG-compressed. Invalid compressed recordings are retained and marked invalid.
- Mixed-type LiDAR messages need structured XYZ extraction. Exact-time TF lookup also needs a bounded retry rather than dropping every scan while TF catches up.
- Startup deadlines must use a steady-clock timer so a missing simulation clock cannot disable the watchdog.
- Closing the bag immediately on completion could miss queued or in-flight decisions. Shutdown now drains those records with a bounded timeout; motion metrics stop at the task boundary.

## Checks

- 102 agent-package tests and 12 host-runner/report tests passed.
- Affected ROS 2 workspace built successfully; package pre-commit checks passed.
- Missing-clock startup test timed out and flushed its diagnostics as expected.
- All evaluation containers were stopped after the runs.

The full world × task × method × repetition matrix has not been run. Validate task observability/reachability and use independent evidence review before treating model completion as semantic success. A larger output disk is needed for the full study.

## Ground-truth visibility and review workflow validation

The evaluation extension was checked against all eight saved valid pilot bags and
one new 20-second office/search `graph_sample` trial with `--rviz`. The new live
trial reached its time budget, stopped cleanly, recorded both model responses, and
opened the ROS 1 planner RViz window. Its bag audit read **64,022 messages**, matching
metadata, with no missing required topics or transform/decode failures. Artifacts:
`evaluation_results/visibility-rviz-check/`.

Visibility replay evaluated **70 exact RGB/depth/TF frames per office pilot**,
**60 hotel frames**, and **40 frames in the new trial**, with no frame failures.
Airport exploration correctly reports object visibility as not applicable because
it has no configured target objects.

- In the office, two of the six short pilots obtained depth-supported views of the
  Coke mesh; the other four did not. This is geometric evidence, not a success ranking.
- In the hotel, the kiosk and vending machines have supported visible surfaces.
  One suitcase has 940 estimated supported pixels while the other has zero and is
  occluded. The `all` policy therefore rejects visibility of the suitcase pair;
  the ordered task receives only its first visibility stage.
- Synthetic tests check occlusion, mesh holes/background rejection, camera transforms,
  missing depth/TF, exact synchronization, stale geometry, and ordered-stage credit.
- The CLI review workflow was exercised on a temporary trial copy. No artificial
  success labels were added to the scientific recordings. Semantic grades remain unknown.

New trials freeze transformed visual meshes and source hashes at capture time.
Older bags were explicitly enriched without modifying their original `trial.json`;
those sidecars flag historical asset equality as unverified. Formal comparisons
should use newly captured, fully frozen fixtures.

The combined check passed **108 agent-package tests and 27 host tests**. The affected
workspace built successfully; package pre-commit and host Ruff checks passed.
The analysis command generated review packets, PNG/PDF figures, CSV summaries and
Markdown tables for all three pilot task types and the fresh RViz trial. These are
pipeline checks, not a complete study or independently reviewed method ranking.
