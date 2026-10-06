#!/usr/bin/env python3
"""Plot reviewed success, trial measurements and explicit evidence gaps per scenario.

Requires matplotlib. Each model/world/task gets a separate figure; scenarios are
not pooled. Dots are individual valid trials; bars show means and horizontal lines show medians, with
sample standard deviation whiskers on the mean (omitted for a single trial). Success uses
Wilson 95% intervals among reviewed valid trials; unreviewed trials are not failures.
"""
from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
import statistics

from summarize_agentic_benchmark import summarize_campaign, summarize_trial, finite_number

METHODS = ('image_bearing', 'graph_bearing', 'graph_sample', 'frontier', 'graph_scene_graph', 'frontier_scene_graph')


def wilson(successes: int, count: int) -> tuple[float, float]:
    """Calculate a binomial Wilson 95% confidence interval.

    :param successes: Number of positive judgments.
    :param count: Number of independently graded trials.
    :return: Lower and upper confidence limits; NaNs when ungraded.
    """
    if not count:
        return math.nan, math.nan
    z = 1.959963984540054
    p = successes / count
    denominator = 1 + z * z / count
    center = (p + z * z / (2 * count)) / denominator
    half = z * math.sqrt(p * (1 - p) / count + z * z / (4 * count * count)) / denominator
    # Roundoff at 0/n and n/n must not place the bound beyond the observed rate.
    return max(0, min(p, center - half)), min(1, max(p, center + half))


def plot_measurements(axis, index: int, values: list[float]) -> None:
    """Show individual measurements with mean, median and sample variability.

    :param axis: Matplotlib axis for one metric.
    :param index: Method position on the horizontal axis.
    :param values: Finite measurements from valid trials; must be nonempty.
    :return: None.
    """
    first = not axis.has_data()
    deviation = statistics.stdev(values) if len(values) > 1 else None
    axis.bar(index, statistics.mean(values), width=.62, color='#D55E00',
             alpha=.65, yerr=deviation, capsize=4, zorder=2,
             label='Mean ± sample SD' if first else None)
    axis.hlines(statistics.median(values), index - .31, index + .31,
                color='#0072B2', linewidth=2.5, zorder=3,
                label='Median' if first else None)
    # Deterministic separation keeps repeated measurements visible above the bars.
    offsets = [0.] if len(values) == 1 else [-.28 + .56*i/(len(values)-1) for i in range(len(values))]
    axis.scatter([index+offset for offset in offsets], values, color='#222222',
                 edgecolors='white', linewidths=.5, s=23, zorder=4,
                 label='Individual trials' if first else None)
    if first:
        axis.legend(fontsize=6, loc='upper right')


def create_report(campaign: Path, output: Path) -> list[Path]:
    """Generate scenario-specific figures and a Markdown table from original evidence.

    :param campaign: Campaign directory with trial folders.
    :param output: Report directory.
    :return: Created artifact paths.
    """
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    summarize_campaign(campaign)
    rows = [summarize_trial(path.parent, campaign) for path in sorted(campaign.rglob('trial.json'))
            if 'provenance' not in path.relative_to(campaign).parts]
    groups = {}
    for row in rows:
        groups.setdefault((row['model'], row['world_id'], row['task_id']), []).append(row)
    output.mkdir(parents=True, exist_ok=True)
    markdown = ['# Benchmark measurements', '',
                'Success is independently reviewed task completion. Unknown reviews are excluded from the success denominator and reported explicitly. Wilson 95% intervals describe reviewed trials only; selective review can bias results. Review every valid trial before ranking.', '',
                'Each dot is one valid trial; orange bars show means, blue horizontal lines show medians, and whiskers show ±1 sample standard deviation (undefined and omitted for n=1). Runtime includes unsuccessful budget-limited trials and is not time-to-success. Occupied endpoint voxels are a coverage proxy, not ground-truth exploration coverage. Figures compare methods only within the same model, world and task. Small pilot runs do not support rankings.', '',
                '| Model / world / task | Method | Valid / total | Reviewed success (95% CI) | Ungraded | False completion / reviewed claims | Runtime sim s | Distance m | Model calls | Occupied voxels (proxy) |',
                '|---|---|---:|---|---:|---:|---:|---:|---:|---:|']
    visibility_table = ["", "## Geometric viewing opportunities", "",
                        "Depth-consistent simulator mesh visibility provides geometric evidence of a view, not semantic recognition. Unknown geometry/data stays unknown. First-visible times include only observed targets; never compare them without the visibility counts. Ordered counts measure recorded viewing order, not reasoning correctness.", "",
                        "| Model / world / task | Method | Visible / known target opportunities | Unknown targets | First target visible sim s | Ordered visible stages / total |",
                        "|---|---|---:|---:|---|---:|"]
    paths = []
    for key, group in sorted(groups.items(), key=lambda item: str(item[0])):
        present = {row['method'] for row in group}
        methods = [method for method in METHODS if method in present] + sorted(present - set(METHODS))
        fig, axes = plt.subplots(2, 3, figsize=(16, 10), constrained_layout=True)
        fig.suptitle(' / '.join(str(value) for value in key) + '\nIndividual trials; success requires independent review', fontsize=14)
        for index, method in enumerate(methods):
            trials = [row for row in group if row['method'] == method]
            valid = [row for row in trials if row['valid_performance_run']]
            reviewed = [row['semantic_task_success'] for row in valid if type(row['semantic_task_success']) is bool]
            successes = sum(reviewed)
            low, high = wilson(successes, len(reviewed))
            if reviewed:
                rate = successes / len(reviewed)
                axes.flat[0].errorbar(index, rate, yerr=[[rate-low], [high-rate]], fmt='o', capsize=5, color='#0072B2')
                axes.flat[0].text(index, 1.09, f'{successes}/{len(reviewed)}; ?{len(valid)-len(reviewed)}', ha='center', fontsize=8)
                success_text = f'{successes}/{len(reviewed)} ({low:.2f}–{high:.2f})'
            else:
                axes.flat[0].text(index, .5, f'UNREVIEWED\nn={len(valid)}', ha='center', fontsize=8, rotation=90)
                success_text = 'unknown'
            fields = ('elapsed_sim_sec', 'path_length_m', 'model_calls', 'model_latency_mean_sec', 'observed_occupied_voxels_proxy')
            for axis, field in zip(list(axes.flat)[1:], fields):
                values = [row[field] for row in valid if finite_number(row.get(field))]
                if values:
                    plot_measurements(axis, index, values)
                else:
                    axis.text(index, .5, 'missing', transform=axis.get_xaxis_transform(), ha='center', rotation=90)
            def mean(field: str) -> str:
                values = [row[field] for row in valid if finite_number(row.get(field))]
                if not values:
                    return 'unknown'
                sd = f'{statistics.stdev(values):.2f}' if len(values) > 1 else 'undefined'
                return f'mean {statistics.mean(values):.2f}; median {statistics.median(values):.2f}; SD {sd} (n={len(values)})'
            claims = [row['false_completion'] for row in valid if type(row.get('false_completion')) is bool]
            label = ' / '.join(str(value).replace('|', '\\|') for value in key)
            markdown.append(f'| {label} | {method} | {len(valid)}/{len(trials)} | {success_text} | {len(valid)-len(reviewed)} | {sum(claims)}/{len(claims)} | {mean(fields[0])} | {mean(fields[1])} | {mean(fields[2])} | {mean(fields[4])} |')
        titles = ('Reviewed task success (95% Wilson CI)', 'Trial runtime (simulated seconds)', 'Distance traveled (m)', 'Model calls', 'Mean inference latency per trial (wall s)', 'Observed occupied voxels — proxy only')
        for axis, title in zip(axes.flat, titles):
            axis.set_title(title)
            axis.set_xticks(range(len(methods)), [method.replace('_', '\n') for method in methods], fontsize=8)
            axis.grid(axis='y', alpha=.25)
            axis.set_xlim(-.5, len(methods)-.5)
            axis.set_ylim(bottom=min(0, axis.get_ylim()[0]),
                          top=axis.get_ylim()[1] * 1.25)
        axes.flat[0].set_ylim(-.05, 1.2)
        name = '-'.join(str(value).replace('/', '_') for value in key)
        name += '-' + hashlib.sha256(repr(key).encode()).hexdigest()[:6]
        for extension in ('png', 'pdf'):
            path = output / f'{name}.{extension}'
            fig.savefig(path, dpi=150)
            paths.append(path)
        plt.close(fig)
        visibility_fig, visibility_axes = plt.subplots(1, 3, figsize=(16, 5), constrained_layout=True)
        visibility_fig.suptitle(' / '.join(str(value) for value in key) + '\nGeometric visibility evidence — not semantic task success')
        for index, method in enumerate(methods):
            valid = [row for row in group if row['method'] == method and row['valid_performance_run']]
            observed = [row for row in valid if finite_number(row.get('visibility_targets_total'))]
            target_count = sum(row['visibility_targets_total'] - (row.get('visibility_targets_unknown') or 0) for row in observed)
            visible_count = sum(row['visibility_targets_observed'] for row in observed)
            unknown_count = sum(row.get('visibility_targets_unknown') or 0 for row in observed)
            times = [row['visibility_first_visible_elapsed_sec'] for row in observed if finite_number(row.get('visibility_first_visible_elapsed_sec'))]
            stages = sum(row.get('visibility_ordered_stages_completed') or 0 for row in observed)
            total_stages = sum(row.get('visibility_ordered_stages_total') or 0 for row in observed)
            visibility_table.append(f"| {' / '.join(str(value) for value in key)} | {method} | {str(visible_count)+'/'+str(target_count) if target_count else 'unknown'} | {unknown_count if observed else 'unknown'} | {f'{statistics.mean(times):.2f} (n={len(times)})' if times else 'unknown'} | {str(stages)+'/'+str(total_stages) if observed and not unknown_count else 'unknown'} |")
            fractions = [row['visibility_targets_observed']/row['visibility_targets_total'] for row in observed if row['visibility_targets_total'] and not row.get('visibility_targets_unknown')]
            ordered = [row['visibility_ordered_stages_completed']/row['visibility_ordered_stages_total'] for row in observed if row.get('visibility_ordered_stages_total') and not row.get('visibility_targets_unknown') and finite_number(row.get('visibility_ordered_stages_completed'))]
            for axis, values in zip(visibility_axes, (fractions, times, ordered)):
                if values:
                    plot_measurements(axis, index, values)
                else:
                    label = 'not observed' if axis is visibility_axes[1] and observed and not unknown_count else 'unknown'
                    axis.text(index, .5, label, transform=axis.get_xaxis_transform(), ha='center', rotation=90)
        for axis, title in zip(visibility_axes, ('Fraction of targets visibly observed', 'First target visible (sim s; observed trials)', 'Fraction of stages visibly observed in order')):
            axis.set_title(title, fontsize=10)
            axis.set_xticks(range(len(methods)), [method.replace('_', '\n') for method in methods], fontsize=8)
            axis.set_xlim(-.5, len(methods)-.5)
            axis.set_ylim(bottom=min(0, axis.get_ylim()[0]),
                          top=axis.get_ylim()[1] * 1.25)
            axis.grid(axis='y', alpha=.25)
        visibility_axes[0].set_ylim(bottom=min(-.05, visibility_axes[0].get_ylim()[0]),
                                    top=max(1.1, visibility_axes[0].get_ylim()[1]))
        visibility_axes[2].set_ylim(bottom=min(-.05, visibility_axes[2].get_ylim()[0]),
                                    top=max(1.1, visibility_axes[2].get_ylim()[1]))
        visibility_axes[1].set_ylim(top=max(1, visibility_axes[1].get_ylim()[1] * 1.15))
        for extension in ('png', 'pdf'):
            path = output / f'{name}-visibility.{extension}'
            visibility_fig.savefig(path, dpi=150)
            paths.append(path)
        plt.close(visibility_fig)

    table = output / 'metrics.md'
    table.write_text('\n'.join(markdown + visibility_table) + '\n')
    paths.append(table)
    return paths


def main() -> None:
    """Produce plots, CSV summaries and a Markdown table for one campaign."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaign', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:
        paths = create_report(args.campaign.resolve(), args.output or args.campaign / 'report')
    except (ValueError, ImportError) as error:
        parser.error(str(error))
    for path in paths:
        print(path)


if __name__ == '__main__':
    main()
