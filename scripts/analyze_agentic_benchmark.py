#!/usr/bin/env python3
"""Replay closed benchmark bags, prepare review packets, and generate campaign figures."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    """Analyze saved trials without starting the simulator or calling a model.

    :return: None; a nonzero exit indicates at least one failed replay.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaign', type=Path)
    parser.add_argument('--image', default='unified_autonomy:ros2_agentic_uas')
    parser.add_argument('--skip-replay', action='store_true', help='Refresh reviews and plots from existing replay results')
    args = parser.parse_args()
    campaign = args.campaign.expanduser().resolve()
    trials = sorted(campaign.rglob('trial.json'))
    if not trials:
        parser.error(f'No trial.json files below {campaign}')
    failures = []
    if not args.skip_replay:
        for trial in trials:
            folder = trial.parent
            if not (folder / 'bag/metadata.yaml').exists():
                failures.append({'trial': str(folder.relative_to(campaign)), 'error': 'No closed bag metadata'})
                continue
            print(f'Replaying {folder.relative_to(campaign)}', flush=True)
            result = subprocess.run([
                'docker', 'run', '--rm', '-v', f'{ROOT / "workspaces/ws_agentic_uas"}:/workspace',
                '-v', f'{campaign}:/results', args.image,
                'ros2', 'run', 'agentic_uas', 'replay_benchmark',
                str(Path('/results') / folder.relative_to(campaign)),
            ], check=False)
            if result.returncode:
                failures.append({'trial': str(folder.relative_to(campaign)), 'exit_code': result.returncode})
    if not args.skip_replay:
        (campaign / 'analysis_failures.json').write_text(json.dumps(failures, indent=2) + '\n')
    elif (campaign / 'analysis_failures.json').exists():
        failures = json.loads((campaign / 'analysis_failures.json').read_text())
    # Review and plotting commands do not alter the original recordings or labels.
    subprocess.run([sys.executable, str(ROOT / 'scripts/review_agentic_benchmark.py'),
                    'prepare', str(campaign)], check=True)
    subprocess.run([sys.executable, str(ROOT / 'scripts/plot_agentic_benchmark.py'),
                    str(campaign)], check=True)
    if failures:
        raise SystemExit(f'{len(failures)} trials could not be replayed; see analysis_failures.json')


if __name__ == '__main__':
    main()
