#!/usr/bin/env python3
"""Create portable evidence packets and record independent completion reviews."""
from __future__ import annotations

import argparse
import base64
from datetime import datetime, timezone
import hashlib
import html
import json
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import secrets
import threading
from urllib.parse import urlsplit


def declared_complete(folder: Path) -> bool:
    """Check whether this trial recorded a final completion declaration.

    :param folder: Recorded trial directory.
    :return: Whether completion was declared by the agent.
    """
    metrics = load_json(folder / 'metrics.json')
    metrics.update(load_json(folder / 'replay_metrics.json'))
    outcome = load_json(folder / 'runner_outcome.json') or load_json(folder / 'outcome.json')
    return metrics.get('model_declared_complete') is True or outcome.get('outcome') == 'model_completed'


def trial_digest(folder: Path) -> str:
    """Identify the exact trial manifest being reviewed.

    :param folder: Recorded trial directory.
    :return: SHA256 digest of the manifest bytes.
    """
    return hashlib.sha256((folder / 'trial.json').read_bytes()).hexdigest()


def load_json(path: Path) -> dict:
    """Read optional JSON evidence.

    :param path: JSON object path.
    :return: Object, or an empty object when absent.
    """
    return json.loads(path.read_text()) if path.exists() else {}


def packet(folder: Path, output: Path, reveal_method: bool = False) -> Path:
    """Build an offline HTML packet with embedded images and visible model statements.

    :param folder: Trial directory.
    :param output: HTML destination.
    :param reveal_method: Include the model and representation method.
    :return: Created HTML path.
    """
    trial = load_json(folder / 'trial.json')
    if not trial:
        raise ValueError('Missing trial.json')
    digest = trial_digest(folder)
    def pretty(value: object) -> str:
        return '<pre>' + html.escape(json.dumps(value, indent=2)) + '</pre>'
    def picture(path: Path, caption: str) -> str:
        # Evidence must stay inside this trial; never embed arbitrary external paths.
        if not path.resolve().is_relative_to(folder.resolve()) or not path.is_file():
            return '<p>Missing image: ' + html.escape(caption) + '</p>'
        mime = 'image/png' if path.suffix.lower() == '.png' else 'image/jpeg'
        data = base64.b64encode(path.read_bytes()).decode('ascii')
        return f'<figure><img src="data:{mime};base64,{data}"><figcaption>{html.escape(caption)}</figcaption></figure>'
    parts = ['<!doctype html><meta charset="utf-8"><title>Completion review</title>',
             '<style>body{font:16px sans-serif;max-width:1100px;margin:30px auto;padding:15px}pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#eee;padding:12px}img{max-width:100%}figure{display:inline-block;max-width:48%;vertical-align:top}section{border-top:1px solid #888;margin-top:25px}</style>',
             '<h1>Independent completion review</h1>',
             '<p><a href="#judgment">Go to judgment form</a></p>',
             f'<p>Trial code: <code>{digest[:12]}</code></p>',
             '<p>Judge the task from the images and recorded statements. Geometric visibility is evidence of a viewing opportunity, not semantic recognition. Mark unknown when evidence is insufficient. Verify sequential stages in order. Do not accept a model success claim without supporting evidence.</p>',
             '<p>Method identity is hidden by default, but overlays and outputs can reveal the representation; this is only partial blinding.</p>',
             pretty({'world': trial.get('world_id'), 'task': trial.get('task')})]
    if reveal_method:
        parts.append(pretty({key: trial.get(key) for key in ('method', 'model', 'seed')}))
    outcome = load_json(folder / 'runner_outcome.json') or load_json(folder / 'outcome.json')
    parts += ['<h2>Trial outcome</h2>', pretty(outcome), '<p>Prefetched or discarded decisions may not have been applied. Judge final completion claims using the trial outcome and application records below.</p>']
    visibility = load_json(folder / 'visibility.json')
    overview = {key: value for key, value in visibility.items()
                if key not in ('observations', 'ground_truth_provenance')}
    parts += ['<h2>Independent geometric visibility</h2>',
              pretty(overview or {'status': 'unknown', 'reason': 'Visibility evaluator has not been run'}),
              '<details><summary>Full visibility trace and geometry provenance</summary>', pretty(visibility), '</details>']
    for target, item in visibility.get('targets', {}).items():
        evidence = item.get('best_evidence') or {}
        if evidence.get('image'):
            parts.append(picture(folder / evidence['image'], f'Visibility target {target}: {evidence.get("sim_time", "?")} s'))
        if evidence.get('raw_image'):
            parts.append(picture(folder / evidence['raw_image'], f'Unannotated target evidence: {target}'))
    for index, evidence in enumerate(visibility.get('ordered_stage_evidence', [])):
        if evidence.get('image'):
            parts.append(picture(folder / evidence['image'],
                                 f'Ordered stage {index + 1}: {evidence.get("target")} at {evidence.get("sim_time")} s'))
    records = []
    source = folder / 'iterations.jsonl'
    if source.exists():
        for line in source.read_text().splitlines():
            try:
                record = json.loads(line)
                if record.get('kind') in ('validated', 'error'):
                    records.append(record)
            except ValueError:
                parts.append('<p>Warning: malformed iteration record.</p>')
    events = []
    if (folder / 'events.jsonl').exists():
        for line in (folder / 'events.jsonl').read_text().splitlines():
            try:
                event = json.loads(line)
                if event.get('kind') in ('applied', 'staged', 'discarded', 'rejected'):
                    events.append(event)
            except ValueError:
                parts.append('<p>Warning: malformed application event.</p>')
    for iteration in sorted((folder / 'iterations').glob('*')):
        if not iteration.is_dir():
            continue
        parts += ['<section><h2>Iteration ' + html.escape(iteration.name) + '</h2>']
        context = load_json(iteration / 'context.json')
        parts.append(pretty({'images': context.get('images', []), 'robot_position': context.get('robot_position')}))
        for image in sorted(iteration.glob('image_*.*')):
            if image.suffix.lower() in ('.jpg', '.jpeg', '.png'):
                parts.append(picture(image, f'{iteration.name}/{image.name}'))
        for record in records:
            if str(record.get('iteration_id', '')).lstrip('0') == iteration.name.lstrip('0'):
                parts.append(pretty({key: record[key] for key in ('kind', 'decision', 'error') if key in record}))
        dispositions = [event for event in events if str(event.get('iteration_id', '')).lstrip('0') == iteration.name.lstrip('0')]
        parts.append(pretty({'application_records': dispositions}))
        parts.append('</section>')
    form = Path(__file__).with_name('benchmark_review_form.html').read_text()
    data = {'code': digest[:12], 'trial_sha256': digest,
            'declared_complete': declared_complete(folder),
            'existing_review': load_json(folder / 'review.json')}
    # JSON inside a script element must not permit an embedded </script> tag.
    encoded = json.dumps(data).replace('<', '\\u003c').replace('>', '\\u003e').replace('&', '\\u0026')
    parts.append(form.replace('REVIEW_DATA_JSON', encoded))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text('\n'.join(parts))
    return output


def save_review(folder: Path, result: str, reviewer: str, evidence: str,
                completion_claim: str = 'unknown', overwrite: bool = False) -> Path:
    """Validate and save a human judgment without inferring it from proximity.

    :param folder: Trial directory.
    :param result: success, failure, or unknown.
    :param reviewer: Reviewer identity.
    :param evidence: Referenced image/iteration evidence and rationale.
    :param completion_claim: correct, incorrect, or unknown final declaration.
    :param overwrite: Explicitly replace a previous review.
    :return: Saved review path.
    """
    if result not in ('success', 'failure', 'unknown') or completion_claim not in ('correct', 'incorrect', 'unknown'):
        raise ValueError('Invalid review result or completion claim judgment')
    if not reviewer.strip() or not evidence.strip():
        raise ValueError('Reviewer and evidence must be nonempty')
    if completion_claim != 'unknown' and not declared_complete(folder):
        raise ValueError('No final model completion declaration was recorded')
    if (result == 'success' and completion_claim == 'incorrect') or (result == 'failure' and completion_claim == 'correct'):
        raise ValueError('Task success and completion-claim judgments contradict each other')
    value = {'schema_version': 1, 'trial_sha256': trial_digest(folder),
             'semantic_task_success': {'success': True, 'failure': False, 'unknown': None}[result],
             'result': result, 'reviewer': reviewer.strip(), 'evidence': evidence.strip(),
             'completion_claim': completion_claim,
             'false_completion': {'correct': False, 'incorrect': True, 'unknown': None}[completion_claim],
             'reviewed_utc': datetime.now(timezone.utc).isoformat()}
    output = folder / 'review.json'
    with output.open('w' if overwrite else 'x') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')
    return output


class ReviewServer(ThreadingHTTPServer):
    """Serve campaign packets and persist validated judgments to mapped trials."""

    def __init__(self, campaign: Path, port: int) -> None:
        """Bind the review application to localhost.

        :param campaign: Campaign containing generated review packets.
        :param port: HTTP port; zero selects an available port for tests.
        :return: None.
        """
        self.campaign = campaign.resolve()
        self.trials = {trial_digest(path.parent)[:12]: path.parent
                       for path in sorted(self.campaign.rglob('trial.json'))
                       if 'provenance' not in path.relative_to(self.campaign).parts}
        self.token = secrets.token_urlsafe(32)
        self.review_lock = threading.Lock()
        super().__init__(('127.0.0.1', port), ReviewHandler)


class ReviewHandler(BaseHTTPRequestHandler):
    """Handle only review packets and the campaign's mapped review API."""

    def respond(self, status: int, value: dict) -> None:
        """Send a JSON response without caching reviewer data.

        :param status: HTTP status code.
        :param value: JSON response body.
        :return: None.
        """
        payload = json.dumps(value).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self) -> None:
        """Load a review packet or current saved judgment.

        :return: None.
        """
        path = urlsplit(self.path).path
        prefix = '/api/reviews/'
        if path.startswith(prefix):
            folder = self.server.trials.get(path[len(prefix):])
            if folder is None:
                self.respond(404, {'error': 'Unknown trial'})
                return
            self.respond(200, {'trial_sha256': trial_digest(folder), 'token': self.server.token,
                               'declared_complete': declared_complete(folder),
                               'review': load_json(folder / 'review.json')})
            return
        name = 'index.html' if path in ('/', '/index.html') else path.removeprefix('/')
        allowed = {'index.html'} | {code + '.html' for code in self.server.trials}
        if name not in allowed:
            self.respond(404, {'error': 'Unknown review page'})
            return
        payload = (self.server.campaign / 'review_packets' / name).read_bytes()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_POST(self) -> None:
        """Validate and save a judgment using the same rules as the CLI.

        :return: None.
        """
        if not secrets.compare_digest(self.headers.get('X-Review-Token', ''), self.server.token):
            self.respond(403, {'error': 'Invalid review session. Reload the page.'})
            return
        prefix = '/api/reviews/'
        path = urlsplit(self.path).path
        folder = self.server.trials.get(path[len(prefix):]) if path.startswith(prefix) else None
        if folder is None:
            self.respond(404, {'error': 'Unknown trial'})
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 65536:
                raise ValueError('Review must contain at most 64 KiB of text')
            value = json.loads(self.rfile.read(length))
            if not isinstance(value, dict) or any(not isinstance(value.get(field), str)
                                                   for field in ('trial_sha256', 'result', 'reviewer', 'evidence', 'completion_claim')):
                raise ValueError('Review fields must be text')
            if type(value.get('overwrite', False)) is not bool:
                raise ValueError('Replace existing review must be a checkbox value')
            with self.server.review_lock:
                if value['trial_sha256'] != trial_digest(folder):
                    self.respond(409, {'error': 'Trial changed. Reload the packet before reviewing.'})
                    return
                save_review(folder, value['result'], value['reviewer'], value['evidence'],
                            value['completion_claim'], value.get('overwrite', False))
            self.respond(200, {'saved': True})
        except FileExistsError:
            self.respond(409, {'error': 'A judgment already exists. Check Replace to revise it.'})
        except (ValueError, OSError) as error:
            self.respond(400, {'error': str(error)})



def prepare(campaign: Path) -> Path:
    """Generate all partially blinded packets and a portable index.

    :param campaign: Campaign containing trial directories.
    :return: Review index HTML path.
    """
    links = []
    destination = campaign / "review_packets"
    for path in sorted(campaign.rglob("trial.json")):
        if "provenance" in path.relative_to(campaign).parts:
            continue
        code = trial_digest(path.parent)[:12]
        packet(path.parent, destination / f"{code}.html")
        links.append(f'<li><a href="{code}.html">Trial {code}</a></li>')
    if not links:
        raise ValueError("No trial.json files found")
    output = destination / "index.html"
    output.write_text('<!doctype html><meta charset="utf-8"><title>Review packets</title><h1>Review packets</h1><p>Trial identities are hidden; image overlays may reveal the method.</p><ul>' + "".join(links) + '</ul>')
    # Mapping stays outside packets so it is not accidentally sent to reviewers.
    mapping = {trial_digest(path.parent)[:12]: str(path.parent.relative_to(campaign))
               for path in sorted(campaign.rglob("trial.json")) if "provenance" not in path.relative_to(campaign).parts}
    (campaign / "review_packet_mapping.json").write_text(json.dumps(mapping, indent=2) + "\n")
    return output


def main() -> None:
    """Run packet generation or review recording from the command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    batch = sub.add_parser('prepare', help='Create all packets plus partially blinded index')
    batch.add_argument('campaign', type=Path)
    web = sub.add_parser('serve', help='Review in a browser and save judgments directly to trial folders')
    web.add_argument('campaign', type=Path)
    web.add_argument('--port', type=int, default=8765)
    build = sub.add_parser('packet', help='Generate self-contained offline review HTML')
    build.add_argument('trial', type=Path)
    build.add_argument('--output', type=Path)
    build.add_argument('--reveal-method', action='store_true')
    review = sub.add_parser('review', help='Record an independent human judgment')
    review.add_argument('trial', type=Path)
    review.add_argument('--result', choices=['success', 'failure', 'unknown'], required=True)
    review.add_argument('--reviewer', required=True)
    review.add_argument('--evidence', required=True)
    review.add_argument('--completion-claim', choices=['correct', 'incorrect', 'unknown'], default='unknown')
    review.add_argument('--overwrite', action='store_true')
    args = parser.parse_args()
    try:
        if args.command == 'serve':
            prepare(args.campaign)
            with ReviewServer(args.campaign, args.port) as server:
                print(f'Review and save judgments at http://127.0.0.1:{server.server_port}/', flush=True)
                try:
                    server.serve_forever()
                except KeyboardInterrupt:
                    pass
            return
        elif args.command == 'prepare':
            output = prepare(args.campaign)
        elif args.command == 'packet':
            output = packet(args.trial, args.output or args.trial / 'review_packet.html', args.reveal_method)
        else:
            output = save_review(args.trial, args.result, args.reviewer, args.evidence, args.completion_claim, args.overwrite)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(output)


if __name__ == '__main__':
    main()
