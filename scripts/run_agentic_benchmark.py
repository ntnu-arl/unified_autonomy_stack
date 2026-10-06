#!/usr/bin/env python3
"""Run paired, isolated agent evaluations using the existing simulation images."""
from __future__ import annotations

import argparse
import copy
import hashlib
import fcntl
import itertools
import math
import json
import os
from pathlib import Path
import re
import socket
import shutil
import signal
import time
import subprocess
from datetime import datetime, timezone
from typing import Any
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'workspaces/robot_bringup/config/evaluation/agentic_benchmark.yaml'
COMPOSE = ROOT / 'docker-compose.uav_nmpc_unipilot_scene_graph_sim.yml'
PROFILE = ROOT / 'workspaces/robot_bringup/config/ros2/agentic_uas_unipilot.yaml'
OWNER_LABEL = 'org.unified-autonomy.benchmark.root'
REPOSITORIES = {
    'stack': ROOT,
    'agent': ROOT / 'workspaces/ws_agentic_uas/src/agentic_uas',
    'bringup': ROOT / 'workspaces/robot_bringup',
    'worlds': ROOT / 'workspaces/ws_sim/src/gz_sim_worlds',
    'planner': ROOT / 'workspaces/ws_gbplanner/src/exploration/gbplanner_ros',
    'pci': ROOT / 'workspaces/ws_gbplanner/src/exploration/pci_general',
    'controller': ROOT / 'workspaces/ws_nmpc/src/sdf-nmpc',
    'controller_ros': ROOT / 'workspaces/ws_nmpc/src/sdf_nmpc_ros',
    'robot_model': ROOT / 'workspaces/ws_sim/src/rmf_gz',
    'semantic_inference': ROOT / 'workspaces/ws_scene_graph/src/semantic_inference',
    'hydra': ROOT / 'workspaces/ws_scene_graph/src/hydra',
    'spark_dsg': ROOT / 'workspaces/ws_scene_graph/src/spark_dsg',
}


def command(args: list[str], **kwargs: Any) -> subprocess.CompletedProcess:
    """Execute without a shell and raise on failure.

    :param args: Command and literal arguments.
    :return: Completed process.
    """
    return subprocess.run(args, check=True, text=True, **kwargs)


def slug(value: str) -> str:
    """Return a filesystem-safe experiment identifier.

    :param value: Model or campaign name.
    :return: Safe identifier.
    """
    return re.sub(r'[^a-zA-Z0-9_.-]+', '-', value).strip('.-') or 'unnamed'


def owned_benchmark(labels: dict[str, str]) -> bool:
    """Identify this checkout's benchmark containers, including older unlabeled runs.

    :param labels: Docker container labels, not its environment or credentials.
    :return: Whether the container belongs to a benchmark from this checkout.
    """
    if not re.fullmatch(r'agentic-eval-\d+-\d+', labels.get('com.docker.compose.project', '')):
        return False
    if OWNER_LABEL in labels:
        return labels[OWNER_LABEL] == str(ROOT)
    directory = labels.get('com.docker.compose.project.working_dir')
    return bool(directory and Path(directory).resolve().is_relative_to(ROOT / 'evaluation_results'))


def cleanup_benchmarks(project: str | None = None) -> None:
    """Stop abandoned containers by labels even if their Compose file was deleted.

    The caller must hold the benchmark lock, or own the named active project.
    Recording containers stop first while the rest of ROS is still available.
    :param project: Limit cleanup to this project, or all owned abandoned projects.
    :return: None; volumes, bind-mounted recordings and unrelated containers survive.
    """
    ids = command(['docker', 'ps', '-aq', '--filter', 'label=com.docker.compose.project'],
                  capture_output=True).stdout.split()
    if not ids:
        return
    metadata = json.loads(command(['docker', 'inspect', *ids], capture_output=True).stdout)
    selected = [item for item in metadata if owned_benchmark(item['Config'].get('Labels') or {})
                and (project is None or item['Config']['Labels']['com.docker.compose.project'] == project)]
    if not selected:
        return
    projects = sorted({item['Config']['Labels']['com.docker.compose.project'] for item in selected})
    print('Cleaning abandoned benchmark containers: ' + ', '.join(projects), flush=True)
    for recording in (True, False):
        running = [item['Id'] for item in selected if item['State']['Running']
                   and (item['Config']['Labels'].get('com.docker.compose.service') == 'evaluation') == recording]
        if running:
            command(['docker', 'stop', '--signal', 'SIGINT', '--timeout', '30', *running],
                    capture_output=True, timeout=60)
    command(['docker', 'rm', *[item['Id'] for item in selected]], capture_output=True, timeout=30)


def failed_service(states: list[dict[str, Any]]) -> dict[str, Any] | None:
    """Find an exited required service, allowing the successful parameter loader.

    :param states: Docker Compose ps JSON records.
    :return: First failed service, or None while startup/execution remains viable.
    """
    for state in states:
        if state.get('Service') == 'evaluation':
            continue  # Its outcome.json and docker wait exit status are handled separately.
        if state.get('Health') == 'unhealthy':
            return state
        if state.get('State') in ('exited', 'dead'):
            if state.get('Service') == 'ros1_launch_bridge_params' and state.get('ExitCode') == 0:
                continue
            return state
    return None


def monitored_command(args: list[str], compose: list[str], env: dict[str, str],
                      folder: Path, timeout: float, log: Any = None) -> subprocess.CompletedProcess:
    """Wait with visible progress and fail promptly when a required container exits.

    :param args: Compose startup or Docker wait command.
    :param compose: Trial-specific Compose invocation prefix.
    :param env: Environment used for this trial.
    :param folder: Trial artifacts, including live metrics.
    :param timeout: Maximum wall seconds for the operation.
    :param log: Optional output log; otherwise capture the small Docker wait result.
    :return: Completed command; failures include the offending service logs.
    """
    process = subprocess.Popen(args, env=env, stdout=log or subprocess.PIPE,
                               stderr=log or subprocess.PIPE, text=True)
    started = last_report = time.monotonic()
    try:
        while process.poll() is None:
            now = time.monotonic()
            if now - started > timeout:
                raise subprocess.TimeoutExpired(args, timeout)
            if now - last_report >= 5:
                output = command([*compose, 'ps', '-a', '--format', 'json'],
                                 env=env, capture_output=True, timeout=15).stdout.strip()
                states = json.loads(output) if output.startswith('[') else [json.loads(line) for line in output.splitlines()]
                failed = failed_service(states)
                if failed:
                    service = failed['Service']
                    logs = command([*compose, 'logs', '--no-color', '--tail', '30', service],
                                   env=env, capture_output=True, timeout=15)
                    print(logs.stdout + logs.stderr, flush=True)
                    raise RuntimeError(f"{service} failed: {failed.get('State')}, exit={failed.get('ExitCode')}, health={failed.get('Health')}")
                metrics_path = folder / 'metrics.json'
                if (folder / 'outcome.json').exists():
                    outcome = json.loads((folder / 'outcome.json').read_text())
                    print(f"[{now-started:.0f}s] Task ended: {outcome.get('outcome')}; flushing recordings and pending model responses...", flush=True)
                elif metrics_path.exists():
                    metrics = json.loads(metrics_path.read_text())
                    missing = [key for key, ready in metrics.get('readiness', {}).items() if not ready]
                    print(f"[{now-started:.0f}s] {metrics.get('phase', 'evaluating')}; "
                          f"task={metrics.get('elapsed_sim_sec', 0):.1f} sim s; "
                          f"agent={metrics.get('agent_state')}; waiting for: {', '.join(missing) or 'none'}", flush=True)
                else:
                    status = ', '.join(f"{s['Service']}={s.get('State')}/{s.get('Health') or '-'}" for s in states)
                    print(f'[{now-started:.0f}s] Starting stack: {status}', flush=True)
                last_report = now
            time.sleep(0.2)
        stdout, stderr = process.communicate()
        if process.returncode:
            if log:
                print('\n'.join((folder / 'runner.log').read_text().splitlines()[-30:]), flush=True)
            raise subprocess.CalledProcessError(process.returncode, args, stdout, stderr)
        return subprocess.CompletedProcess(args, process.returncode, stdout, stderr)
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def write_json(path: Path, value: Any) -> None:
    """Write readable metadata.

    :param path: Destination.
    :param value: JSON-serializable value.
    :return: None.
    """
    path.write_text(json.dumps(value, indent=2) + '\n')


def snapshot_sources(destination: Path) -> dict[str, Any]:
    """Record revisions and local changes needed to reconstruct this campaign.

    :param destination: Campaign provenance directory.
    :return: Revision metadata without credentials.
    """
    destination.mkdir(parents=True)
    result = {}
    for name, repo in REPOSITORIES.items():
        def git(*args: str) -> str:
            return command(['git', '-C', str(repo), *args], capture_output=True).stdout
        revision = {'path': str(repo), 'commit': git('rev-parse', 'HEAD').strip(),
                    'branch': git('branch', '--show-current').strip(),
                    'status': git('status', '--short')}
        (destination / f'{name}.patch').write_text(git('diff', '--binary', 'HEAD'))
        # New implementation files are not represented by git diff.
        for filename in git('ls-files', '--others', '--exclude-standard').splitlines():
            source = repo / filename
            if source.is_file() and source.suffix in {'.py', '.yaml', '.yml', '.xml', '.md', '.txt', '.sh'}:
                target = destination / 'untracked' / name / filename
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(source.read_bytes())
        result[name] = revision
    write_json(destination / 'revisions.json', result)
    return result


class BenchmarkRunner:
    """Create fresh simulation processes and preserve all trial artifacts."""

    def __init__(self, args: argparse.Namespace) -> None:
        """Load the benchmark without starting ROS.

        :param args: Parsed CLI settings.
        :return: None.
        """
        self.args = args
        self.manifest = yaml.safe_load(args.manifest.read_text())
        if self.manifest.get('version') != 1:
            raise ValueError('Only benchmark manifest version 1 is supported')
        self.profile = yaml.safe_load(PROFILE.read_text())
        self.model = args.model or self.profile['model']
        self.max_model_iterations = (args.max_model_iterations if args.max_model_iterations is not None
                                     else self.manifest.get('max_model_iterations', self.profile.get('max_model_iterations', 0)))
        if (type(self.max_model_iterations) is not int or self.max_model_iterations < 0):
            raise ValueError('max_model_iterations must be a nonnegative integer (0 means unlimited)')
        self.campaign = args.output.resolve() / slug(args.campaign)
        self.env = dict(os.environ, DOMAIN_ID=str(args.domain),
                        SCENE_GRAPH_DOMAIN_ID=str(args.domain + 1))

    def trials(self) -> list[dict[str, Any]]:
        """Expand matched world/task/method/repetition settings.

        :return: Paired trial specifications in repetition-major order.
        """
        worlds = self.args.worlds or list(self.manifest['worlds'])
        methods = self.args.methods or self.manifest['methods']
        if not set(methods).issubset(self.manifest['methods']):
            raise ValueError('Unknown method')
        trials = []
        repetitions = self.args.repetitions or self.manifest.get('repetitions', 3)
        for repetition, world_id in itertools.product(range(repetitions), worlds):
            world = self.manifest['worlds'][world_id]
            for task_id in self.args.tasks or list(world['tasks']):
                # Rotate order to reduce systematic warm-cache/order effects.
                for method in methods[repetition % len(methods):] + methods[:repetition % len(methods)]:
                    task = copy.deepcopy(world['tasks'][task_id])
                    if self.args.duration is not None:
                        task['duration_sec'] = self.args.duration
                    trials.append({'version': 1, 'simulation_only': True, 'model': self.model, 'method': method,
                                   'world_id': world_id, 'task_id': task_id,
                                   'seed': self.args.seed + repetition, 'repetition': repetition,
                                   'max_model_iterations': self.max_model_iterations,
                                   'world': {k: v for k, v in world.items() if k != 'tasks'},
                                   'task': task, 'rviz': self.args.rviz,
                                   'visibility': copy.deepcopy(self.manifest.get('visibility', {})),
                                   'metrics': copy.deepcopy(self.manifest.get('metrics', {}))})
        return trials[:self.args.max_runs] if self.args.max_runs else trials

    def compose_config(self, trial: dict[str, Any], folder: Path, project: str) -> dict[str, Any]:
        """Resolve the normal stack and apply isolated benchmark overrides.

        :param trial: Trial specification.
        :param folder: Host artifact folder.
        :param project: Unique Compose project identifier.
        :return: Compose configuration with no saved API key.
        """
        config = json.loads(command(['docker', 'compose', '-f', str(COMPOSE),
                                     '--profile', 'launch', 'config', '--format', 'json'],
                                    cwd=ROOT, env=self.env, capture_output=True).stdout)
        config.pop('name', None)
        config['services'] = {name: svc for name, svc in config['services'].items()
                              if 'launch' in svc.get('profiles', [])}
        for name, svc in config['services'].items():
            svc.pop('profiles', None)
            svc.pop('container_name', None)
            svc['init'] = True
            svc['stop_signal'] = 'SIGINT'
            svc.setdefault('labels', {})[OWNER_LABEL] = str(ROOT)
            environment = svc.setdefault('environment', {})
            environment['ROS_MASTER_URI'] = f'http://127.0.0.1:{self.args.master_port}'
            environment['ROS_HOSTNAME'] = '127.0.0.1'
            environment['GZ_PARTITION'] = project
            # Credentials are expanded only by Compose when launching, never archived.
            for key in list(environment):
                if any(token in key.upper() for token in ('KEY', 'TOKEN', 'PASSWORD', 'SECRET')):
                    environment[key] = '${' + key + ':-}'
            if name.startswith('ros2_'):
                environment['ROS_DOMAIN_ID'] = str(self.args.domain + int(name == 'ros2_launch_scene_graph'))
                environment['FASTDDS_BUILTIN_TRANSPORTS'] = 'UDPv4'
        services = config['services']
        services['ros1_launch_roscore']['command'] = ['roscore', '-p', str(self.args.master_port)]
        # Normal planner wait-for-sensors logic remains intact.
        planner = services['ros1_launch_gbplanner']['command']
        planner[-1] += f" rviz:={'true' if self.args.rviz else 'false'}"
        world = trial['world']
        x, y, z, yaw = world['start']
        services['ros2_launch_uav_sim']['command'] = [
            'ros2', 'launch', 'robot_bringup', 'uav_sim_unipilot.launch.xml',
            'world_package:=gz_sim_worlds', f"world:={Path(world['world']).stem}",
            'world_file:=/evaluation/run/world.sdf',
            f"world_name:={world['world_name']}", f'x:={x}', f'y:={y}', f'z:={z}',
            f'Y:={yaw}', f"gz_extra_args:=--seed {trial['seed']}", 'verbosity:=2']
        mount = {'type': 'bind', 'source': str(folder), 'target': '/evaluation/run'}
        services['ros2_launch_uav_sim']['volumes'].append(dict(mount, read_only=True))
        agent = services['ros2_launch_agentic_uas']
        agent['volumes'].append(mount)
        agent['command'] = ['ros2', 'launch', 'robot_bringup', 'agentic_uas_unipilot.launch.py',
                            'config_path:=/evaluation/run/agent.yaml',
                            'run_directory:=/evaluation/run', 'log_level:=info']
        services['evaluation'] = {
            'image': agent['image'], 'network_mode': 'host', 'ipc': 'host', 'init': True,
            'labels': {OWNER_LABEL: str(ROOT)},
            'working_dir': '/workspace', 'environment': {
                'ROS_DOMAIN_ID': str(self.args.domain), 'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4'},
            'volumes': [v for v in agent['volumes'] if v.get('target') in
                        {'/workspace', '/workspace/src/robot_bringup', '/evaluation/run'}],
            'command': ['ros2', 'run', 'agentic_uas', 'benchmark_node', '--ros-args',
                        '-p', 'trial_file:=/evaluation/run/trial.json',
                        '-p', 'run_directory:=/evaluation/run', '-p', 'use_sim_time:=true'],
            'depends_on': {'ros2_launch_agentic_uas': {'condition': 'service_started'}}}
        # Reuse the model cache rather than downloading once per trial project.
        for volume in config.get('volumes', {}).values():
            volume['external'] = True
            volume.pop('labels', None)
        return config

    def trial_folder(self, trial: dict[str, Any]) -> Path:
        """Resolve the canonical trial directory.

        :param trial: Trial specification.
        :return: Directory containing this trial's artifacts.
        """
        return self.campaign / slug(self.model) / trial['world_id'] / trial['task_id'] / trial['method'] / f"seed-{trial['seed']}"

    def resume_outcome(self, trial: dict[str, Any]) -> dict[str, Any] | None:
        """Validate existing settings and identify trials that need no rerun.

        :param trial: Requested trial specification.
        :return: Successful recorded outcome, or None for absent/unfinished/failed trials.
        """
        folder = self.trial_folder(trial)
        if not folder.exists():
            return None
        if (folder / 'trial.json').exists():
            saved = json.loads((folder / 'trial.json').read_text())
            for key in ('model', 'method', 'world_id', 'task_id', 'seed', 'world', 'task', 'max_model_iterations'):
                if saved.get(key) != trial.get(key):
                    raise ValueError(f'Resume settings differ for {key} in {folder}; use the original arguments or a new campaign')
        if (folder / 'agent.yaml').exists():
            expected = copy.deepcopy(self.profile)
            expected.update(model=self.model, method={'type': trial['method']},
                            max_model_iterations=trial['max_model_iterations'])
            if yaml.safe_load((folder / 'agent.yaml').read_text()) != expected:
                raise ValueError(f'Agent configuration changed in {folder}; restore it or use a new campaign')
        outcome_path = folder / 'runner_outcome.json'
        if not outcome_path.exists():
            outcome_path = folder / 'outcome.json'
        if not outcome_path.exists():
            return None
        outcome = json.loads(outcome_path.read_text())
        if (outcome.get('outcome') in ('model_completed', 'time_budget', 'iteration_budget')
                and outcome.get('container_exit_code') == 0):
            return dict(trial_id=str(folder.relative_to(self.campaign)), **outcome)
        return None

    def archive_partial(self, trial: dict[str, Any]) -> None:
        """Preserve a failed attempt outside the campaign before rerunning it.

        :param trial: Trial to restart from a fresh simulation.
        :return: None.
        """
        folder = self.trial_folder(trial)
        if folder.exists():
            archive = (self.campaign.parent / '.interrupted' / self.campaign.name /
                       datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') /
                       folder.relative_to(self.campaign))
            archive.parent.mkdir(parents=True, exist_ok=True)
            folder.rename(archive)
            print(f'Archived incomplete attempt: {archive}', flush=True)

    def run(self, trial: dict[str, Any], index: int) -> dict[str, Any]:
        """Run a single trial and always tear down only its own project.

        :param trial: Frozen trial settings.
        :param index: Campaign trial index.
        :return: Outcome metadata.
        """
        if shutil.disk_usage(self.campaign).free < 3 * 1024 ** 3:
            raise RuntimeError('Less than 3 GiB free: refusing another bag recording; existing results preserved')
        folder = self.trial_folder(trial)
        if folder.exists():
            raise FileExistsError(f'Refusing to mix trials in {folder}; use a new --campaign')
        folder.mkdir(parents=True)
        project = f'agentic-eval-{os.getpid()}-{index}'
        print(f'Preparing trial; logs: {folder}', flush=True)
        trial['project'] = project
        trial['runtime'] = copy.deepcopy(self.manifest.get('runtime', {}))
        trial['runtime'].setdefault('wall_timeout_sec', 300 + float(trial['task']['duration_sec']) * 5)
        trial['started_utc'] = datetime.now(timezone.utc).isoformat()
        trial['source_provenance'] = str(self.provenance)
        world_path = REPOSITORIES['worlds'] / 'worlds' / trial['world']['world']
        trial['world_sha256'] = hashlib.sha256(world_path.read_bytes()).hexdigest()
        world_tree = ET.parse(world_path)
        step = world_tree.find('./world/physics/max_step_size')
        if step is None:
            raise ValueError(f'World has no explicit physics step: {world_path}')
        trial['original_physics_step_sec'] = float(step.text)
        trial['physics_step_sec'] = self.manifest.get('physics_step_sec', 0.001)
        step.text = str(trial['physics_step_sec'])
        world_tree.write(folder / 'world.sdf', encoding='utf-8', xml_declaration=True)
        trial['evaluated_world_sha256'] = hashlib.sha256((folder / 'world.sdf').read_bytes()).hexdigest()
        # Evaluator-only meshes never enter the agent configuration or task prompt.
        from ground_truth_assets import extract_task
        extract_task(folder / 'world.sdf', REPOSITORIES['worlds'] / 'models',
                     trial['task'].get('targets', []), folder)
        write_json(folder / 'trial.json', trial)
        profile = copy.deepcopy(self.profile)
        profile['model'] = self.model
        profile['max_model_iterations'] = trial['max_model_iterations']
        # Methods use their registered defaults; all other settings are held fixed.
        profile['method'] = {'type': trial['method']}
        (folder / 'agent.yaml').write_text(yaml.safe_dump(profile, sort_keys=False))
        config = self.compose_config(trial, folder, project)
        for volume in config.get('volumes', {}).values():
            if volume.get('external') and volume.get('name'):
                command(['docker', 'volume', 'create', volume['name']], capture_output=True)
        compose_file = folder / 'compose.yaml'
        compose_file.write_text(yaml.safe_dump(config, sort_keys=False))
        images = sorted({svc['image'] for svc in config['services'].values()})
        write_json(folder / 'images.json', json.loads(command(
            ['docker', 'image', 'inspect', *images], capture_output=True).stdout))
        compose = ['docker', 'compose', '-p', project, '-f', str(compose_file)]
        log_process = None
        outcome = {'outcome': 'infrastructure_error'}
        def interrupt(signum: int, frame: Any) -> None:
            raise KeyboardInterrupt
        handlers = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
        with (folder / 'runner.log').open('w') as log, (folder / 'containers.log').open('w') as container_log:
            try:
                print('Starting containers; checking service health every 5 seconds...', flush=True)
                monitored_command([*compose, 'up', '-d'], compose, self.env, folder, 420, log)
                log_process = subprocess.Popen([*compose, 'logs', '--no-color', '-f'],
                                               env=self.env, stdout=container_log, stderr=subprocess.STDOUT)
                evaluator = command([*compose, 'ps', '-a', '-q', 'evaluation'], env=self.env, capture_output=True).stdout.strip()
                # Bound wall time even if simulated time stops advancing.
                timeout = float(trial['runtime']['wall_timeout_sec']) + 90
                print('Containers started; waiting for sensors, planner services and takeoff...', flush=True)
                completed = monitored_command(['docker', 'wait', evaluator], compose, self.env, folder, timeout)
                outcome_path = folder / 'outcome.json'
                outcome = json.loads(outcome_path.read_text()) if outcome_path.exists() else {
                    'outcome': 'infrastructure_error', 'error': 'Evaluator exited without outcome.json'}
                outcome['container_exit_code'] = int(completed.stdout.strip())
            except (subprocess.SubprocessError, OSError, RuntimeError) as error:
                outcome = {'outcome': 'infrastructure_error', 'error': str(error)}
                print(f'Benchmark failed: {error}. Logs: {folder}', flush=True)
            except KeyboardInterrupt:
                outcome = {'outcome': 'interrupted', 'error': 'Benchmark interrupted; shutting down its containers'}
            finally:
                print('Stopping trial containers and preserving recordings...', flush=True)
                # A second Ctrl-C must not leave half of the project running.
                for sig in handlers:
                    signal.signal(sig, signal.SIG_IGN)
                # SIGINT gives rosbag time to close its database and metadata.
                for cleanup, limit in [
                    ([*compose, 'kill', '-s', 'SIGINT', 'evaluation'], 20),
                    ([*compose, 'stop', '-t', '30', 'evaluation'], 45),
                    ([*compose, 'down', '--timeout', '20', '--remove-orphans'], 90),
                ]:
                    try:
                        subprocess.run(cleanup, env=self.env, stdout=log,
                                       stderr=subprocess.STDOUT, timeout=limit, check=False)
                    except subprocess.SubprocessError as error:
                        log.write(f'Cleanup failed: {error}\n')
                try:
                    cleanup_benchmarks(project)
                except (subprocess.SubprocessError, OSError) as error:
                    log.write(f'Container fallback cleanup failed: {error}\n')
                if log_process:
                    log_process.terminate()
                    try:
                        log_process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        log_process.kill()
                        log_process.wait()
                for sig, handler in handlers.items():
                    signal.signal(sig, handler)
        write_json(folder / 'runner_outcome.json', outcome)
        return dict(trial_id=str(folder.relative_to(self.campaign)), **outcome)


def main() -> None:
    """Parse filters and execute a campaign.

    :return: None.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=MANIFEST)
    parser.add_argument('--output', type=Path, default=ROOT / 'evaluation_results')
    parser.add_argument('--campaign', default=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    parser.add_argument('--methods', nargs='+')
    parser.add_argument('--worlds', nargs='+')
    parser.add_argument('--tasks', nargs='+')
    parser.add_argument('--model')
    parser.add_argument('--repetitions', type=int)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--duration', type=float, help='Override simulated seconds per task (pilot runs)')
    parser.add_argument('--max-model-iterations', type=int,
                        help='Decision calls per task, including prefetch/errors; 0 means unlimited')
    parser.add_argument('--max-runs', type=int)
    parser.add_argument('--domain', type=int, default=218)
    parser.add_argument('--master-port', type=int, default=11321)
    parser.add_argument('--rviz', action='store_true',
                        help='Show the ROS 1 planner RViz during each trial (requires a working X display)')
    parser.add_argument('--resume', action='store_true',
                        help='Skip completed campaign trials and archive/restart failed or unfinished trials')
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--stop-existing', action='store_true',
                        help='Stop abandoned benchmark containers from this checkout; do not launch a trial')
    args = parser.parse_args()
    for name in ('repetitions', 'max_runs', 'duration'):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error(f'--{name.replace("_", "-")} must be positive')
    if args.max_model_iterations is not None and args.max_model_iterations < 0:
        parser.error('--max-model-iterations must be nonnegative (0 means unlimited)')
    if not 0 <= args.domain <= 231 or not 1024 <= args.master_port <= 65535:
        parser.error('Use ROS domain 0..231 and an unprivileged master port')
    if not args.stop_existing:
        runner = BenchmarkRunner(args)
        trials = runner.trials()
        print(f'{len(trials)} trials; model={runner.model}; campaign={runner.campaign}', flush=True)
        completed_trials = {}
        if args.resume:
            if not runner.campaign.is_dir():
                parser.error('--resume requires an existing campaign')
            for index, trial in enumerate(trials):
                outcome = runner.resume_outcome(trial)
                if outcome is not None:
                    completed_trials[index] = outcome
            print(f'Resume: {len(completed_trials)} completed; {len(trials) - len(completed_trials)} to run', flush=True)
        elif runner.campaign.exists():
            parser.error('Campaign already exists; use --resume or a new --campaign')
        if args.list or args.dry_run:
            for trial in trials:
                action = (' [completed; skip]' if runner.resume_outcome(trial) else ' [run]') if args.resume else ''
                print(f"{trial['world_id']}/{trial['task_id']}/{trial['method']}/seed-{trial['seed']}{action}")
            return
    lock = open('/tmp/agentic-benchmark.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('A benchmark runner is still active. Stop it with Ctrl-C in its terminal before cleanup.')
    cleanup_benchmarks()
    if args.stop_existing:
        return
    planner_config = REPOSITORIES['bringup'] / 'config/ros1/gbplanner/uav_nmpc_unipilot_sim'
    for name in ('gbplanner_config.yaml', 'voxblox_config.yaml', 'planner_control_interface_config.yaml', 'manhole_detector.yaml'):
        if not (planner_config / name).is_file():
            raise SystemExit(f'Missing required Unipilot planner configuration: {planner_config / name}. No trial was launched.')
    if not os.environ.get('OPENAI_API_KEY'):
        raise SystemExit('OPENAI_API_KEY must be set; no model calls were made')
    # ZMQ endpoints currently belong to the stack and cannot share concurrent runs.
    for port in (args.master_port, 8002, 8003, 8004):
        with socket.socket() as check:
            if check.connect_ex(('127.0.0.1', port)) == 0:
                raise SystemExit(f'Port {port} is occupied by another process or stack. '
                                 'Owned abandoned benchmark containers have been cleaned. '
                                 f"Inspect with: ss -ltnp 'sport = :{port}'")
    command(['docker', 'run', '--rm', '-v',
             str(ROOT / 'workspaces/ws_agentic_uas') + ':/workspace',
             'unified_autonomy:ros2_agentic_uas', 'python3', '-c',
             'import agentic_uas.evaluation.runtime; import agentic_uas.evaluation.replay'])
    runner.campaign.mkdir(parents=True, exist_ok=args.resume)
    provenance = (runner.campaign / 'resume_provenance' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
                  if args.resume else runner.campaign / 'provenance')
    runner.provenance = provenance
    snapshot_sources(provenance)
    write_json(provenance.parent / 'host.json' if args.resume else runner.campaign / 'host.json', {'cpu_count': os.cpu_count(),
        'docker_version': command(['docker', 'version', '--format', '{{.Server.Version}}'], capture_output=True).stdout.strip()})
    if not args.resume:
        (runner.campaign / 'benchmark.yaml').write_bytes(args.manifest.read_bytes())
    outcomes = (json.loads((runner.campaign / 'outcomes.json').read_text())
                if args.resume and (runner.campaign / 'outcomes.json').exists() else [])
    for index, trial in enumerate(trials):
        print(f"[{index + 1}/{len(trials)}] {trial['world_id']}/{trial['task_id']}/{trial['method']}", flush=True)
        if index in completed_trials:
            print('Already completed; skipping', flush=True)
            result = completed_trials[index]
        else:
            if args.resume:
                runner.archive_partial(trial)
            result = runner.run(trial, index)
        outcomes = [outcome for outcome in outcomes if outcome.get('trial_id') != result['trial_id']]
        outcomes.append(result)
        write_json(runner.campaign / 'outcomes.json', outcomes)
        print(outcomes[-1], flush=True)
        if outcomes[-1]['outcome'] == 'interrupted':
            raise SystemExit(130)
        if outcomes[-1]['outcome'] == 'storage_limit':
            raise SystemExit('Storage limit reached; free disk space and rerun with --resume')


if __name__ == '__main__':
    main()
