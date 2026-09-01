#!/usr/bin/env python3
"""Shared helpers for the RH8D integration tests.

These tests drive the real nodes over real topics: each node is started as a
subprocess the way a launch file starts it, talked to with rclpy, and judged
by what comes back. Nothing is mocked, so a passing test means the node
works as installed.

Every test file is also runnable on its own:

    ros2 run ... # nothing to set up, just:
    python3 src/seed_rh8d_gazebo/test/test_driver_interface.py
"""
import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node

# Tests get their own DDS domain so a `colcon test` run (which builds and
# tests packages in parallel) cannot have two of them talking to each other,
# and so they never disturb a hand or simulation already running on the
# default domain.
DOMAIN_BASE = int(os.environ.get('RH8D_TEST_DOMAIN_BASE', '88'))


def use_test_domain(offset):
    """Pin this test to its own ROS domain. Call before rclpy.init()."""
    domain = DOMAIN_BASE + offset
    if domain > 101:
        raise ValueError(f'ROS domain {domain} out of the portable range; '
                         'lower RH8D_TEST_DOMAIN_BASE')
    # set in the environment, not just for us: every node we spawn inherits it
    os.environ['ROS_DOMAIN_ID'] = str(domain)


class Checks:
    """Collects results and prints them as the test runs."""

    def __init__(self, title=''):
        self.failures = []
        if title:
            print(f'\n=== {title} ===', flush=True)

    def __call__(self, ok, message):
        print(('  PASS  ' if ok else '  FAIL  ') + message, flush=True)
        if not ok:
            self.failures.append(message)
        return bool(ok)

    def info(self, message):
        print('  info  ' + message, flush=True)

    def section(self, title):
        print(f'\n--- {title} ---', flush=True)

    def report(self):
        """Print the summary and return a process exit code."""
        print('=' * 70, flush=True)
        if self.failures:
            print(f'{len(self.failures)} FAILURE(S):', flush=True)
            for f in self.failures:
                print('  - ' + f, flush=True)
            return 1
        print('ALL CHECKS PASSED', flush=True)
        return 0


# ── subprocess handling ─────────────────────────────────────────────────────

def _ros_args(params, remaps):
    args = []
    for key, value in (params or {}).items():
        args += ['-p', f'{key}:={value}']
    for key, value in (remaps or {}).items():
        args += ['-r', f'{key}:={value}']
    return ['--ros-args'] + args if args else []


def ros_run(package, executable, params=None, remaps=None, node_name=None):
    """`ros2 run` a node in its own process group, capturing its output."""
    remaps = dict(remaps or {})
    if node_name:
        remaps['__node'] = node_name
    cmd = ['ros2', 'run', package, executable] + _ros_args(params, remaps)
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)


def terminate(proc, timeout=10.0):
    """Stop a process and everything it spawned; return its captured output.

    Signals the whole process group, so a launch file's children (gz sim in
    particular) go down with it - a surviving gz server would talk to the
    next test over gz transport, which ROS_DOMAIN_ID does not isolate.
    """
    if proc is None:
        return ''
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return _drain(proc)

    _signal_group(pgid, signal.SIGINT)
    try:
        out = proc.communicate(timeout=timeout)[0] or ''
    except subprocess.TimeoutExpired:
        _signal_group(pgid, signal.SIGKILL)
        out = _drain(proc)
    # the launcher can exit before its children do
    if not _await_group_exit(pgid, 10.0):
        _signal_group(pgid, signal.SIGKILL)
        _await_group_exit(pgid, 5.0)
    return out


def _drain(proc):
    try:
        return proc.communicate(timeout=10)[0] or ''
    except subprocess.TimeoutExpired:
        proc.kill()
        return proc.communicate()[0] or ''


def _signal_group(pgid, sig):
    try:
        os.killpg(pgid, sig)
    except (ProcessLookupError, PermissionError):
        pass


def _await_group_exit(pgid, timeout):
    """Wait until no process is left in the group. True if it emptied."""
    end = time.time() + timeout
    while time.time() < end:
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            return True
        except PermissionError:
            return True
        time.sleep(0.2)
    return False


# ── probing nodes ───────────────────────────────────────────────────────────

class Probe(Node):
    """A test node with the spin/wait helpers every test here needs."""

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_until(self, predicate, timeout, what=''):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        if what:
            print(f'  (timed out after {timeout:.0f} s waiting for {what})',
                  flush=True)
        return False


# ── the simulation ──────────────────────────────────────────────────────────

class Simulation:
    """gazebo.launch.py as a context manager, with guaranteed teardown.

        with Simulation(side='right', coupling='mimic') as sim:
            ...
    """

    def __init__(self, side='right', coupling='mimic', **launch_args):
        self.side = side
        self.coupling = coupling
        self.args = {'side': side, 'finger_coupling': coupling,
                     'headless': 'true'}
        self.args.update({k: str(v) for k, v in launch_args.items()})
        self.joint_prefix = 'l_' if side == 'left' else 'r_'
        self.topic_prefix = 'L_' if side == 'left' else 'R_'
        self.proc = None
        self.log_path = None
        self._log = None

    def __enter__(self):
        fd, self.log_path = tempfile.mkstemp(prefix='rh8d_sim_', suffix='.log')
        self._log = os.fdopen(fd, 'w')
        cmd = ['ros2', 'launch', 'seed_rh8d_gazebo', 'gazebo.launch.py'] + \
              [f'{k}:={v}' for k, v in self.args.items()]
        print(f'  starting: {" ".join(cmd[2:])}', flush=True)
        self.proc = subprocess.Popen(cmd, stdout=self._log,
                                     stderr=subprocess.STDOUT,
                                     start_new_session=True)
        return self

    def __exit__(self, *exc_info):
        terminate(self.proc, timeout=25.0)
        if self._log:
            self._log.close()
        return False

    def dump_log(self, lines=25):
        """Print the tail of the simulation log - for diagnosing a failure."""
        try:
            with open(self.log_path) as f:
                tail = f.read().splitlines()[-lines:]
        except OSError:
            return
        print('  --- simulation log tail ---', flush=True)
        for line in tail:
            print('  | ' + line[:200], flush=True)


def gazebo_available():
    """True when the optional simulation dependencies are installed."""
    for package in ('ros_gz_sim', 'gz_ros2_control'):
        r = subprocess.run(['ros2', 'pkg', 'prefix', package],
                           capture_output=True)
        if r.returncode != 0:
            return False
    return True


def require_gazebo():
    """Exit cleanly with a clear message when the sim cannot run here."""
    if not gazebo_available():
        print('SKIP: ros_gz_sim / gz_ros2_control are not installed - this '
              'test needs the optional simulation dependencies.', flush=True)
        sys.exit(0)


def main(test_function):
    """Run a test function, always shutting rclpy down, and exit with its code."""
    rclpy.init()
    try:
        code = test_function()
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    sys.exit(code)


# ── launch files ────────────────────────────────────────────────────────────

_LAUNCH_TROUBLE = None


def resolve_launch(package, launch_file, arguments, timeout=240):
    """Resolve a launch description without starting anything.

    `ros2 launch -p` expands every substitution and condition and runs the
    xacro commands, so a bad PythonExpression, a wrong path or a broken
    argument combination fails here instead of at run time.
    """
    global _LAUNCH_TROUBLE
    import re
    if _LAUNCH_TROUBLE is None:
        _LAUNCH_TROUBLE = re.compile(r'traceback|exception|error|invalid',
                                     re.IGNORECASE)
    command = (['ros2', 'launch', '-p', package, launch_file]
               + [f'{k}:={v}' for k, v in arguments.items()])
    try:
        result = subprocess.run(command, capture_output=True, text=True,
                                timeout=timeout)
    except subprocess.TimeoutExpired:
        return False, f'timed out after {timeout} s'
    text = (result.stdout or '') + (result.stderr or '')
    if result.returncode != 0 or _LAUNCH_TROUBLE.search(text):
        trouble = [line for line in text.splitlines()
                   if _LAUNCH_TROUBLE.search(line)]
        return False, ' | '.join(trouble[:3]) or f'exit code {result.returncode}'
    return True, ''


def check_launch_combinations(check, package, launch_file, combinations):
    """Resolve every argument combination, reporting one result per file."""
    failures = []
    for arguments in combinations:
        ok, detail = resolve_launch(package, launch_file, arguments)
        if not ok:
            failures.append((arguments, detail))
            print(f'    {launch_file} {arguments}\n      {detail}', flush=True)
    check(not failures,
          f'{launch_file}: {len(combinations)} argument combinations resolve'
          + (f', {len(failures)} failed' if failures else ''))
