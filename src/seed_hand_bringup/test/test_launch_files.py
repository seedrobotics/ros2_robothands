#!/usr/bin/env python3
"""Every hand.launch.py / view.launch.py argument combination must resolve.

Nothing is started and no hardware is touched: `ros2 launch -p` expands the
substitutions, runs the OpaqueFunction that builds the node list and
evaluates the xacro command, which is where a wrong side, a missing config
or a broken condition shows up.
"""
import itertools
import re
import subprocess
import sys

TROUBLE = re.compile(r'traceback|exception|error|invalid', re.IGNORECASE)


def resolve(package, launch_file, arguments, timeout=240):
    command = (['ros2', 'launch', '-p', package, launch_file]
               + [f'{k}:={v}' for k, v in arguments.items()])
    try:
        result = subprocess.run(command, capture_output=True, text=True,
                                timeout=timeout)
    except subprocess.TimeoutExpired:
        return False, f'timed out after {timeout} s'
    text = (result.stdout or '') + (result.stderr or '')
    if result.returncode != 0 or TROUBLE.search(text):
        lines = [line for line in text.splitlines() if TROUBLE.search(line)]
        return False, ' | '.join(lines[:3]) or f'exit code {result.returncode}'
    return True, ''


def check_all(package, launch_file, combinations):
    failures = []
    for arguments in combinations:
        ok, detail = resolve(package, launch_file, arguments)
        if not ok:
            failures.append(arguments)
            print(f'  FAIL  {launch_file} {arguments}\n        {detail}', flush=True)
    if failures:
        print(f'  FAIL  {launch_file}: {len(failures)} of '
              f'{len(combinations)} combinations failed', flush=True)
    else:
        print(f'  PASS  {launch_file}: {len(combinations)} argument '
              f'combinations resolve', flush=True)
    return not failures


def main():
    print('\n=== seed_hand_bringup launch files ===', flush=True)
    ok = True
    ok &= check_all('seed_hand_bringup', 'hand.launch.py', [
        dict(side=side, use_sensors=sensors, aligned_interface=aligned,
             rviz=rviz, motor_gui=gui)
        for side, sensors, aligned, rviz, gui
        in itertools.product(['left', 'right', 'both'], ['true', 'false'],
                             ['true', 'false'], ['true', 'false'],
                             ['true', 'false'])])
    ok &= check_all('seed_hand_bringup', 'view.launch.py', [
        dict(side=side, gui=gui)
        for side, gui in itertools.product(['left', 'right'], ['true', 'false'])])
    print('=' * 70, flush=True)
    print('ALL CHECKS PASSED' if ok else 'FAILURES', flush=True)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
