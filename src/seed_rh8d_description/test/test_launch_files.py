#!/usr/bin/env python3
"""Every display.launch.py argument combination must resolve.

Nothing is started: `ros2 launch -p` expands the substitutions, evaluates
the coupling-mode conditions and runs xacro. validate_urdf.sh checks the
model itself; this checks the launch file that serves it.
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


def main():
    print('\n=== seed_rh8d_description launch files ===', flush=True)
    combinations = [
        dict(side=side, finger_coupling=coupling, use_coupling=use_coupling,
             couple_ring_little=couple)
        for side, coupling, use_coupling, couple
        in itertools.product(['left', 'right'], ['mimic', 'independent'],
                             ['true', 'false'], ['true', 'false'])
    ]
    failures = []
    for arguments in combinations:
        ok, detail = resolve('seed_rh8d_description', 'display.launch.py', arguments)
        if not ok:
            failures.append(arguments)
            print(f'  FAIL  display.launch.py {arguments}\n        {detail}',
                  flush=True)
    if failures:
        print(f'  FAIL  display.launch.py: {len(failures)} of '
              f'{len(combinations)} combinations failed', flush=True)
    else:
        print(f'  PASS  display.launch.py: {len(combinations)} argument '
              f'combinations resolve', flush=True)
    print('=' * 70, flush=True)
    print('ALL CHECKS PASSED' if not failures else 'FAILURES', flush=True)
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
