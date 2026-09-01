#!/usr/bin/env python3
"""Run the glove's open/fist calibration from the ROS package.

    ros2 run rokoko_glove_control glove_calibrate

Runs rgmp_client_rh8d's four-pose procedure unchanged; this only supplies a
shebang and a default path, so it can be reached with `ros2 run` and writes
where glove_node looks. Needs rkk-hand-solver running with a glove
streaming.

The file lands beside the glove modules in the source tree - resolved
through the symlink a --symlink-install leaves behind - so it survives a
clean rebuild, which a copy under install/ would not.
"""
import argparse
import sys
from pathlib import Path

# Installed, the glove modules sit beside this script; in the source tree
# they are one level up in rokoko_glove_control/. They import each other
# flatly, so the directory holding them has to be on the path.
_HERE = Path(__file__).resolve().parent
for _candidate in (_HERE, _HERE.parent / 'rokoko_glove_control'):
    if str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))

import rgmp_client_rh8d
from hand_joints import Calibration
from rgmp_client_rh8d import DEFAULT_CALIBRATION, SOLVER_PORT, HandReader, run_calibration


def default_path():
    """Where glove_node looks first: beside the real module file."""
    return Path(rgmp_client_rh8d.__file__).resolve().with_name(
        Path(DEFAULT_CALIBRATION).name)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--host', default='127.0.0.1', help='solver host')
    parser.add_argument('--port', type=int, default=SOLVER_PORT,
                        help=f'rkk-hand-solver RGMP port (default {SOLVER_PORT})')
    parser.add_argument('--calibration', type=Path, default=default_path(),
                        help='where to write the calibration')
    args = parser.parse_args()

    try:
        with HandReader(args.host, args.port, Calibration()) as reader:
            if not reader.saw_definition.wait(timeout=10.0):
                if reader.non_solved_device:
                    print(f'{args.host}:{args.port} is streaming '
                          f"'{reader.non_solved_device}', not solved hands. "
                          'Point this at rkk-hand-solver.', file=sys.stderr)
                else:
                    print('No solved hand appeared within 10 s - is a glove '
                          'connected and streaming?', file=sys.stderr)
                return 1
            return run_calibration(reader, args.calibration)
    except OSError as exc:
        print(f'Cannot connect to {args.host}:{args.port} - {exc}\n'
              'Is rkk-hand-solver running? It serves solved hands on '
              f'127.0.0.1:{SOLVER_PORT}.', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
