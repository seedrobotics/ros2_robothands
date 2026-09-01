#!/usr/bin/env python3
"""Every gazebo.launch.py argument combination must resolve.

Nothing is started: `ros2 launch -p` expands the substitutions, evaluates the
conditions and runs xacro, which is where the launch file's PythonExpression
conditions and per-mode controller-file selection can go wrong.
"""
import itertools

from rh8d_test_support import (Checks, check_launch_combinations, main,
                               use_test_domain)

use_test_domain(6)


def run():
    check = Checks('seed_rh8d_gazebo launch files')
    combinations = [
        dict(side=side, finger_coupling=coupling, use_coupling=use_coupling,
             couple_ring_little=couple, driver_interface=driver,
             motor_gui=gui, headless='true')
        for side, coupling, use_coupling, couple, driver, gui
        in itertools.product(['left', 'right'], ['mimic', 'independent'],
                             ['true', 'false'], ['true', 'false'],
                             ['true', 'false'], ['true', 'false'])
    ]
    check_launch_combinations(check, 'seed_rh8d_gazebo', 'gazebo.launch.py',
                              combinations)
    return check.report()


if __name__ == '__main__':
    rc = run()
    raise SystemExit(rc)
