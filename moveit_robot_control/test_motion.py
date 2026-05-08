#!/usr/bin/env python3
"""
Motion test: joints → Cartesian pose → keep-orientation position move → Cartesian path.
Usage:
  ros2 run moveit_robot_control test_motion
  ros2 run moveit_robot_control test_motion -- /path/to/config.yaml
"""
import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_robot_control import MoveitRobot
from moveit_robot_control.utils import pose

CONFIG = os.path.join(get_package_share_directory('moveit_robot_control'), 'config', 'franka_fr3.yaml')
HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]


def main():
    rclpy.init()
    config = sys.argv[1] if len(sys.argv) > 1 else CONFIG
    robot = MoveitRobot(config)

    print('── Current state ──────────────────────────────')
    print('Joints:', robot.get_joint_positions())
    print('EE pose:', robot.get_ee_pose())

    print('\n── Move to home (joint goal) ──────────────────')
    ok = robot.move_to_joints(HOME, vel=0.05)
    print('Success:', ok)

    print('\n── Move to Cartesian pose ──────────────────────')
    target = pose(0.5, 0.0, 0.5, 0.912, 0.380, 0.145, 0.058)
    ok = robot.move_to_pose(target, vel=0.05)
    print('Success:', ok)

    print('\n── Keep orientation, move to new position ──────')
    ok = robot.move_to_position(0.45, 0.1, 0.55, vel=0.05)
    print('Success:', ok)

    print('\n── Keep x, move y and z ────────────────────────')
    ok = robot.move_to(y=0.05, z=0.50, vel=0.05)
    print('Success:', ok)

    print('\n── Keep x and y, move only z ───────────────────')
    ok = robot.move_to(z=0.45, vel=0.05)
    print('Success:', ok)

    print('\n── Specify all axes (orientation always kept) ──')
    ok = robot.move_to(x=0.48, y=0.02, z=0.48, vel=0.05)
    print('Success:', ok)

    print('\n── Cartesian path (3 waypoints) ────────────────')
    wp1 = pose(0.5, 0.0, 0.45, 0.912, 0.380, 0.145, 0.058)
    wp2 = pose(0.5, 0.1, 0.40, 0.912, 0.380, 0.145, 0.058)
    wp3 = pose(0.5, 0.0, 0.40, 0.912, 0.380, 0.145, 0.058)
    ok = robot.move_cartesian([wp1, wp2, wp3], max_step=0.01, vel=0.05)
    print('Success:', ok)

    print('\n── Plan only (no execute) ──────────────────────')
    traj = robot.plan_to_joints(HOME)
    if traj:
        print(f'Planned {len(traj.joint_trajectory.points)} trajectory points')
        print('Executing planned trajectory...')
        ok = robot.execute(traj)
        print('Execute success:', ok)
    else:
        print('Planning failed')


if __name__ == '__main__':
    main()
