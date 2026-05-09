#!/usr/bin/env python3
"""
Motion test: basic sanity checks → constrained planning examples.

Usage:
  ros2 run moveit_robot_control test_motion
  ros2 run moveit_robot_control test_motion -- /path/to/config.yaml

RViz setup:
  Add display: MarkerArray → /constrained_planning_markers
"""
import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_robot_control import MoveitRobot, Visualizer
from moveit_robot_control.utils import pose, plan_exec, demo_box, demo_orientation, demo_plane

CONFIG = os.path.join(
    get_package_share_directory('moveit_robot_control'), 'config', 'franka_fr3.yaml'
)
HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]


def main():
    rclpy.init()
    config = sys.argv[1] if len(sys.argv) > 1 else CONFIG
    robot = MoveitRobot(config)
    viz = Visualizer(robot.node, robot.base_frame)

    print('── Current state ───────────────────────────────')
    print('Joints:', robot.get_joint_positions())
    print('EE pose:', robot.get_ee_pose())

    print('\n── Move to home (joint goal) ───────────────────')
    plan_exec(robot, viz, robot.plan_to_joints(HOME, vel=0.05), pause_after=None)

    print('\n── Move to Cartesian pose ──────────────────────')
    plan_exec(robot, viz,
              robot.plan_to_pose(pose(0.5, 0.0, 0.5, 0.912, 0.380, 0.145, 0.058), vel=0.05),
              pause_after=None)

    print('\n── Cartesian path (3 waypoints) ────────────────')
    plan_exec(robot, viz,
              robot.plan_cartesian([
                  pose(0.5, 0.0, 0.45, 0.912, 0.380, 0.145, 0.058),
                  pose(0.5, 0.1, 0.40, 0.912, 0.380, 0.145, 0.058),
                  pose(0.5, 0.0, 0.40, 0.912, 0.380, 0.145, 0.058),
              ], max_step=0.01, vel=0.05),
              pause_after=None)

    print('\n── Constrained planning examples ───────────────────────────────')
    print('  [1] Box constraint   — EE stays inside a 3D box along the whole path')
    print('  [2] Orientation lock — EE orientation fixed throughout (no tilt)')
    print('  [3] Plane constraint — EE constrained to a horizontal plane (equality)')
    input('\n  Press Enter to go to home and start...')

    robot.move_to_joints(HOME, vel=0.05)
    robot.set_planning_time(10.0)

    demo_box(robot, viz)
    demo_orientation(robot, viz)
    demo_plane(robot, viz)

    print('\nAll examples done.')


if __name__ == '__main__':
    main()
