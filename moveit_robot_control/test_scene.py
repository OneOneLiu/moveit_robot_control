#!/usr/bin/env python3
"""
Planning scene test: add/remove objects, attach/detach, save/load.
Usage:
  ros2 run moveit_robot_control test_scene
  ros2 run moveit_robot_control test_scene -- /path/to/config.yaml
"""
import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_robot_control import MoveitRobot, PlanningScene
from moveit_robot_control.utils import pose

CONFIG = os.path.join(get_package_share_directory('moveit_robot_control'), 'config', 'franka_fr3.yaml')
HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]


def main():
    rclpy.init()
    config = sys.argv[1] if len(sys.argv) > 1 else CONFIG
    robot = MoveitRobot(config)
    scene = PlanningScene(base_frame=robot.base_frame, node=robot.node)

    print('── Add table and wall ──────────────────────────')
    scene.add_box('table', pose(0.6, 0.0, -0.025), (1.2, 0.8, 0.05))
    scene.add_box('wall', pose(0.85, 0.0, 0.3), (0.05, 1.2, 0.6))
    print('Objects:', scene.list_objects())

    print('\n── Move home (with obstacles in scene) ─────────')
    ok = robot.move_to_joints(HOME, vel=0.2)
    print('Success:', ok)

    print('\n── Save scene ──────────────────────────────────')
    ok = scene.save('/tmp/test_scene.scene')
    print('Saved:', ok)

    print('\n── Remove wall, add sphere ─────────────────────')
    scene.remove_object('wall')
    scene.add_sphere('obstacle', pose(0.5, 0.3, 0.3), 0.08)
    print('Objects:', scene.list_objects())

    target = pose(0.4, 0.0, 0.4, 0.912, 0.380, 0.145, 0.058)
    ok = robot.move_to_pose(target, vel=0.2)
    print('Move to pose success:', ok)

    print('\n── Attach/detach object ────────────────────────')
    scene.add_box('payload', pose(0.4, 0.0, 0.35), (0.05, 0.05, 0.05))
    # touch_links: all gripper links that are allowed to be in contact with the payload
    gripper_links = [robot.ee_link, 'fr3_leftfinger', 'fr3_rightfinger']
    scene.attach_object('payload', robot.ee_link, touch_links=gripper_links)
    print('Attached. Objects:', scene.list_objects())
    ok = robot.move_to_joints(HOME, vel=0.1)
    print('Move with payload:', ok)
    scene.detach_object('payload')
    print('Detached.')

    print('\n── Clear scene ─────────────────────────────────')
    scene.clear()
    print('Objects after clear:', scene.list_objects())


if __name__ == '__main__':
    main()
