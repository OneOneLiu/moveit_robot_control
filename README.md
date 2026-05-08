# moveit_robot_control

Python library for controlling robot arms via MoveIt 2 (ROS 2 Jazzy). Supports joint-space and Cartesian motion planning, Cartesian path execution, FK/IK queries, and collision scene management — configured via a YAML file.

Tested with Franka FR3 and Universal Robots UR5e.

## Requirements

- ROS 2 Jazzy
- `ros-jazzy-moveit` / `ros-jazzy-moveit-py`
- A running `move_group` node (real hardware or `use_fake_hardware:=true`)

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select moveit_robot_control
source install/setup.bash
```

## Configuration

Pre-built configs in `config/`:

| File | Robot |
|------|-------|
| `franka_fr3.yaml` | Franka Research 3 |
| `ur5e.yaml` | Universal Robots UR5e |

Key fields:

```yaml
planning_group: fr3_arm
ee_link: fr3_hand
base_frame: base
joint_names: [fr3_joint1, ..., fr3_joint7]
velocity_scaling: 0.3
acceleration_scaling: 0.3
planner_pipeline: ompl
planner_id: RRTConnect
```

## Usage

### Motion control

```python
from moveit_robot_control import MoveitRobot
from moveit_robot_control.utils import pose

robot = MoveitRobot('config/franka_fr3.yaml')

# Joint goal
robot.move_to_joints([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

# Cartesian pose goal
robot.move_to_pose(pose(0.5, 0.0, 0.5, qw=1.0))

# XYZ only — keeps current orientation
robot.move_to_position(0.45, 0.1, 0.55)

# Cartesian path through waypoints
robot.move_cartesian([pose(0.5, 0.0, 0.45), pose(0.5, 0.1, 0.40)])

# Plan without executing, then execute separately
traj = robot.plan_to_joints([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
robot.execute(traj)

# State queries
robot.get_joint_positions()    # dict {name: radians}
robot.get_ee_pose()            # geometry_msgs/Pose
robot.compute_ik(target_pose)  # dict {name: radians} or None
```

### Planning scene

```python
from moveit_robot_control import PlanningScene
from moveit_robot_control.utils import pose

scene = PlanningScene(base_frame='base', node=robot.node)

scene.add_box('table', pose(0.6, 0.0, 0.0), size=(0.8, 1.2, 0.05))
scene.add_sphere('ball', pose(0.5, 0.0, 0.3), radius=0.05)
scene.add_cylinder('post', pose(0.4, 0.0, 0.2), radius=0.02, height=0.4)

scene.attach_object('ball', link='fr3_hand')
scene.detach_object('ball')
scene.remove_object('table')
scene.clear()

scene.save('/tmp/scene.scene')
scene.load('/tmp/scene.scene')
```

## Smoke test

```bash
# Start MoveIt first, e.g.:
# ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true

ros2 run moveit_robot_control test_motion
ros2 run moveit_robot_control test_scene

# Use a custom config:
ros2 run moveit_robot_control test_motion -- /path/to/config.yaml
```
