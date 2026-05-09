# 带约束的运动规划

## 核心概念澄清：goal_constraints vs path_constraints

MoveIt 的 `MotionPlanRequest` 中有两个约束字段，语义完全不同：

| 字段 | 作用范围 | 典型用途 |
|---|---|---|
| `goal_constraints[]` | 仅在轨迹终点满足 | "机器人最终到达哪里" |
| `path_constraints` | 轨迹上每个中间点都必须满足 | "运动过程中 EE 必须保持什么性质" |

**这两个字段不能混用。** 最常见的错误是把目标点的紧小球形约束放进 `path_constraints`——这意味着轨迹上每个点都要在目标点 1cm 内，对任何非零位移的运动来说根本不可能满足。

### 原有代码的问题

旧实现中 `_pose_constraints` 构建了一个以目标为圆心、半径 0.01m 的球形约束，然后通过 `set_constraints` 放入 `path_constraints`。规划器要么找不到满足条件的路径失败，要么在内部忽略了这个不合理的约束——两种情况都无法产生真正带约束的轨迹。

---

## 修复后的架构

### 关注点分离

```
constraints.py          ← 纯函数：只构建 Constraints 消息对象
moveit_robot.py         ← 状态管理 + 规划执行
```

`moveit_robot.py` 内部的 `_goal_constraints()` 专门为 `goal_constraints[]` 字段构建约束（终点紧约束），对外不暴露。对外只提供：

```python
robot.set_path_constraints(c: Constraints)   # 设置路径约束
robot.clear_path_constraints()               # 清除路径约束
```

路径约束对象由 `constraints.py` 中的纯函数构建，调用者显式管理约束的生命周期。

---

## Python 接口层与 C++ 的对应关系

官方文档用 C++ 的 `MoveGroupInterface::setPathConstraints()` 设置约束。其底层实现就是填充 `/move_action` 的 `MotionPlanRequest.path_constraints` 字段。Python 直接通过 ROS2 Action Client 发送同一消息，调用的是完全相同的接口，不需要任何 C++ 绑定。

---

## constraints.py 函数说明

### `box(center, dims, frame_id, link_name)`

EE 在整条轨迹上保持在一个长方体区域内。

```python
from geometry_msgs.msg import Pose
from moveit_robot_control import constraints

center = Pose()
center.position.x = 0.4
center.position.y = 0.0
center.position.z = 0.5
center.orientation.w = 1.0

c = constraints.box(center, (0.1, 0.4, 0.4), robot.base_frame, robot.ee_link)
robot.set_path_constraints(c)
robot.move_to_pose(target)
robot.clear_path_constraints()
```

### `plane(center, normal_quat, frame_id, link_name, thickness=0.0005)`

EE 约束在一个平面上运动（OMPL equality constraint）。

`normal_quat` 旋转约束盒子，使其 Y 轴对齐平面法向量。`thickness` 取值范围：

- 必须 > 1e-4（OMPL 内部可行性容差）
- 必须 < 1e-3（OMPL 判定为 equality constraint 的阈值）

0.0005 是官方文档给出的安全值。

```python
import math
from geometry_msgs.msg import Quaternion

# 绕 X 轴旋转 45° 的平面
normal_quat = Quaternion(
    x=math.sin(math.pi / 8),
    y=0.0, z=0.0,
    w=math.cos(math.pi / 8)
)
c = constraints.plane(cur_pose, normal_quat, robot.base_frame, robot.ee_link)
robot.set_path_constraints(c)
robot.move_to_pose(target)
robot.clear_path_constraints()
```

注意：使用此约束时，起点和终点都必须在平面上，否则规划必然失败。

### `line(center, dir_quat, frame_id, link_name, thickness=0.0005)`

EE 约束在一条直线上运动（两个横截面维度均为 equality constraint）。

`dir_quat` 旋转盒子使其 Z 轴对齐直线方向。

```python
# 沿 Z 轴方向的竖直线
from geometry_msgs.msg import Quaternion
dir_quat = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # 不旋转，Z 轴即直线方向

c = constraints.line(cur_pose, dir_quat, robot.base_frame, robot.ee_link)
robot.set_path_constraints(c)
robot.move_to_pose(target)
robot.clear_path_constraints()
```

### `keep_orientation(quat, tol_xyz, frame_id, link_name)`

EE 在整条轨迹上保持固定姿态（如防止容器倾斜洒水）。

```python
cur_pose = robot.get_ee_pose()

c = constraints.keep_orientation(
    cur_pose.orientation,
    (0.05, 0.05, 0.05),       # 各轴姿态容差（弧度）
    robot.base_frame,
    robot.ee_link
)
robot.set_path_constraints(c)
robot.move_to_pose(target)
robot.clear_path_constraints()
```

### `combine(*constraints)`

同时施加多个约束（position + orientation 混合）。

```python
c = constraints.combine(
    constraints.box(center, (1.0, 0.6, 1.0), robot.base_frame, robot.ee_link),
    constraints.keep_orientation(cur_pose.orientation, (0.4, 0.4, 0.4),
                                  robot.base_frame, robot.ee_link)
)
robot.set_path_constraints(c)
robot.move_to_pose(target)
robot.clear_path_constraints()
```

注意：同时施加位置和姿态约束会大幅缩小可达区域，需确保起点和终点都满足两个约束。

---

## OMPL 配置要求

对于 equality constraint（plane/line），必须在规划组的 `ompl_planning.yaml` 中添加：

```yaml
your_planning_group:
  enforce_constrained_state_space: true
  projection_evaluator: joints(joint1,joint2)
```

`projection_evaluator` 用于将高维关节空间投影到低维欧式空间以辅助采样，通常选前两个关节即可。

不添加此配置时，box 约束依然有效（OMPL 会自动切换到约束状态空间），但 plane/line equality 约束不生效。

---

## 规划时间建议

带约束规划比无约束规划慢，建议：

```python
robot.set_planning_time(10.0)   # 默认 5.0 秒，带约束建议 10~30 秒
```

---

## 典型工作流

```python
from moveit_robot_control import MoveitRobot, constraints

robot = MoveitRobot('config/franka_fr3.yaml')
robot.set_planning_time(10.0)

cur_pose = robot.get_ee_pose()

# 1. 构建约束
c = constraints.keep_orientation(
    cur_pose.orientation, (0.05, 0.05, 0.05),
    robot.base_frame, robot.ee_link
)

# 2. 设置约束
robot.set_path_constraints(c)

# 3. 规划并执行（约束在每次规划中自动生效）
robot.move_to_pose(target_a)
robot.move_to_pose(target_b)

# 4. 清除约束（后续规划不再受约束）
robot.clear_path_constraints()
```
