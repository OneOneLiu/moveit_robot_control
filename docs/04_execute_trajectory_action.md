# ExecuteTrajectory Action

Action 类型：`moveit_msgs/action/ExecuteTrajectory`
Action 服务器：`/execute_trajectory`

---

## 定位

**只执行，不规划。** 接收一条已经算好的轨迹并下发给控制器执行。

与 MoveGroup action 的分工：
- MoveGroup：规划 + 执行（两步合一）
- ExecuteTrajectory：只执行（轨迹来自外部）

典型来源：
- `MoveGroup`（`plan_only: true`）规划出的 `planned_trajectory`
- `GetCartesianPath` service 计算出的笛卡尔轨迹
- 自定义算法生成的轨迹

---

## Goal 字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `trajectory` | RobotTrajectory | **必填**，要执行的轨迹 |
| `controller_names[]` | string[] | 可选，指定用哪个控制器执行；留空则 MoveIt 自动选择 |

**RobotTrajectory 结构：**
```
trajectory:
  joint_trajectory:               # 关节空间轨迹（最常用）
    joint_names: [...]
    points:
      - positions: [...]
        velocities: [...]
        accelerations: [...]
        time_from_start: {sec: 1, nanosec: 0}
  multi_dof_joint_trajectory:     # 多自由度关节轨迹（移动底盘等）
    ...
```

---

## Result 字段

| 字段 | 说明 |
|------|------|
| `error_code.val` | `1`=SUCCESS，`-4`=CONTROL_FAILED（执行失败），`-7`=PREEMPTED（被抢占） |
| `error_code.message` | 错误描述 |

---

## Feedback 字段

| 字段 | 说明 |
|------|------|
| `state` | 当前状态字符串，如 `IDLE`、`RUNNING` |

---

## 典型使用流程

```
第一步：用 MoveGroup（plan_only=true）规划轨迹
         ↓ 从 result.planned_trajectory 取出轨迹
第二步：用 ExecuteTrajectory 执行
```

**第一步：规划（不执行）**

```bash
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "{
  request: {
    group_name: 'fr3_arm',
    num_planning_attempts: 5,
    allowed_planning_time: 5.0,
    max_velocity_scaling_factor: 0.1,
    max_acceleration_scaling_factor: 0.1,
    goal_constraints: [{
      position_constraints: [{
        header: {frame_id: 'base'},
        link_name: 'fr3_hand',
        constraint_region: {
          primitives: [{type: 2, dimensions: [0.005]}],
          primitive_poses: [{
            position: {x: 0.5, y: 0.0, z: 0.5},
            orientation: {w: 1.0}
          }]
        },
        weight: 1.0
      }]
    }]
  },
  planning_options: {plan_only: true, planning_scene_diff: {is_diff: true}}
}"
```

**第二步：执行规划好的轨迹**

将上一步 result 中的 `planned_trajectory` 填入：

```bash
ros2 action send_goal /execute_trajectory moveit_msgs/action/ExecuteTrajectory "{
  trajectory: {
    joint_trajectory: {
      joint_names: ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7'],
      points: [
        {positions: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 0, nanosec: 0}},
        {positions: [0.1, -0.9,   0.0, -2.2,   0.0, 1.6,   0.8],   velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2, nanosec: 0}}
      ]
    }
  }
}"
```

---

## 与 MoveGroup 的对比

| | MoveGroup | ExecuteTrajectory |
|---|---|---|
| 规划 | 是 | 否 |
| 执行 | 是（`plan_only=false`） | 是 |
| 适用场景 | 常规运动 | 外部算法/离线规划/调试 |
| 控制粒度 | 低（全托管） | 高（自己控制轨迹每个点） |
