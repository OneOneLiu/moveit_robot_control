# MoveIt MoveGroup Action 接口文档

Action 类型：`moveit_msgs/action/MoveGroup`
Action 服务器：`/move_action`

---

## 整体结构

```
Goal      →  发送给 MoveGroup 的规划与执行请求
Result    ←  规划完成/执行完成后返回的结果
Feedback  ←  执行过程中的实时状态
```

---

## Goal 字段

### 1. `request`（MotionPlanRequest）— 规划请求

#### 1.1 基础参数

| 字段 | 类型 | 说明 |
|------|------|------|
| `group_name` | string | **必填**，规划组名，如 `fr3_arm` |
| `pipeline_id` | string | 规划管线，如 `ompl`、`pilz_industrial_motion_planner` |
| `planner_id` | string | 具体规划器，如 `RRTConnect`、`LIN`、`PTP` |
| `num_planning_attempts` | int32 | 最大规划尝试次数，建议 5~10 |
| `allowed_planning_time` | float64 | 规划超时（秒），建议 5.0 |
| `max_velocity_scaling_factor` | float64 | 速度缩放 `(0, 1]`，测试时建议 0.1 |
| `max_acceleration_scaling_factor` | float64 | 加速度缩放 `(0, 1]`，测试时建议 0.1 |
| `max_cartesian_speed` | float64 | 最大笛卡尔速度（m/s），需配合下方字段使用 |
| `cartesian_speed_limited_link` | string | 要限速的 link 名 |

#### 1.2 起始状态 `start_state`

不填则使用机器人当前实际状态。

```yaml
start_state:
  joint_state:
    name: ['fr3_joint1', 'fr3_joint2', ...]
    position: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
  is_diff: false   # false=完整状态，true=只更新指定字段
```

#### 1.3 目标约束 `goal_constraints[]`

数组中每个元素是一组约束（组内取 AND，组间取 OR）。支持四种约束类型混合使用。

---

## 功能一：关节空间目标（JointConstraint）

直接指定每个关节的目标角度，**跳过 IK**，适合已知构型或回零场景。

**所需参数：**

| 字段 | 说明 |
|------|------|
| `joint_name` | 关节名称 |
| `position` | 目标角度（弧度） |
| `tolerance_above` | 正方向容差（弧度） |
| `tolerance_below` | 负方向容差（弧度） |
| `weight` | 约束权重，通常设 1.0 |

**示例命令：**

```bash
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "{
  request: {
    group_name: 'fr3_arm',
    num_planning_attempts: 5,
    allowed_planning_time: 5.0,
    max_velocity_scaling_factor: 0.1,
    max_acceleration_scaling_factor: 0.1,
    goal_constraints: [{
      joint_constraints: [
        {joint_name: 'fr3_joint1', position: 0.0,    tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0},
        {joint_name: 'fr3_joint2', position: -0.785, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0},
        {joint_name: 'fr3_joint3', position: 0.0,    tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0},
        {joint_name: 'fr3_joint4', position: -2.356, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0},
        {joint_name: 'fr3_joint5', position: 0.0,    tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0},
        {joint_name: 'fr3_joint6', position: 1.571,  tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0},
        {joint_name: 'fr3_joint7', position: 0.785,  tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}
      ]
    }]
  },
  planning_options: {
    plan_only: false,
    planning_scene_diff: {is_diff: true}
  }
}"
```

---

## 功能二：笛卡尔位置目标（PositionConstraint）

控制指定 link 运动到空间中某个位置，**MoveIt 内部自动求解 IK**。
通过 `constraint_region` 指定一个容差区域（球体最常用）。

**所需参数：**

| 字段 | 说明 |
|------|------|
| `header.frame_id` | 参考坐标系，如 `base`、`world` |
| `link_name` | 要控制的 link，通常是末端 |
| `constraint_region.primitives[0].type` | 容差区域形状：`1`=BOX，`2`=SPHERE，`3`=CYLINDER |
| `constraint_region.primitives[0].dimensions` | SPHERE: `[半径]`；BOX: `[x,y,z]` |
| `constraint_region.primitive_poses[0].position` | 目标位置 xyz |
| `target_point_offset` | link 原点到控制点的偏移，通常为 `{x:0,y:0,z:0}` |
| `weight` | 约束权重，通常设 1.0 |

**示例命令（仅控制位置，姿态自由）：**

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
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
          }]
        },
        weight: 1.0
      }]
    }]
  },
  planning_options: {
    plan_only: false,
    planning_scene_diff: {is_diff: true}
  }
}"
```

---

## 功能三：笛卡尔姿态目标（OrientationConstraint）

控制指定 link 保持或到达某个姿态。通常与 PositionConstraint 一起使用实现完整位姿控制。

**所需参数：**

| 字段 | 说明 |
|------|------|
| `header.frame_id` | 参考坐标系 |
| `link_name` | 要控制的 link |
| `orientation` | 目标四元数 `{x, y, z, w}` |
| `absolute_x_axis_tolerance` | 绕 X 轴容差（弧度） |
| `absolute_y_axis_tolerance` | 绕 Y 轴容差（弧度） |
| `absolute_z_axis_tolerance` | 绕 Z 轴容差（弧度） |
| `parameterization` | `0`=XYZ_EULER_ANGLES（默认），`1`=ROTATION_VECTOR |
| `weight` | 约束权重，通常设 1.0 |

**获取当前末端四元数：**

```bash
ros2 run tf2_ros tf2_echo base fr3_hand
# 取输出中 Rotation: in Quaternion (xyzw) 一行
```

**示例命令（位置 + 保持当前姿态）：**

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
            position: {x: 0.5, y: 0.5, z: 0.5},
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
          }]
        },
        weight: 1.0
      }],
      orientation_constraints: [{
        header: {frame_id: 'base'},
        link_name: 'fr3_hand',
        orientation: {x: 0.912, y: 0.380, z: 0.145, w: 0.058},
        absolute_x_axis_tolerance: 0.01,
        absolute_y_axis_tolerance: 0.01,
        absolute_z_axis_tolerance: 0.01,
        weight: 1.0
      }]
    }]
  },
  planning_options: {
    plan_only: false,
    planning_scene_diff: {is_diff: true}
  }
}"
```

---

## 功能四：路径约束（PathConstraints）

整条运动轨迹上每个点都必须满足的约束（而非仅目标点）。
结构与 `goal_constraints` 完全相同，填在 `request.path_constraints` 字段下。

常见用途：末端保持水平（防止洒水）、末端始终朝向某方向。

**示例片段（路径中保持末端朝下）：**

```yaml
request:
  path_constraints:
    orientation_constraints:
      - header: {frame_id: 'base'}
        link_name: 'fr3_hand'
        orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}
        absolute_x_axis_tolerance: 0.1
        absolute_y_axis_tolerance: 0.1
        absolute_z_axis_tolerance: 0.1
        weight: 1.0
```

---

## 功能五：动态场景障碍物（PlanningSceneDiff）

在规划前临时向场景中添加或移除碰撞体，无需修改持久化场景。

**操作类型：** `operation` 字段取值：`0`=ADD，`1`=REMOVE，`2`=APPEND，`3`=MOVE

**支持的形状（SolidPrimitive.type）：**

| 值 | 形状 | dimensions |
|----|------|------------|
| 1 | BOX | `[x长, y长, z长]` |
| 2 | SPHERE | `[半径]` |
| 3 | CYLINDER | `[高度, 半径]` |
| 4 | CONE | `[高度, 半径]` |

**示例命令（添加一个球形障碍物后再规划）：**

```bash
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "{
  request: {
    group_name: 'fr3_arm',
    num_planning_attempts: 10,
    allowed_planning_time: 10.0,
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
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
          }]
        },
        weight: 1.0
      }]
    }]
  },
  planning_options: {
    plan_only: false,
    planning_scene_diff: {
      is_diff: true,
      world: {
        collision_objects: [{
          header: {frame_id: 'base'},
          id: 'obstacle_sphere',
          primitives: [{type: 2, dimensions: [0.1]}],
          primitive_poses: [{
            position: {x: 0.4, y: 0.2, z: 0.5},
            orientation: {w: 1.0}
          }],
          operation: 0
        }]
      }
    }
  }
}"
```

---

## 功能六：仅规划不执行（plan_only）

将 `planning_options.plan_only` 设为 `true`，只生成轨迹不下发执行。
用于验证规划可行性、调试目标点，或由外部模块决定是否执行。

Result 中的 `planned_trajectory` 会包含完整轨迹，`executed_trajectory` 为空。

```yaml
planning_options:
  plan_only: true
```

---

## 功能七：执行失败自动重规划（replan）

```yaml
planning_options:
  plan_only: false
  replan: true
  replan_attempts: 3
  replan_delay: 1.0   # 每次重规划前等待秒数
```

---

## Result 字段说明

| 字段 | 说明 |
|------|------|
| `error_code.val` | `1`=成功，负数=失败（见下表） |
| `error_code.message` | 错误描述 |
| `trajectory_start` | 实际规划起点状态 |
| `planned_trajectory` | 规划生成的轨迹（含位置/速度/加速度时间戳） |
| `executed_trajectory` | 实际执行时记录的真实轨迹 |
| `planning_time` | 规划耗时（秒） |

**常见错误码：**

| 值 | 含义 |
|----|------|
| `1` | SUCCESS |
| `-1` | PLANNING_FAILED |
| `-2` | INVALID_MOTION_PLAN |
| `-4` | CONTROL_FAILED（执行失败） |
| `-6` | TIMED_OUT |
| `-10` | START_STATE_IN_COLLISION |
| `-12` | GOAL_IN_COLLISION |
| `-15` | INVALID_GROUP_NAME |
| `-31` | NO_IK_SOLUTION |

---

## Feedback 字段说明

| 字段 | 可能值 |
|------|--------|
| `state` | `IDLE` / `PLANNING` / `MONITOR`（执行监控中） / `LOOK`（感知中） |

---

## 常用调试流程

```
1. plan_only: true           → 先验证规划是否可行
2. max_velocity_scaling_factor: 0.1  → 低速测试执行
3. tf2_echo base fr3_hand    → 查询当前末端位姿
4. error_code 检查           → 根据错误码定位问题
```
