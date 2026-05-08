# 规划类 Services

---

## GetMotionPlan — 同步规划

服务路径：`/plan_kinematic_path`
类型：`moveit_msgs/srv/GetMotionPlan`

**定位：** MoveGroup action（`plan_only=true`）的 service 版本。同步阻塞，只规划不执行，省略了 action 的状态机开销。

**Request：** 完整的 `MotionPlanRequest`，字段与 MoveGroup action 的 `request` 完全一致（group_name、goal_constraints、path_constraints、planner_id 等）。

**Response：**
| 字段 | 说明 |
|------|------|
| `motion_plan_response.trajectory` | 规划出的轨迹 |
| `motion_plan_response.trajectory_start` | 起始状态 |
| `motion_plan_response.planning_time` | 规划耗时（秒） |
| `motion_plan_response.error_code` | 错误码 |
| `motion_plan_response.group_name` | 规划组名 |

**示例命令：**

```bash
ros2 service call /plan_kinematic_path moveit_msgs/srv/GetMotionPlan "{
  motion_plan_request: {
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
  }
}"
```

**与 MoveGroup action（plan_only=true）的差异：**
- Service 是同步调用，阻塞直到返回
- 没有 Feedback，没有取消机制
- 略低的协议开销（无状态机握手）
- 适合不需要进度监控的批量规划场景

---

## GetCartesianPath — 笛卡尔多点路径

服务路径：`/compute_cartesian_path`
类型：`moveit_msgs/srv/GetCartesianPath`

**定位：** 计算末端经过一系列笛卡尔 waypoints 的轨迹。这是 MoveGroup action 做不到的功能——action 只能指定终点，这个 service 可以指定路径上的所有中间点。

**Request 关键字段：**

| 字段 | 说明 |
|------|------|
| `header.frame_id` | waypoints 所在参考系 |
| `group_name` | 规划组名 |
| `link_name` | 要跟踪 waypoints 的 link（末端） |
| `waypoints[]` | 末端依次经过的位姿列表（Pose[]） |
| `max_step` | 相邻路径点的最大间距（米），**必须 > 0** |
| `jump_threshold` | 关节跳变检测阈值（缩放因子），> 0 时启用 |
| `prismatic_jump_threshold` | 平动关节绝对跳变阈值（米） |
| `revolute_jump_threshold` | 转动关节绝对跳变阈值（弧度） |
| `avoid_collisions` | 是否做碰撞检测 |
| `path_constraints` | 路径约束（同 MoveGroup） |
| `max_velocity_scaling_factor` | 速度缩放 |
| `max_acceleration_scaling_factor` | 加速度缩放 |
| `start_state` | 起始状态，留空用当前状态 |

**Response 关键字段：**

| 字段 | 说明 |
|------|------|
| `solution` | 计算出的轨迹（RobotTrajectory） |
| `fraction` | 实际完成的路径比例 `[0,1]`，= 1 表示所有 waypoints 都可达 |
| `start_state` | 实际起始状态 |
| `error_code` | 错误码 |

**重要：fraction 字段**

`fraction < 1.0` 表示轨迹在某个 waypoint 处中断（IK 无解或碰撞）。调用方需要检查此值决定是否执行。

**示例命令（末端走三个笛卡尔点）：**

```bash
ros2 service call /compute_cartesian_path moveit_msgs/srv/GetCartesianPath "{
  header: {frame_id: 'base'},
  group_name: 'fr3_arm',
  link_name: 'fr3_hand',
  waypoints: [
    {position: {x: 0.4, y: 0.0,  z: 0.5}, orientation: {x: 0.912, y: 0.380, z: 0.145, w: 0.058}},
    {position: {x: 0.4, y: 0.1,  z: 0.5}, orientation: {x: 0.912, y: 0.380, z: 0.145, w: 0.058}},
    {position: {x: 0.4, y: -0.1, z: 0.5}, orientation: {x: 0.912, y: 0.380, z: 0.145, w: 0.058}}
  ],
  max_step: 0.01,
  jump_threshold: 0.0,
  avoid_collisions: true,
  max_velocity_scaling_factor: 0.1,
  max_acceleration_scaling_factor: 0.1
}"
```

**典型使用流程：**
```
1. 调用 GetCartesianPath → 获得 solution 轨迹
2. 检查 fraction == 1.0
3. 调用 ExecuteTrajectory action 执行 solution
```

---

## GetMotionSequence — 同步序列规划

服务路径：`/plan_sequence_path`
类型：`moveit_msgs/srv/GetMotionSequence`

**定位：** MoveGroupSequence action 的 service 版本，只规划不执行。

**Request：** `MotionSequenceRequest`，与 MoveGroupSequence action 的结构完全相同（`items[]` 数组，每项包含 `MotionPlanRequest` + `blend_radius`）。

**Response：**

| 字段 | 说明 |
|------|------|
| `response.error_code` | 错误码 |
| `response.sequence_start` | 起始状态 |
| `response.planned_trajectories[]` | 各段规划轨迹数组 |
| `response.planning_time` | 总规划耗时 |

**示例命令：**

```bash
ros2 service call /plan_sequence_path moveit_msgs/srv/GetMotionSequence "{
  request: {
    items: [
      {
        req: {
          group_name: 'fr3_arm',
          pipeline_id: 'pilz_industrial_motion_planner',
          planner_id: 'LIN',
          max_velocity_scaling_factor: 0.2,
          max_acceleration_scaling_factor: 0.2,
          goal_constraints: [{
            position_constraints: [{
              header: {frame_id: 'base'},
              link_name: 'fr3_hand',
              constraint_region: {
                primitives: [{type: 2, dimensions: [0.005]}],
                primitive_poses: [{position: {x: 0.4, y: 0.1, z: 0.5}, orientation: {w: 1.0}}]
              },
              weight: 1.0
            }]
          }]
        },
        blend_radius: 0.05
      },
      {
        req: {
          group_name: 'fr3_arm',
          pipeline_id: 'pilz_industrial_motion_planner',
          planner_id: 'LIN',
          max_velocity_scaling_factor: 0.2,
          max_acceleration_scaling_factor: 0.2,
          goal_constraints: [{
            position_constraints: [{
              header: {frame_id: 'base'},
              link_name: 'fr3_hand',
              constraint_region: {
                primitives: [{type: 2, dimensions: [0.005]}],
                primitive_poses: [{position: {x: 0.4, y: -0.1, z: 0.5}, orientation: {w: 1.0}}]
              },
              weight: 1.0
            }]
          }]
        },
        blend_radius: 0.0
      }
    ]
  }
}"
```
