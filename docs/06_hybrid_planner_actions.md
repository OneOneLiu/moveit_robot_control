# Hybrid Planner Actions

涉及三个 action：
- `moveit_msgs/action/HybridPlanner` — 入口，统一调用全局+局部规划器
- `moveit_msgs/action/GlobalPlanner` — 全局规划组件
- `moveit_msgs/action/LocalPlanner` — 局部规划组件

Action 服务器：
- `/run_hybrid_planning`（HybridPlanner）
- `/run_global_planner`（GlobalPlanner，通常由 HybridPlanner 内部调用）
- `/run_local_planner`（LocalPlanner，通常由 HybridPlanner 内部调用）

---

## 定位：为什么需要混合规划

MoveGroup 是**离线规划**：先算完整路径，再执行。执行期间环境变化（新障碍物出现）无法响应。

混合规划架构将规划分为两层：

```
GlobalPlanner（全局规划器）
  → 离线计算从起点到终点的参考轨迹（不考虑动态变化）

LocalPlanner（局部规划器）
  → 实时跟踪全局轨迹，同时响应实时传感器数据和动态障碍物
  → 在不偏离全局目标的前提下，在线修正轨迹

HybridPlanner
  → 协调两者，对外提供统一接口
```

类比：导航系统先规划总路线（全局），开车时再根据实时路况动态调整（局部）。

---

## HybridPlanner — Goal 字段

```
planning_group:   string                  # 规划组名
motion_sequence:  MotionSequenceRequest   # 与 MoveGroupSequence 完全相同的结构
  items[]:
    req:   MotionPlanRequest              # 每段运动的规划请求
    blend_radius: float64
```

## HybridPlanner — Result 字段

| 字段 | 说明 |
|------|------|
| `error_code` | MoveItErrorCodes |
| `error_message` | 详细错误信息（比 error_code 更具体） |

## HybridPlanner — Feedback 字段

| 字段 | 说明 |
|------|------|
| `feedback` | 当前状态描述字符串 |

---

## GlobalPlanner — Goal 字段

与 HybridPlanner 相同（`planning_group` + `motion_sequence`）。

## GlobalPlanner — Result 字段

| 字段 | 说明 |
|------|------|
| `response` | MotionPlanResponse，包含 `trajectory`、`trajectory_start`、`planning_time`、`error_code` |

## GlobalPlanner — Feedback 字段

| 字段 | 说明 |
|------|------|
| `feedback` | 状态字符串 |

---

## LocalPlanner — Goal 字段

```
local_constraints[]:   Constraints[]   # 局部规划器执行过程中额外施加的约束
```

注意：LocalPlanner 本身不接收目标位置，它的目标轨迹来自 GlobalPlanner 发布的 topic，局部规划器订阅并实时跟踪。

## LocalPlanner — Result 字段

| 字段 | 说明 |
|------|------|
| `error_code` | MoveItErrorCodes |
| `error_message` | 详细错误信息 |

---

## 示例命令：通过 HybridPlanner 发送运动目标

通常只需要调用 HybridPlanner，不直接调用 Global/LocalPlanner：

```bash
ros2 action send_goal /run_hybrid_planning moveit_msgs/action/HybridPlanner "{
  planning_group: 'fr3_arm',
  motion_sequence: {
    items: [
      {
        req: {
          group_name: 'fr3_arm',
          pipeline_id: 'ompl',
          planner_id: 'RRTConnect',
          num_planning_attempts: 5,
          allowed_planning_time: 5.0,
          max_velocity_scaling_factor: 0.1,
          max_acceleration_scaling_factor: 0.1,
          goal_constraints: [{
            position_constraints: [{
              header: {frame_id: 'base'},
              link_name: 'fr3_hand',
              constraint_region: {
                primitives: [{type: 2, dimensions: [0.01]}],
                primitive_poses: [{
                  position: {x: 0.5, y: 0.0, z: 0.5},
                  orientation: {w: 1.0}
                }]
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

---

## 与 MoveGroup 的对比

| | MoveGroup | HybridPlanner |
|---|---|---|
| 规划方式 | 离线一次规划 | 全局离线 + 局部在线 |
| 动态障碍物响应 | 不支持（执行中无法响应） | **支持**（局部规划器实时修正） |
| 部署复杂度 | 低 | 高（需要配置两个规划器节点） |
| 适用场景 | 静态环境 | 动态环境、人机协作 |

---

## 注意事项

- HybridPlanner 是 MoveIt2 引入的新架构，需要单独启动 `hybrid_planning_manager`、`global_planner_component`、`local_planner_component` 三个节点
- 局部规划器的具体算法（如 STOMP、TrajOpt、时间缩放等）通过插件配置，接口本身不限定实现
- 如果只需要静态环境下的运动规划，直接用 MoveGroup 更简单
