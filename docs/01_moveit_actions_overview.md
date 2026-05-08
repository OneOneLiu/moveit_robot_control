# MoveIt Action 接口总览

MoveIt 提供 7 个 action 接口，覆盖从简单运动到复杂操作的全部场景。

---

## 一览表

| Action | 服务器 | 一句话定位 | 状态 |
|--------|--------|-----------|------|
| `MoveGroup` | `/move_action` | 单段运动：规划 + 执行 | 主力接口 |
| `ExecuteTrajectory` | `/execute_trajectory` | 只执行，不规划 | 常用 |
| `MoveGroupSequence` | `/sequence_move_group` | 多段运动，支持段间融合 | 常用（Pilz） |
| `HybridPlanner` | `/run_hybrid_planning` | 全局规划 + 实时局部修正 | 动态环境 |
| `GlobalPlanner` | `/run_global_planner` | 混合规划的全局层（内部组件） | 内部 |
| `LocalPlanner` | `/run_local_planner` | 混合规划的局部层（内部组件） | 内部 |
| `Pickup` | `/pickup` | 高层抓取操作 | **Deprecated** |
| `Place` | `/place` | 高层放置操作 | **Deprecated** |

---

## 选择哪个 Action

```
需要运动到某个目标？
├─ 静态环境（障碍物不动）
│   ├─ 单段运动                    → MoveGroup
│   ├─ 多段连续运动（工业流程）     → MoveGroupSequence
│   └─ 已有轨迹，只需执行           → ExecuteTrajectory
│
└─ 动态环境（人/障碍物实时变化）    → HybridPlanner

需要抓取/放置物体？
└─ 新项目                          → MoveIt Task Constructor（MTC，不是 action）
   旧代码维护                       → Pickup / Place（已 deprecated）
```

---

## 各 Action 的 Goal 核心字段对比

### MoveGroup
```yaml
request:
  group_name: 'fr3_arm'
  goal_constraints: [...]       # 目标约束
  path_constraints: ...         # 路径约束
  pipeline_id / planner_id      # 规划器选择
  max_velocity_scaling_factor
planning_options:
  plan_only: false
```

### ExecuteTrajectory
```yaml
trajectory:
  joint_trajectory:
    joint_names: [...]
    points: [{positions, velocities, time_from_start}, ...]
controller_names: []            # 可选，指定控制器
```

### MoveGroupSequence
```yaml
request:
  items:
    - req: {MotionPlanRequest...}  # 同 MoveGroup 的 request
      blend_radius: 0.05           # > 0 则过渡融合，0 则停顿
    - req: {MotionPlanRequest...}
      blend_radius: 0.0            # 最后一段必须为 0
```

### HybridPlanner
```yaml
planning_group: 'fr3_arm'
motion_sequence:                   # 同 MoveGroupSequence 的 request
  items: [...]
```

### Pickup
```yaml
target_name: 'object_id'
group_name: 'fr3_arm'
end_effector: 'fr3_hand'
possible_grasps:
  - grasp_pose: {PoseStamped}
    pre_grasp_posture: {JointTrajectory}   # 夹爪张开
    grasp_posture: {JointTrajectory}       # 夹爪闭合
    pre_grasp_approach: {direction, distance}
    post_grasp_retreat: {direction, distance}
```

### Place
```yaml
group_name: 'fr3_arm'
attached_object_name: 'object_id'
place_locations:
  - place_pose: {PoseStamped}
    post_place_posture: {JointTrajectory}  # 夹爪张开
    pre_place_approach: {direction, distance}
    post_place_retreat: {direction, distance}
```

---

## 规划器选择速查

| pipeline_id | planner_id | 路径类型 | 备注 |
|-------------|------------|---------|------|
| `ompl` | `RRTConnect` | 关节空间自由路径 | 默认，通用 |
| `ompl` | `RRT*` | 关节空间渐进最优 | 较慢但路径更优 |
| `pilz_industrial_motion_planner` | `PTP` | 关节空间点对点 | 各关节同步，确定性 |
| `pilz_industrial_motion_planner` | `LIN` | 笛卡尔直线 | 必须全程有 IK 解 |
| `pilz_industrial_motion_planner` | `CIRC` | 笛卡尔圆弧 | 需要中间辅助点 |

---

## 文档索引（建议阅读顺序）

| 序号 | 文件 | 内容 |
|------|------|------|
| 01 | `01_moveit_actions_overview.md` | **本文**，总览与选择指南 |
| 02 | `02_moveit_movegroup_design_philosophy.md` | MoveIt 接口设计理念（先读懂为什么） |
| 03 | `03_moveit_movegroup_action.md` | MoveGroup 详细参数和示例（核心接口） |
| 04 | `04_execute_trajectory_action.md` | ExecuteTrajectory 详细参数和示例 |
| 05 | `05_movegroup_sequence_action.md` | MoveGroupSequence 详细参数和示例 |
| 06 | `06_hybrid_planner_actions.md` | HybridPlanner / GlobalPlanner / LocalPlanner |
| 07 | `07_pickup_place_actions.md` | Pickup / Place（deprecated） |
