# MoveGroupSequence Action

Action 类型：`moveit_msgs/action/MoveGroupSequence`
Action 服务器：`/sequence_move_group`

---

## 定位

**多段运动一次性规划并连续执行，支持段间过渡融合（blend）。**

MoveGroup 每次只规划一段运动，到达目标后停下来再接收下一个请求。  
MoveGroupSequence 一次提交多段，规划器统一处理，相邻段之间可以用 `blend_radius` 做平滑过渡，机器人**不停顿**地连续运动。

主要配合 **Pilz Industrial Motion Planner** 使用（PTP / LIN / CIRC）。

---

## Goal 字段

```
request:
  items[]:              # 运动段数组，顺序执行
    req:                # 每段是一个完整的 MotionPlanRequest（与 MoveGroup 相同）
      group_name
      pipeline_id
      planner_id        # 通常为 PTP / LIN / CIRC
      goal_constraints
      path_constraints
      max_velocity_scaling_factor
      max_acceleration_scaling_factor
      ...
    blend_radius:       # 与下一段的过渡半径（米），0 表示停顿后再执行下一段
```

**blend_radius 的含义：**
- `0.0`：该段完全执行到终点停止，再开始下一段（默认行为）
- `> 0.0`：在距离终点 blend_radius 处开始过渡到下一段，轨迹平滑连续，**终点不会精确到达**
- 最后一段的 blend_radius 必须为 `0.0`

---

## Result 字段

与 MoveGroup 相同：`error_code`、`trajectory_start`、`planned_trajectory`、`executed_trajectory`、`planning_time`

---

## Feedback 字段

| 字段 | 说明 |
|------|------|
| `state` | 当前状态字符串 |

---

## 示例命令：LIN + LIN 两段直线运动（带过渡融合）

机器人从当前位置先直线运动到中间点，不停顿平滑过渡到终点：

```bash
ros2 action send_goal /sequence_move_group moveit_msgs/action/MoveGroupSequence "{
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
                primitive_poses: [{
                  position: {x: 0.4, y: 0.2, z: 0.5},
                  orientation: {w: 1.0}
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
                primitive_poses: [{
                  position: {x: 0.4, y: -0.2, z: 0.5},
                  orientation: {w: 1.0}
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
        blend_radius: 0.0
      }
    ]
  }
}"
```

---

## 示例命令：PTP + LIN（先关节空间到达，再直线运动）

```bash
ros2 action send_goal /sequence_move_group moveit_msgs/action/MoveGroupSequence "{
  request: {
    items: [
      {
        req: {
          group_name: 'fr3_arm',
          pipeline_id: 'pilz_industrial_motion_planner',
          planner_id: 'PTP',
          max_velocity_scaling_factor: 0.3,
          max_acceleration_scaling_factor: 0.3,
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
        blend_radius: 0.0
      },
      {
        req: {
          group_name: 'fr3_arm',
          pipeline_id: 'pilz_industrial_motion_planner',
          planner_id: 'LIN',
          max_velocity_scaling_factor: 0.1,
          max_acceleration_scaling_factor: 0.1,
          goal_constraints: [{
            position_constraints: [{
              header: {frame_id: 'base'},
              link_name: 'fr3_hand',
              constraint_region: {
                primitives: [{type: 2, dimensions: [0.005]}],
                primitive_poses: [{
                  position: {x: 0.5, y: 0.0, z: 0.4},
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

| | MoveGroup | MoveGroupSequence |
|---|---|---|
| 运动段数 | 1 | N 段 |
| 段间停顿 | 每段都停 | 可配置 blend_radius 不停顿 |
| 推荐规划器 | OMPL / Pilz | **Pilz**（必须） |
| 适用场景 | 单次运动 | 工业流程、多段焊接/涂胶/搬运 |
