# Pickup / Place Actions

Action 类型：
- `moveit_msgs/action/Pickup`
- `moveit_msgs/action/Place`

Action 服务器：`/pickup`、`/place`

> **重要提示：Pickup/Place 在 MoveIt2 中已被标记为 deprecated（不推荐使用）。**
> 官方推荐使用 **MoveIt Task Constructor（MTC）** 替代。
> 本文档记录其接口结构，便于理解遗留代码或评估迁移工作量。

---

## 定位

Pickup/Place 将抓取和放置操作封装为高层 action，自动规划：
1. 移动到预抓取位置
2. 打开夹爪（pre_grasp_posture）
3. 沿接近方向运动到抓取位置
4. 闭合夹爪（grasp_posture）
5. 沿撤退方向抬起物体

Place 则是抓取的逆过程。

---

## Pickup — Goal 字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `target_name` | string | 规划场景中目标物体的 ID |
| `group_name` | string | 规划组名（手臂） |
| `end_effector` | string | 末端执行器组名（夹爪） |
| `possible_grasps[]` | Grasp[] | **候选抓取姿态列表**，至少填一个 |
| `support_surface_name` | string | 支撑面名称（如桌子 ID），可为空 |
| `allow_gripper_support_collision` | bool | 接近和抬起时是否允许夹爪与支撑面碰撞 |
| `attached_object_touch_links[]` | string[] | 抓取后物体允许接触的 link |
| `minimize_object_distance` | bool | 是否尽量靠近物体后再抓（修改抓取姿态） |
| `path_constraints` | Constraints | 整个过程的路径约束 |
| `planner_id` | string | 规划器 |
| `allowed_planning_time` | float64 | 规划超时 |
| `planning_options` | PlanningOptions | 同 MoveGroup |

**Grasp 结构（每个候选抓取）：**

| 字段 | 说明 |
|------|------|
| `id` | 抓取姿态的标识 |
| `pre_grasp_posture` | 夹爪张开的关节轨迹（接近时） |
| `grasp_posture` | 夹爪闭合的关节轨迹（抓取时） |
| `grasp_pose` | 抓取时末端执行器的 6DOF 位姿（PoseStamped） |
| `grasp_quality` | 质量分数，规划器优先尝试高分姿态 |
| `pre_grasp_approach` | 接近方向 + 距离（direction + desired_distance + min_distance） |
| `post_grasp_retreat` | 抓取后撤退方向 + 距离 |
| `max_contact_force` | 最大允许接触力（N） |
| `allowed_touch_objects[]` | 抓取时允许接触的物体 |

---

## Pickup — Result 字段

| 字段 | 说明 |
|------|------|
| `error_code` | MoveItErrorCodes |
| `trajectory_start` | 起始机器人状态 |
| `trajectory_stages[]` | 各阶段的轨迹（接近、抓取、撤退等） |
| `trajectory_descriptions[]` | 各阶段名称描述 |
| `grasp` | 实际成功使用的抓取姿态 |
| `planning_time` | 规划耗时 |

---

## Pickup — 示例命令

```bash
ros2 action send_goal /pickup moveit_msgs/action/Pickup "{
  target_name: 'red_box',
  group_name: 'fr3_arm',
  end_effector: 'fr3_hand',
  possible_grasps: [{
    id: 'top_grasp',
    pre_grasp_posture: {
      joint_names: ['fr3_finger_joint1', 'fr3_finger_joint2'],
      points: [{positions: [0.04, 0.04], time_from_start: {sec: 1, nanosec: 0}}]
    },
    grasp_posture: {
      joint_names: ['fr3_finger_joint1', 'fr3_finger_joint2'],
      points: [{positions: [0.0, 0.0], time_from_start: {sec: 1, nanosec: 0}}]
    },
    grasp_pose: {
      header: {frame_id: 'base'},
      pose: {
        position: {x: 0.4, y: 0.0, z: 0.3},
        orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}
      }
    },
    grasp_quality: 1.0,
    pre_grasp_approach: {
      direction: {
        header: {frame_id: 'base'},
        vector: {x: 0.0, y: 0.0, z: -1.0}
      },
      desired_distance: 0.1,
      min_distance: 0.05
    },
    post_grasp_retreat: {
      direction: {
        header: {frame_id: 'base'},
        vector: {x: 0.0, y: 0.0, z: 1.0}
      },
      desired_distance: 0.15,
      min_distance: 0.1
    }
  }],
  support_surface_name: 'table',
  allow_gripper_support_collision: true,
  allowed_planning_time: 10.0,
  planning_options: {plan_only: false, planning_scene_diff: {is_diff: true}}
}"
```

---

## Place — Goal 字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `group_name` | string | 规划组名 |
| `attached_object_name` | string | 当前已抓取并附着的物体 ID |
| `place_locations[]` | PlaceLocation[] | **候选放置位置列表** |
| `place_eef` | bool | `true`=用末端坐标描述位置；`false`=用物体坐标 |
| `support_surface_name` | string | 目标支撑面 ID |
| `allow_gripper_support_collision` | bool | 放置时是否允许夹爪与支撑面碰撞 |
| `path_constraints` | Constraints | 整个过程的路径约束 |
| `planner_id` | string | 规划器 |
| `allowed_planning_time` | float64 | 规划超时 |
| `planning_options` | PlanningOptions | 同 MoveGroup |

**PlaceLocation 结构：**

| 字段 | 说明 |
|------|------|
| `id` | 放置位置标识 |
| `post_place_posture` | 放置后夹爪张开的关节轨迹 |
| `place_pose` | 放置位置的 6DOF 位姿 |
| `quality` | 质量分数 |
| `pre_place_approach` | 接近方向 + 距离 |
| `post_place_retreat` | 放置后撤退方向 + 距离 |

---

## Place — Result 字段

| 字段 | 说明 |
|------|------|
| `error_code` | MoveItErrorCodes |
| `trajectory_start` | 起始状态 |
| `trajectory_stages[]` | 各阶段轨迹 |
| `place_location` | 实际成功使用的放置位置 |
| `planning_time` | 规划耗时 |

---

## Deprecated 说明

Pickup/Place 的问题：
1. 只支持简单的接近-抓取-撤退模式，无法表达复杂的多步骤操作
2. 抓取候选需要外部算法提供（如 GPD、GraspIt），与 action 没有集成
3. 错误恢复能力有限

**替代方案：MoveIt Task Constructor（MTC）**  
MTC 使用分层任务规划，支持任意顺序的子任务组合（移动、抓取、操作、放置），并内置回退和恢复机制。  
如果是新项目，直接使用 MTC。
