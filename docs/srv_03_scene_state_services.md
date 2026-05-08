# 场景与状态管理 Services

---

## ApplyPlanningScene — 持久化修改场景

服务路径：`/apply_planning_scene`
类型：`moveit_msgs/srv/ApplyPlanningScene`

**定位：** 将场景更新**持久化**写入 MoveGroup 维护的规划场景，所有后续规划请求都会看到这个改动。

与 MoveGroup action 中 `planning_scene_diff` 的区别：
- `planning_scene_diff`：只在**本次规划请求**中临时生效
- `ApplyPlanningScene`：**永久生效**，直到下次更新

**Request：**

```
scene: PlanningScene
```

完整的 `PlanningScene` 结构，`is_diff: true` 表示增量更新（只改指定部分），`false` 表示全量替换。

**Response：**

| 字段 | 说明 |
|------|------|
| `success` | bool，是否成功 |

**示例：添加一个盒形障碍物**

```bash
ros2 service call /apply_planning_scene moveit_msgs/srv/ApplyPlanningScene "{
  scene: {
    is_diff: true,
    world: {
      collision_objects: [{
        header: {frame_id: 'base'},
        id: 'table',
        primitives: [{type: 1, dimensions: [1.0, 0.8, 0.05]}],
        primitive_poses: [{
          position: {x: 0.6, y: 0.0, z: 0.0},
          orientation: {w: 1.0}
        }],
        operation: 0
      }]
    }
  }
}"
```

**示例：移除障碍物**

```bash
ros2 service call /apply_planning_scene moveit_msgs/srv/ApplyPlanningScene "{
  scene: {
    is_diff: true,
    world: {
      collision_objects: [{
        id: 'table',
        operation: 1
      }]
    }
  }
}"
```

**示例：修改碰撞矩阵（允许两个 link 之间的碰撞）**

```bash
ros2 service call /apply_planning_scene moveit_msgs/srv/ApplyPlanningScene "{
  scene: {
    is_diff: true,
    allowed_collision_matrix: {
      entry_names: ['fr3_hand', 'table'],
      entry_values: [{enabled: [true]}]
    }
  }
}"
```

---

## GetPlanningScene — 查询当前场景

服务路径：`/get_planning_scene`
类型：`moveit_msgs/srv/GetPlanningScene`

**定位：** 读取 MoveGroup 当前维护的规划场景内容，用于调试、状态监控或外部系统同步。

**Request：**

```
components: PlanningSceneComponents
```

用位掩码指定需要哪些组件（按需请求，避免返回巨大消息）：

| 常量 | 值 | 说明 |
|------|----|------|
| `SCENE_SETTINGS` | 1 | 场景基本设置 |
| `ROBOT_STATE` | 2 | 机器人当前状态 |
| `ROBOT_STATE_ATTACHED_OBJECTS` | 4 | 附着在机器人上的物体 |
| `WORLD_OBJECT_NAMES` | 8 | 世界中物体的名称列表 |
| `WORLD_OBJECT_GEOMETRY` | 16 | 物体的几何信息 |
| `OCTOMAP` | 32 | Octomap 数据 |
| `TRANSFORMS` | 64 | TF 变换 |
| `ALLOWED_COLLISION_MATRIX` | 128 | 碰撞豁免矩阵 |
| `LINK_PADDING_AND_SCALING` | 256 | link 膨胀和缩放 |
| `OBJECT_COLORS` | 512 | 物体颜色 |

多个组件用 OR 组合，如 `2 | 16 = 18` 表示同时获取机器人状态和物体几何。

**Response：** 完整的 `PlanningScene` 结构。

**示例：只查询场景中的物体名称**

```bash
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{
  components: {components: 8}
}"
```

**示例：查询机器人当前状态 + 附着物体**

```bash
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{
  components: {components: 6}
}"
```

**示例：获取完整场景（components=0 返回全部）**

```bash
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{
  components: {components: 0}
}"
```

---

## GetStateValidity — 状态合法性检查

服务路径：`/check_state_validity`
类型：`moveit_msgs/srv/GetStateValidity`

**定位：** 检查给定的机器人状态是否合法：是否碰撞、是否满足约束。规划之前可以用来预检，也可以验证外部给出的关节配置是否安全。

**Request：**

| 字段 | 说明 |
|------|------|
| `robot_state` | 要检查的机器人状态 |
| `group_name` | 规划组名 |
| `constraints` | 额外的约束条件（可选） |

**Response：**

| 字段 | 说明 |
|------|------|
| `valid` | bool，`true` = 状态合法 |
| `contacts[]` | 碰撞点列表（碰撞时非空） |
| `contacts[].contact_body_1/2` | 发生碰撞的两个物体名称 |
| `contacts[].position` | 碰撞点位置 |
| `contacts[].depth` | 穿透深度 |
| `cost_sources[]` | 代价来源（障碍物代价场） |
| `constraint_result[]` | 每个约束的检查结果 |

**示例：检查 home 位是否有碰撞**

```bash
ros2 service call /check_state_validity moveit_msgs/srv/GetStateValidity "{
  robot_state: {
    joint_state: {
      name: ['fr3_joint1','fr3_joint2','fr3_joint3','fr3_joint4','fr3_joint5','fr3_joint6','fr3_joint7'],
      position: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    }
  },
  group_name: 'fr3_arm'
}"
```

---

## 三者关系

```
GetPlanningScene   → 读（只读，不修改）
ApplyPlanningScene → 写（持久修改）
GetStateValidity   → 查（碰撞/约束检测）

常用调试流程：
1. GetPlanningScene     → 确认场景中当前有哪些物体
2. ApplyPlanningScene   → 添加/删除障碍物
3. GetStateValidity     → 确认目标状态不碰撞
4. GetMotionPlan        → 规划路径
5. ExecuteTrajectory    → 执行
```
