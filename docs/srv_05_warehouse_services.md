# 机器人状态库（Warehouse）Services

涉及 6 个 service，共同构成一个机器人状态的持久化存储系统。

---

## 什么是 Warehouse

MoveIt Warehouse 是一个数据库（默认使用 MongoDB/warehouse_ros），用于持久化保存机器人的关节状态（named states），类似于"收藏夹"。

典型用途：
- 保存调试好的目标位置（如 home、pick_pose、place_pose）
- 在多次启动之间保留状态，不用每次手动输入关节值
- 配合 RViz 的 MotionPlanning 插件使用（可在界面上直接存取）

---

## SaveRobotStateToWarehouse — 保存状态

| 字段 | 说明 |
|------|------|
| `name` | 状态名称（string） |
| `robot` | 机器人名称（string） |
| `state` | 要保存的 RobotState |

**Response：** `success: bool`

**示例：**

```bash
ros2 service call /save_robot_state moveit_msgs/srv/SaveRobotStateToWarehouse "{
  name: 'home_pose',
  robot: 'fr3',
  state: {
    joint_state: {
      name: ['fr3_joint1','fr3_joint2','fr3_joint3','fr3_joint4','fr3_joint5','fr3_joint6','fr3_joint7'],
      position: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    }
  }
}"
```

---

## GetRobotStateFromWarehouse — 读取状态

| 字段 | 说明 |
|------|------|
| `name` | 状态名称 |
| `robot` | 机器人名称 |

**Response：** `state: RobotState`

**示例：**

```bash
ros2 service call /get_robot_state moveit_msgs/srv/GetRobotStateFromWarehouse "{
  name: 'home_pose',
  robot: 'fr3'
}"
```

---

## ListRobotStatesInWarehouse — 列出所有状态

| 字段 | 说明 |
|------|------|
| `regex` | 正则表达式过滤名称，`''` 或 `'.*'` 返回全部 |
| `robot` | 机器人名称 |

**Response：** `states: string[]`（状态名称列表）

**示例：**

```bash
ros2 service call /list_robot_states moveit_msgs/srv/ListRobotStatesInWarehouse "{
  regex: '.*',
  robot: 'fr3'
}"
```

```bash
# 只列出 pick 开头的状态
ros2 service call /list_robot_states moveit_msgs/srv/ListRobotStatesInWarehouse "{
  regex: 'pick.*',
  robot: 'fr3'
}"
```

---

## CheckIfRobotStateExistsInWarehouse — 检查状态是否存在

| 字段 | 说明 |
|------|------|
| `name` | 状态名称 |
| `robot` | 机器人名称 |

**Response：** `exists: bool`、`name: string`、`robot: string`

**示例：**

```bash
ros2 service call /has_robot_state moveit_msgs/srv/CheckIfRobotStateExistsInWarehouse "{
  name: 'home_pose',
  robot: 'fr3'
}"
```

---

## DeleteRobotStateFromWarehouse — 删除状态

| 字段 | 说明 |
|------|------|
| `name` | 状态名称 |
| `robot` | 机器人名称 |

**Response：** 空

**示例：**

```bash
ros2 service call /delete_robot_state moveit_msgs/srv/DeleteRobotStateFromWarehouse "{
  name: 'home_pose',
  robot: 'fr3'
}"
```

---

## RenameRobotStateInWarehouse — 重命名状态

| 字段 | 说明 |
|------|------|
| `old_name` | 原名称 |
| `new_name` | 新名称 |
| `robot` | 机器人名称 |

**Response：** 空

**示例：**

```bash
ros2 service call /rename_robot_state moveit_msgs/srv/RenameRobotStateInWarehouse "{
  old_name: 'home_pose',
  new_name: 'home',
  robot: 'fr3'
}"
```

---

## 注意事项

- Warehouse 功能需要安装并启动 `warehouse_ros` 及其数据库后端（MongoDB 或 SQLite）
- 如果没有启动数据库，这些 service 不会存在
- 验证是否可用：`ros2 service list | grep robot_state`
- 这些 service 主要供 RViz MotionPlanning 插件使用，代码中更常见的做法是在 launch 文件里写好 named states，而不是动态存取
