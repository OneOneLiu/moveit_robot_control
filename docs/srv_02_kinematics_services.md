# 运动学 Services：IK / FK

---

## GetPositionIK — 逆运动学

服务路径：`/compute_ik`
类型：`moveit_msgs/srv/GetPositionIK`

**定位：** 给定末端位姿，求解满足条件的关节角度配置。MoveGroup 在内部规划时会自动调用 IK，这个 service 允许你单独调用 IK 求解器，用于调试、验证可达性，或自定义规划流程。

**Request 关键字段（ik_request）：**

| 字段 | 说明 |
|------|------|
| `group_name` | 规划组名 |
| `ik_link_name` | 单目标时指定末端 link |
| `pose_stamped` | 单目标时的末端位姿（PoseStamped） |
| `ik_link_names[]` | 多目标时的 link 列表 |
| `pose_stamped_vector[]` | 多目标时的位姿列表（与 ik_link_names 一一对应） |
| `robot_state` | 初始猜测状态（影响 IK 求解器搜索方向，7 轴臂有多解时尤其重要） |
| `constraints` | IK 解必须满足的额外约束（关节限位、碰撞等） |
| `avoid_collisions` | 是否排除碰撞状态的 IK 解 |
| `timeout` | IK 求解超时 |

**Response：**

| 字段 | 说明 |
|------|------|
| `solution` | IK 求解结果（RobotState，含所有关节值） |
| `error_code` | `SUCCESS=1`，`NO_IK_SOLUTION=-31` |

**示例命令（求解末端到达某位姿的关节角）：**

```bash
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{
  ik_request: {
    group_name: 'fr3_arm',
    ik_link_name: 'fr3_hand',
    pose_stamped: {
      header: {frame_id: 'base'},
      pose: {
        position: {x: 0.5, y: 0.0, z: 0.5},
        orientation: {x: 0.912, y: 0.380, z: 0.145, w: 0.058}
      }
    },
    avoid_collisions: true,
    timeout: {sec: 5, nanosec: 0}
  }
}"
```

**注意事项：**
- 7 轴冗余机械臂有无穷多 IK 解，`robot_state`（初始猜测）会影响返回哪个解
- `avoid_collisions: true` 时，会过滤掉碰撞状态的解，速度较慢
- 返回的是**一个**解，不是所有解；如果需要多个候选解，需要多次调用并改变初始猜测

---

## GetPositionFK — 正运动学

服务路径：`/compute_fk`
类型：`moveit_msgs/srv/GetPositionFK`

**定位：** 给定关节角度，计算指定 link 的笛卡尔位姿。比 tf2 更灵活——可以对**任意假设的关节状态**（不一定是当前状态）求 FK，也可以同时查询多个 link。

**Request 字段：**

| 字段 | 说明 |
|------|------|
| `header.frame_id` | 结果位姿的参考系 |
| `fk_link_names[]` | 要计算位姿的 link 列表 |
| `robot_state` | 用于计算 FK 的关节状态 |

**Response：**

| 字段 | 说明 |
|------|------|
| `pose_stamped[]` | 各 link 的位姿（与 fk_link_names 一一对应） |
| `fk_link_names[]` | 对应的 link 名称列表 |
| `error_code` | 错误码 |

**示例命令（查询 home 位时末端位姿）：**

```bash
ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{
  header: {frame_id: 'base'},
  fk_link_names: ['fr3_hand'],
  robot_state: {
    joint_state: {
      name: ['fr3_joint1','fr3_joint2','fr3_joint3','fr3_joint4','fr3_joint5','fr3_joint6','fr3_joint7'],
      position: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    }
  }
}"
```

**示例命令（同时查询多个 link）：**

```bash
ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{
  header: {frame_id: 'base'},
  fk_link_names: ['fr3_link4', 'fr3_link7', 'fr3_hand'],
  robot_state: {
    joint_state: {
      name: ['fr3_joint1','fr3_joint2','fr3_joint3','fr3_joint4','fr3_joint5','fr3_joint6','fr3_joint7'],
      position: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    }
  }
}"
```

---

## IK vs FK 对比

| | GetPositionIK | GetPositionFK |
|---|---|---|
| 输入 | 末端位姿 | 关节角度 |
| 输出 | 关节角度 | 末端位姿 |
| 有解唯一性问题 | 是（多解） | 否（唯一） |
| 可能无解 | 是（不可达时） | 否 |
| 典型用途 | 验证目标可达性 | 查询关节构型对应的末端位置 |

**与 tf2_echo 的区别：**
- `tf2_echo` 只能查当前实际状态的 FK
- `GetPositionFK` 可以对**任意假设关节角度**求 FK，用于离线计算
