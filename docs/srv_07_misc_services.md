# 杂项 Services

---

## GraspPlanning — 抓取姿态规划

类型：`moveit_msgs/srv/GraspPlanning`

**定位：** 给定目标物体，生成一组候选抓取位姿。这是抓取规划管线的第一步——在规划抓取动作之前，先用这个 service 获取机械臂能够执行的候选抓取配置。

**Request：**

| 字段 | 说明 |
|------|------|
| `group_name` | 执行抓取的规划组名 |
| `target` | 目标物体（CollisionObject），包含形状和位姿 |
| `support_surfaces[]` | 支撑面名称列表（如桌面），规划时避免穿透 |
| `candidate_grasps[]` | 可选的候选抓取位姿（由外部提供的初始猜测） |
| `movable_obstacles[]` | 需要回避但可以移动的障碍物列表 |

**Response：**

| 字段 | 说明 |
|------|------|
| `grasps[]` | 规划出的抓取位姿列表（按质量排序） |
| `error_code` | 错误码 |

每个 `Grasp` 包含：
- `grasp_pose`：抓取时的末端位姿
- `pre_grasp_approach`：接近路径（方向 + 距离）
- `post_grasp_retreat`：抓取后的退出路径
- `pre_grasp_posture` / `grasp_posture`：抓取前后的夹爪状态
- `grasp_quality`：质量评分（0.0 ~ 1.0）

**示例：**

```bash
ros2 service call /plan_grasps moveit_msgs/srv/GraspPlanning "{
  group_name: 'fr3_arm',
  target: {
    id: 'cube',
    header: {frame_id: 'base'},
    primitives: [{type: 1, dimensions: [0.05, 0.05, 0.05]}],
    primitive_poses: [{
      position: {x: 0.5, y: 0.0, z: 0.1},
      orientation: {w: 1.0}
    }]
  },
  support_surfaces: ['table']
}"
```

**注意事项：**
- 这个 service 依赖抓取规划插件（grasp planner plugin），不同插件实现差异很大
- 常用插件：`moveit_simple_grasps`、`gpd`（Grasp Pose Detection，基于点云深度学习）
- 在 MoveIt2 中，推荐用 MoveIt Task Constructor（MTC）替代手动调用这个 service
- `candidate_grasps` 为空时，完全由插件生成；提供候选时，插件可以在候选基础上优化

---

## ServoCommandType — 切换 Servo 控制模式

类型：`moveit_msgs/srv/ServoCommandType`

服务路径：`/servo_node/switch_command_type`（需要 Servo 节点运行）

**定位：** 在运行时切换 MoveIt Servo 的控制输入模式。Servo 支持三种控制方式，通过这个 service 可以动态切换，不需要重启 Servo 节点。

**Request：**

```
command_type: int8
```

| 值 | 常量名 | 控制方式 | 输入 topic |
|----|--------|---------|-----------|
| 0 | `JOINT_JOG` | 关节空间速度 | `/servo_node/delta_joint_cmds` |
| 1 | `TWIST` | 末端笛卡尔速度（线速度 + 角速度） | `/servo_node/delta_twist_cmds` |
| 2 | `POSE` | 末端目标位姿（增量） | `/servo_node/pose_target_cmds` |

**Response：** `success: bool`

**示例：切换到笛卡尔速度控制模式**

```bash
ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{
  command_type: 1
}"
```

**示例：切换到关节速度控制模式**

```bash
ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{
  command_type: 0
}"
```

**典型使用场景：**

```
启动时：切换到 TWIST 模式，接收遥操作摇杆输入
接触检测到目标后：切换到 POSE 模式，执行精确对准
```

**注意：**
- 这个 service 只在 MoveIt Servo 节点运行时可用
- 模式切换后立即生效，下一帧控制指令按新模式处理
- Servo 绕过 MoveGroup 规划，直接输出关节速度命令，延迟极低（< 10ms）

---

## GetGroupUrdf — 获取规划组的 URDF

类型：`moveit_msgs/srv/GetGroupUrdf`

**定位：** 返回指定规划组（planning group）的 URDF 子树字符串。用于调试、外部工具集成，或需要在运行时获取机器人几何描述的场景。

**Request：**

```
group_name: string
```

**Response：**

| 字段 | 说明 |
|------|------|
| `error_code` | 错误码 |
| `urdf_string` | URDF XML 字符串（规划组对应的子树） |

**示例：**

```bash
ros2 service call /get_group_urdf moveit_msgs/srv/GetGroupUrdf "{
  group_name: 'fr3_arm'
}"
```

**典型用途：**
- 调试时确认 MoveGroup 加载的 URDF 是否正确
- 外部工具（如可视化工具、碰撞检测库）需要获取机器人几何模型时
- 验证某个规划组包含哪些 link 和 joint

**注意：** 返回的是该规划组对应的 link/joint 子集，不是完整机器人的 URDF。

---

## 三者关系总结

| Service | 所属子系统 | 典型调用时机 |
|---------|-----------|------------|
| GraspPlanning | 抓取规划 | 抓取任务开始时，获取候选抓取位姿 |
| ServoCommandType | MoveIt Servo | Servo 运行中，动态切换操控模式 |
| GetGroupUrdf | MoveGroup 内省 | 调试或外部系统集成时，获取几何描述 |

这三个 service 分属不同子系统，没有直接的调用依赖关系。GraspPlanning 属于抓取流水线，ServoCommandType 专属于 Servo 实时控制子系统，GetGroupUrdf 是纯粹的信息查询接口。
