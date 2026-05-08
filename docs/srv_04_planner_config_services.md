# 规划器配置 Services

---

## QueryPlannerInterfaces — 查询可用规划器

服务路径：`/query_planner_interface`
类型：`moveit_msgs/srv/QueryPlannerInterfaces`

**定位：** 查询当前 MoveIt 加载了哪些规划管线（pipeline）以及每个管线下有哪些规划器（planner）。用于发现可用规划器，避免填写错误的 `pipeline_id` 或 `planner_id`。

**Request：**

```
pipeline_id: string   # 可选，指定只查询某个管线；空字符串返回所有
```

**Response：**

```
planner_interfaces[]:
  name: string                # 规划器名称（即 planner_id）
  pipeline_id: string         # 所属管线
  planner_ids[]: string[]     # 该管线下所有规划器 ID 列表
```

**示例：查询所有可用规划器**

```bash
ros2 service call /query_planner_interface moveit_msgs/srv/QueryPlannerInterfaces "{
  pipeline_id: ''
}"
```

**示例：只查询 OMPL 管线**

```bash
ros2 service call /query_planner_interface moveit_msgs/srv/QueryPlannerInterfaces "{
  pipeline_id: 'ompl'
}"
```

典型返回示例（Franka FR3）：
```
pipeline_id: ompl
planner_ids: [RRTConnect, RRT, RRT*, PRM, PRM*, LazyPRM, ...]

pipeline_id: pilz_industrial_motion_planner
planner_ids: [PTP, LIN, CIRC]
```

---

## GetPlannerParams — 读取规划器参数

服务路径：`/get_planner_params`
类型：`moveit_msgs/srv/GetPlannerParams`

**定位：** 读取某个规划器当前的参数配置（如 OMPL 的 range、goal_bias 等）。用于调试，或在动态调整参数前先读取当前值。

**Request：**

| 字段 | 说明 |
|------|------|
| `pipeline_id` | 规划管线名，空字符串用默认 |
| `planner_config` | 规划器名（planner_id） |
| `group` | 规划组名，空字符串返回全局默认值 |

**Response：**

```
params:
  keys[]:         string[]   # 参数名列表
  values[]:       string[]   # 对应参数值（字符串形式）
  descriptions[]: string[]   # 参数说明
```

**示例：查询 RRTConnect 对 fr3_arm 组的参数**

```bash
ros2 service call /get_planner_params moveit_msgs/srv/GetPlannerParams "{
  pipeline_id: 'ompl',
  planner_config: 'RRTConnect',
  group: 'fr3_arm'
}"
```

---

## SetPlannerParams — 写入规划器参数

服务路径：`/set_planner_params`
类型：`moveit_msgs/srv/SetPlannerParams`

**定位：** 在运行时动态修改规划器参数，不需要重启 MoveGroup。常用于调优规划性能（如调整 OMPL 的 `range`）或实验不同参数的效果。

**Request：**

| 字段 | 说明 |
|------|------|
| `pipeline_id` | 规划管线名 |
| `planner_config` | 规划器名 |
| `group` | 规划组名，空字符串修改全局默认 |
| `params.keys[]` | 要修改的参数名列表 |
| `params.values[]` | 对应的新值（字符串形式） |
| `replace` | `true`=替换全部参数，`false`=只更新指定的参数 |

**Response：** 无（空 response）

**示例：调整 RRTConnect 的搜索范围**

```bash
ros2 service call /set_planner_params moveit_msgs/srv/SetPlannerParams "{
  pipeline_id: 'ompl',
  planner_config: 'RRTConnect',
  group: 'fr3_arm',
  params: {
    keys: ['range'],
    values: ['0.05']
  },
  replace: false
}"
```

**示例：调整规划尝试次数上限**

```bash
ros2 service call /set_planner_params moveit_msgs/srv/SetPlannerParams "{
  pipeline_id: 'ompl',
  planner_config: 'RRTConnect',
  group: 'fr3_arm',
  params: {
    keys: ['num_planning_attempts'],
    values: ['10']
  },
  replace: false
}"
```

---

## 常用 OMPL 参数参考

| 参数名 | 说明 | 典型值 |
|--------|------|--------|
| `range` | 随机树每步扩展距离 | 0.0（自动）~ 0.1 |
| `goal_bias` | 直接朝目标采样的概率 | 0.05 ~ 0.1 |
| `longest_valid_segment_fraction` | 碰撞检测精度（越小越精但越慢） | 0.005 ~ 0.01 |
| `type` | 规划器类型字符串 | `geometric::RRTConnect` |

修改这些参数后立即对下一次规划生效，重启 MoveGroup 后恢复配置文件默认值。
