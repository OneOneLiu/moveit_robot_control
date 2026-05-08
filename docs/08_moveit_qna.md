# MoveIt Q&A

---

## Q1：使用 MoveIt 运动规划能力，是否必须使用这些 action 接口？Python 库本质上也是在调用 action 吗？这是最高效的方法吗？

**Python 库的本质**

`moveit_py`（官方）和 `pymoveit2`（第三方）内部都是：

```
Python 调用 → C++ MoveGroupInterface → /move_action action server
```

`MoveGroupInterface` 只是帮你构造 Goal 消息、发送 action、解析 Result 的 C++ 封装。绕不过 action server，网络/IPC 开销都在。

**真正更底层的方法**

1. **GetMotionPlan Service（比 action 轻量）**
   MoveIt 同时暴露了一个同步 service：
   ```
   moveit_msgs/srv/GetMotionPlan  →  /plan_kinematic_path
   ```
   结构和 MoveGroup action 的 `request` 完全一样，但省掉了 action 的状态机（Goal/Feedback/Result 握手），适合只需要规划但不执行的场景，延迟略低。

2. **直接调用规划管线（C++ 插件 API）**
   完全绕过 MoveGroup 节点，在自己的进程里加载规划管线：
   ```cpp
   #include <moveit/planning_pipeline/planning_pipeline.h>
   planning_pipeline::PlanningPipeline pipeline(robot_model, node, ...);
   pipeline.generatePlan(planning_scene, request, response);
   // 无任何 IPC，直接调用 OMPL/Pilz 插件
   ```
   真正的零 IPC 开销，但需要自己管理 `PlanningScene`、`RobotModel`、轨迹执行，工作量大很多。

3. **直接加载规划器插件（最底层）**
   ```cpp
   planning_interface::PlannerManagerPtr planner;
   planner->getPlanningContext(scene, req, ec)->solve(response);
   ```

4. **MoveIt Servo（完全绕开规划）**
   对于实时速度控制，Servo 根本不规划，直接接受速度指令生成关节命令，延迟 ~10ms 级别，但这是另一种控制模式，不是更高效的规划。

**Action 接口"低效"吗？**

对于运动规划来说，几乎不是瓶颈：

| 环节 | 典型耗时 |
|------|---------|
| Action 协议开销（IPC） | ~1-5 ms |
| OMPL 规划（简单场景） | 100-500 ms |
| OMPL 规划（复杂场景） | 1-10 s |
| Pilz LIN/PTP | 10-50 ms |

真正的瓶颈永远是规划算法本身，不是通信协议。

**总结**

| 方法 | IPC 开销 | 实现复杂度 | 适用场景 |
|------|---------|-----------|---------|
| Action（MoveGroup 等） | 有（ms 级） | 低 | 绝大多数场景 |
| GetMotionPlan service | 有（略低） | 低 | 只规划不执行 |
| C++ 规划管线直调 | 无 | 高 | 极低延迟/高频规划需求 |
| C++ 规划器插件直调 | 无 | 极高 | 定制化研究场景 |

除非需要毫秒级延迟的高频规划，或者在同一进程内集成 MoveIt，action 接口就是最务实的选择——不是因为没有更底层的路，而是那条路的收益在大多数场景下不值得付出的工程代价。

---

## Q2：跟 MoveIt 交互，用这些 action + service 接口是否已经覆盖了绝大部分场景？有没有更底层更高效的方式？

是的，这套接口已经覆盖了实际项目中绝大部分场景。但确实存在更底层的方式，代价是开发复杂度显著提升。

| 层次 | 方式 | 效率 | 开发成本 |
|------|------|------|---------|
| 高层封装 | Python `moveit_py` / C++ `MoveGroupInterface` | 最低 | 最低，推荐用于业务逻辑 |
| 中间层 | action + service 接口（本文档体系） | 中等 | 中等，适合精细控制 |
| 底层 | C++ 直接调用 `planning_pipeline`（同进程） | 最高（零 IPC） | 高，必须在 MoveGroup 同一进程内 |
| 实时层 | MoveIt Servo（直接输出关节速度，不经规划） | 最高（< 10ms） | 中等，仅适合速度控制场景 |

关键结论：
- `moveit_py` / `MoveGroupInterface` **本质上是封装这些 action/service**，不存在"绕过接口"的捷径
- 真正更底层的方式是在 C++ 插件或组件内直接调用规划管线，零 IPC，但代码必须与 MoveGroup 同进程
- 运动规划的延迟主要来自**规划计算本身**（OMPL 几百毫秒到数秒），通信开销（1-5ms）可以忽略
- 追求实时性用 Servo，追求规划能力用这套 action/service 体系，两者目标不同

**结论：这套 action + service 体系是 90% 场景的最佳切入点。** 更底层的方式只在有充分理由时（同进程插件开发、极致延迟要求）才值得考虑。

---

## Q3：action 和 service 里都有规划/运动学功能，它们的底层实现是同一个吗？

**结论先行：是的，共享同一套底层实现。**

MoveGroup 内部架构如下：

```
                    ┌─────────────────────────────────────────┐
                    │             MoveGroup 节点               │
                    │                                         │
  action call  ──→  │  ┌──────────────────────────────────┐   │
  /move_action      │  │     MoveGroupMoveAction          │   │
                    │  └──────────────┬───────────────────┘   │
  service call ──→  │  ┌─────────────┴────────────────────┐   │
  /plan_kinematic   │  │    GetMotionPlanService 等        │   │
  /compute_ik       │  └──────────────┬───────────────────┘   │
  /compute_fk       │                 │                        │
                    │        ┌────────▼────────┐               │
                    │        │  共享能力层      │               │
                    │        └────────┬────────┘               │
                    │                 │                         │
                    │   ┌─────────────┼──────────────────┐     │
                    │   ▼             ▼                   ▼     │
                    │ KinematicsBase  PlanningPipeline  PlanningScene │
                    │ (IK/FK 插件)   (OMPL/Pilz)      (碰撞场景)    │
                    └─────────────────────────────────────────┘
```

**具体对应关系：**

| 你调用的接口 | 在 MoveGroup 内部实际调用 |
|------------|------------------------|
| `MoveGroup action` → 规划 | `PlanningPipeline::generatePlan()` |
| `GetMotionPlan service` → 规划 | 同上，完全一样 |
| `GetPositionIK service` | `KinematicsBase::searchPositionIK()`（IK 插件） |
| `GetPositionFK service` | `RobotState::getGlobalLinkTransform()`（URDF 模型计算） |
| `MoveGroup action` 内部 → IK | 同上，MoveGroup 规划时自动调用 IK |

**你说的场景完全正确：**

当你通过 action 发起笛卡尔坐标目标的规划时，MoveGroup 内部流程：

```
action goal (目标位姿)
    → MoveGroupMoveAction::goalCallback()
    → PlanningPipeline::generatePlan()
        → OMPL/Pilz 规划器
            → KinematicsBase::searchPositionIK()   ← 和 /compute_ik 调用的是同一个插件
            → RobotState::getGlobalLinkTransform()  ← 和 /compute_fk 调用的是同一套代码
    → 返回轨迹
```

**那为什么还要单独暴露 IK/FK service？**

action 是把"目标 → 规划 → 执行"打包成一个黑盒。单独的 `compute_ik` / `compute_fk` service 是把这个黑盒**拆开**，让你可以：
- 在不启动完整规划的情况下验证某个位姿**是否可达**（IK 有解）
- 离线对任意关节配置计算末端位置（FK），不需要机器人当前处于那个状态
- 调试：确认规划失败是因为 IK 无解，还是规划器找不到无碰撞路径

**SetPlannerParams 和 action 里指定 planner_id 的区别：**

- `SetPlannerParams`：修改的是**持久化配置**，重启前一直有效，所有后续规划都使用新参数
- action goal 里的 `planner_id`：只影响**这一次规划请求**

两者最终都是传递给同一个规划器插件实例。

---

## Q4：action 和 service 都能设置规划场景，两者是共通的吗？预先用 service 设置的场景，action 里可以缺省吗？

**先核实：action 里确实有规划场景字段。**

在 `moveit_msgs/action/MoveGroup` 的 Goal 里：

```
planning_options:
    PlanningScene planning_scene_diff   # 在这里
    bool plan_only
    bool replan
    ...
```

**两者的本质区别：持久 vs 临时**

MoveGroup 节点内部维护一份**全局规划场景（Master Scene）**，这是所有规划请求都会使用的基准场景。

```
┌────────────────────────────────────────────────────┐
│         MoveGroup 内部：全局规划场景（持久）          │
│   table, shelf, robot state, ACM, octomap...        │
└─────────────────────┬──────────────────────────────┘
                      │
       ┌──────────────┼──────────────┐
       ▼              ▼              ▼
  规划请求 A      规划请求 B      规划请求 C
  (无 diff)    (有 planning_   (有 planning_
               scene_diff:     scene_diff:
               临时加一个箱子)  临时移除一个障碍)
       │              │              │
       ▼              ▼              ▼
  用全局场景      全局场景 +      全局场景 +
                 临时箱子        临时移除
              (仅本次规划有效)  (仅本次规划有效)
```

| 方式 | 作用范围 | 持久性 | 字段位置 |
|------|---------|--------|---------|
| `ApplyPlanningScene` service | 修改**全局规划场景** | 持久，直到下次修改 | `/apply_planning_scene` |
| `GetPlanningScene` service | 读取全局规划场景 | 只读 | `/get_planning_scene` |
| action 里的 `planning_scene_diff` | 对**本次规划请求**的临时叠加 | 仅本次有效，不修改全局场景 | `planning_options.planning_scene_diff` |

**你的理解基本正确，但有一个细节需要明确：**

> "用 service 设置好场景后，action 里的场景部分可以缺省"

✅ **是的**，如果你用 `ApplyPlanningScene` 把场景设置好了，action 里 `planning_scene_diff` 留空（或 `is_diff: true` 但内容为空），规划器就会直接使用全局场景。完全不需要每次 action 都重复描述场景。

**`planning_scene_diff` 里的 `is_diff` 字段含义：**

- `is_diff: true`（增量模式）：在全局场景基础上**叠加**这次请求里的改动，两者合并后用于规划
- `is_diff: false`（全量替换模式）：**完全忽略**全局场景，只用这次请求里提供的场景规划（规划完后全局场景不变）

**典型工作流建议：**

```
启动时：
  ApplyPlanningScene  → 加载固定障碍物（桌子、架子），持久有效

每次规划任务：
  MoveGroup action (planning_scene_diff 留空或 is_diff:true 内容为空)
  → 自动使用全局场景，无需重复描述

特殊情况（临时障碍）：
  MoveGroup action (planning_scene_diff: {is_diff: true, 临时箱子})
  → 本次规划考虑临时箱子，但全局场景不变，下次规划不受影响
```

**两者会互相覆盖吗？**

不会互相覆盖。`planning_scene_diff` 不会写回全局场景，全局场景只能通过 `ApplyPlanningScene` service（或 `/planning_scene` topic）修改。两者是完全独立的读写路径：

```
写全局场景：ApplyPlanningScene service  ──→  全局 Master Scene
读全局场景：GetPlanningScene service    ←──  全局 Master Scene

action 规划时：
  全局 Master Scene + planning_scene_diff（临时）→ 合并 → 用于规划 → 规划完丢弃 diff
```

---
