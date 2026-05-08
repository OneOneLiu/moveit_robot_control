# MoveIt Service 接口总览

MoveIt 提供 25 个 service 接口，分为 7 个功能组。

Service 与 Action 的根本区别：**同步阻塞调用，无 Feedback，适合查询和快速操作。**

---

## 功能分组一览

| 组 | Service | 服务器路径 | 一句话功能 |
|----|---------|-----------|-----------|
| **规划** | GetMotionPlan | `/plan_kinematic_path` | 同步规划，不执行 |
| **规划** | GetCartesianPath | `/compute_cartesian_path` | 多 waypoint 笛卡尔路径 |
| **规划** | GetMotionSequence | `/plan_sequence_path` | 同步序列规划（Pilz） |
| **运动学** | GetPositionIK | `/compute_ik` | 逆运动学求解 |
| **运动学** | GetPositionFK | `/compute_fk` | 正运动学求解 |
| **场景/状态** | ApplyPlanningScene | `/apply_planning_scene` | 持久化修改场景 |
| **场景/状态** | GetPlanningScene | `/get_planning_scene` | 查询当前场景 |
| **场景/状态** | GetStateValidity | `/check_state_validity` | 碰撞检测/状态合法性 |
| **规划器配置** | QueryPlannerInterfaces | `/query_planner_interface` | 查询可用规划器列表 |
| **规划器配置** | GetPlannerParams | `/get_planner_params` | 读取规划器参数 |
| **规划器配置** | SetPlannerParams | `/set_planner_params` | 写入规划器参数 |
| **状态库** | SaveRobotStateToWarehouse | — | 保存机器人状态到库 |
| **状态库** | GetRobotStateFromWarehouse | — | 从库读取机器人状态 |
| **状态库** | ListRobotStatesInWarehouse | — | 列出库中所有状态 |
| **状态库** | CheckIfRobotStateExistsInWarehouse | — | 检查状态是否存在 |
| **状态库** | DeleteRobotStateFromWarehouse | — | 删除库中状态 |
| **状态库** | RenameRobotStateInWarehouse | — | 重命名库中状态 |
| **地图/几何** | LoadMap | `/load_map` | 从文件加载 octomap |
| **地图/几何** | SaveMap | `/save_map` | 保存 octomap 到文件 |
| **地图/几何** | UpdatePointcloudOctomap | — | 用点云更新 octomap |
| **地图/几何** | LoadGeometryFromFile | — | 从 .scene 文件加载碰撞体 |
| **地图/几何** | SaveGeometryToFile | — | 保存碰撞体到 .scene 文件 |
| **其他** | GraspPlanning | — | 生成候选抓取姿态 |
| **其他** | ServoCommandType | — | 切换 Servo 控制模式 |
| **其他** | GetGroupUrdf | — | 获取规划组 URDF |

---

## Service vs Action 选择原则

| 场景 | 用 Service | 用 Action |
|------|-----------|-----------|
| 查询信息（IK/FK/场景） | ✓ | |
| 只规划不执行 | ✓（GetMotionPlan） | ✓（plan_only=true） |
| 规划并执行 | | ✓（MoveGroup） |
| 需要进度反馈 | | ✓ |
| 场景管理 | ✓ | |
| 快速同步调用 | ✓ | |

---

## 文档索引

| 文件 | 内容 |
|------|------|
| `srv_01_planning_services.md` | 规划类 service |
| `srv_02_kinematics_services.md` | IK / FK |
| `srv_03_scene_state_services.md` | 场景与状态管理 |
| `srv_04_planner_config_services.md` | 规划器配置 |
| `srv_05_warehouse_services.md` | 机器人状态库 |
| `srv_06_map_geometry_services.md` | 地图与几何文件 |
| `srv_07_misc_services.md` | GraspPlanning / Servo / GetGroupUrdf |
