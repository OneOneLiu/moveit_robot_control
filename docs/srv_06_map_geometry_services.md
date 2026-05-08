# 地图与几何文件 Services

---

## 概览

这组 service 负责 MoveIt 场景的**持久化 I/O**：把规划场景中的障碍物和环境地图保存到文件，或从文件加载，实现场景在启动之间的复用。

---

## LoadGeometryFromFile — 从文件加载碰撞体

类型：`moveit_msgs/srv/LoadGeometryFromFile`

**定位：** 从 `.scene` 文件加载碰撞几何体（CollisionObjects）到当前规划场景。`.scene` 是 MoveIt 定义的文本格式，描述障碍物形状和位姿。

**Request：**

```
file_path_and_name: string   # .scene 文件的完整路径
```

**Response：** `success: bool`

**示例：**

```bash
ros2 service call /load_scene_geometry moveit_msgs/srv/LoadGeometryFromFile "{
  file_path_and_name: '/ros2_ws/scenes/lab_setup.scene'
}"
```

**.scene 文件格式示例：**

```
scene_name
* table
1
box
0.6 0.0 0.0 0 0 0 1
1.0 0.8 0.05
0
.
```

---

## SaveGeometryToFile — 保存碰撞体到文件

类型：`moveit_msgs/srv/SaveGeometryToFile`

**定位：** 将当前规划场景中的所有碰撞体导出为 `.scene` 文件。

**Request：**

```
file_path_and_name: string
```

**Response：** `success: bool`

**示例：**

```bash
ros2 service call /save_scene_geometry moveit_msgs/srv/SaveGeometryToFile "{
  file_path_and_name: '/ros2_ws/scenes/current_scene.scene'
}"
```

---

## LoadMap — 从文件加载 Octomap

服务路径：`/load_map`
类型：`moveit_msgs/srv/LoadMap`

**定位：** 从文件加载 octomap（体素地图）到规划场景的环境感知层，用于碰撞检测。

**Request：**

```
filename: string   # octomap 文件路径（.bt 或 .ot 格式）
```

**Response：** `success: bool`

**示例：**

```bash
ros2 service call /load_map moveit_msgs/srv/LoadMap "{
  filename: '/ros2_ws/maps/lab_octomap.bt'
}"
```

---

## SaveMap — 保存 Octomap 到文件

服务路径：`/save_map`
类型：`moveit_msgs/srv/SaveMap`

**定位：** 将当前规划场景中的 octomap 保存到文件，用于离线存档或复用。

**Request：**

```
filename: string
```

**Response：** `success: bool`

**示例：**

```bash
ros2 service call /save_map moveit_msgs/srv/SaveMap "{
  filename: '/ros2_ws/maps/snapshot.bt'
}"
```

---

## UpdatePointcloudOctomap — 用点云更新 Octomap

类型：`moveit_msgs/srv/UpdatePointcloudOctomap`

**定位：** 用外部提供的点云数据手动更新 MoveIt 场景中的 octomap，用于不依赖实时传感器的离线场景更新。

**Request：**

```
cloud: sensor_msgs/PointCloud2   # 点云数据
```

**Response：** `success: bool`

**注意：** 在有实时深度传感器的系统中，octomap 通常由 `moveit_ros_perception` 的 occupancy map monitor 自动更新，不需要手动调用这个 service。这个 service 主要用于仿真或批量处理场景。

---

## 使用场景对比

| Service | 数据类型 | 格式 | 典型用途 |
|---------|---------|------|---------|
| LoadGeometryFromFile | 碰撞体（障碍物形状） | `.scene` 文本 | 复用固定的实验台/夹具 |
| SaveGeometryToFile | 碰撞体 | `.scene` 文本 | 保存当前障碍物配置 |
| LoadMap | 体素地图（octomap） | `.bt`/`.ot` 二进制 | 加载扫描好的环境地图 |
| SaveMap | 体素地图 | `.bt`/`.ot` 二进制 | 保存实时扫描的环境 |
| UpdatePointcloudOctomap | 点云 | PointCloud2 | 手动触发 octomap 更新 |

---

## 典型工作流

**场景初始化（每次启动）：**
```
LoadGeometryFromFile  → 加载固定障碍物（桌子、架子等）
LoadMap               → 加载预扫描的环境 octomap
```

**保存当前状态：**
```
SaveGeometryToFile    → 保存手动添加的障碍物
SaveMap               → 保存传感器扫描出的当前 octomap
```
