# Servo Launch 调试记录

## 问题背景

`fr3_servo.launch.py` 使用 `MoveItConfigsBuilder` 构建配置，但包的目录结构与 `MoveItConfigsBuilder` 的预期不一致，导致连锁错误。

---

## 问题一：SRDF xacro 未安装

**错误**

```
ParameterBuilderFileNotFoundError: ".../moveit_config/fr3.srdf.xacro doesn't exist"
```

**原因**  
`setup.py` 只安装了 `moveit_config/*.yaml`，`.xacro` 文件被排除在外。`MoveItConfigsBuilder` 所有路径都指向 **install 目录**，不读 src。

**修复** (`setup.py`)

```python
# 改前
glob('moveit_config/*.yaml')
# 改后
glob('moveit_config/*.yaml') + glob('moveit_config/*.xacro')
```

修改后需重新构建：

```bash
colcon build --packages-select moveit_robot_control && source install/setup.bash
```

---

## 问题二：MoveItConfigsBuilder 路径默认值陷阱

**现象**  
`to_moveit_configs()` 会自动补全所有未显式设置的配置项，每项默认从 `config/` 目录查找。但本包的 MoveIt 配置放在 `moveit_config/` 下，导致：


| 自动调用的方法                          | 默认查找路径                              | 实际文件位置           | 结果     |
| -------------------------------- | ----------------------------------- | ---------------- | ------ |
| `robot_description_kinematics()` | `config/kinematics.yaml`            | `moveit_config/` | 找不到    |
| `trajectory_execution()`         | `config/*_controllers.yaml`         | `moveit_config/` | 警告后跳过  |
| `pilz_cartesian_limits()`        | `config/pilz_cartesian_limits.yaml` | 不存在              | **崩溃** |


Pilz 问题的触发条件：pipelines 列表里包含 `pilz_industrial_motion_planner` 时，`to_moveit_configs()` 必然调用 `pilz_cartesian_limits()`。

**修复** (`fr3_servo.launch.py`)  
对所有非标准路径显式指定，并移除没有对应 yaml 的 pipeline：

```python
moveit_config = (
    MoveItConfigsBuilder("fr3", package_name="moveit_robot_control")
    .robot_description(file_path="urdf/fr3.urdf.xacro")
    .robot_description_semantic(file_path="moveit_config/fr3.srdf.xacro")
    .robot_description_kinematics(file_path="moveit_config/kinematics.yaml")
    .joint_limits(file_path="moveit_config/fr3_joint_limits.yaml")
    .trajectory_execution(file_path="moveit_config/fr3_controllers.yaml")
    .planning_pipelines(pipelines=["ompl"])
    .to_moveit_configs()
)
```

> **注意**：`planning_pipelines()` 的 yaml 路径硬编码为 `config/`，无法覆盖。
> 自定义的 `moveit_config/ompl_planning.yaml` 不会被 `MoveItConfigsBuilder` 读取，
> 会使用 MoveIt 默认配置。需要自定义 OMPL 参数时，改用 `fr3_moveit.launch.py`。

---

## 问题三：WARNING 日志（无害）

```
WARNING: Cannot infer URDF from `.../share/moveit_robot_control`. -- using config/fr3.urdf
WARNING: Cannot infer SRDF from `.../share/moveit_robot_control`. -- using config/fr3.srdf
```

`MoveItConfigsBuilder.__init__()` 在构建时先做自动推断（找 `.setup_assistant` 或 `config/fr3.urdf.xacro`），推断失败就打这两行 WARNING 并设 fallback 路径。但只要后续显式调用了 `.robot_description(file_path=...)` 和 `.robot_description_semantic(file_path=...)`，fallback 就完全被忽略。**警告无需处理。**

---

## 问题四：moveit_servo 未安装

**错误**

```
Could not find requested resource in ament index
Failed to load node 'servo_node' of type 'moveit_servo::ServoNode'
```

**原因**  
`moveit_servo` 包（提供 `ServoNode` 组件）未安装，只有 `moveit_msgs`（消息定义）被安装。

**修复**

```bash
apt install ros-jazzy-moveit-servo
```

