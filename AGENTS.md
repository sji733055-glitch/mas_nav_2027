# AGENTS.md

# 项目 Agent 行为准则

本文件是本仓库中 AI Agent、Codex、Claude、superpower skill、多 agent 工作流的统一行为规范。
所有代码修改、审计、重构、调参、文档化任务都必须优先阅读并遵守本文件。

本仓库是 MAS 2027 RoboMaster 哨兵导航仓库，基于 ROS 2 Humble，包含 MID360 驱动、Small Point-LIO 里程计、地形分析、Nav2 导航栈、SLAM Toolbox 二维建图、机器人描述与 Docker 开发环境。仓库中存在已经在容器或实车中验证过的链路，任何修改都必须以“保持可复现、最小改动、可审计”为第一原则。

---

## 0. 最高优先级限制：编译构建限制

**禁止在未经用户明确允许的情况下执行任何编译或构建操作**，包括但不限于：

- `colcon build`
- `cmake` 配置或构建
- `make` / `ninja`
- `docker build` / `docker compose build` / `docker compose up --build`
- `rosdep install`（会触发 apt 安装）
- `apt update` / `apt install`
- 任何其他会触发代码编译、链接、代码生成、镜像构建、依赖安装的命令

在执行任何编译构建命令之前，必须先向用户确认并获得明确许可。

允许在未确认前执行的低风险检查包括：

- `grep` / `rg`
- `find` / `ls` / `du`
- `cat` / `sed` 的只读用法
- `python3` 静态解析脚本
- XML 语法解析（`package.xml`、行为树 XML、插件描述 XML、URDF）
- YAML 语法解析（Nav2、Small Point-LIO、SLAM Toolbox、地形参数）
- `python3 -m py_compile`，用于 launch 文件语法检查
- `git status` / `git diff` / `git log` / `git show`
- 不触发编译、链接、代码生成、镜像构建的静态检查

如果不确定某条命令是否会触发构建，必须先询问用户。

## 0.1 最高优先级限制：禁止版本控制提交操作

**禁止执行任何会改变 Git 历史、Git 索引、远端仓库、分支或标签状态的提交类操作**，无论用户是否要求修改代码，均不得由 Agent 主动提交。

禁止命令包括但不限于：

- `git add`
- `git commit`
- `git commit --amend`
- `git push`
- `git tag`
- `git merge`
- `git rebase`
- `git cherry-pick`
- `git revert`
- `git reset --hard`
- `git checkout -- <file>` / `git restore <file>` 等会覆盖工作区修改的命令
- `git clean -f`
- `gh pr merge`、`gh release create` 等任何会产生提交、合并、发布或远端变更的命令

允许的只读 Git 检查包括：

- `git status`
- `git diff`
- `git log`
- `git show`

Agent 只负责修改工作区文件和说明变更，不负责暂存、提交、推送、打标签或合并。
即使任务已经完成，也只能提醒用户自行检查和提交，不能代替用户执行任何提交操作。

## 0.2 最高优先级限制：禁止 writingplans / 计划说明工具

**禁止使用 `writingplans`、Superpower writing plan、计划说明 skill 或类似工具来生成计划、改造计划、测试计划或阶段说明。**

执行方式要求：

1. 用户与 Agent 已经商讨清楚需求和边界后，直接修改代码或文件。
2. 不要在正式修改前额外输出长篇“计划说明”“改造计划说明”“测试计划说明”。
3. 不要新建 `writingplans` 相关文件。
4. 不要把计划说明作为修改前的阻塞步骤。
5. 必要的任务边界、风险、检查项可以写入最终改造记录或最终回复，但不能调用 writingplans 工作流。
6. 多 Agent 流程只有在用户明确要求时使用，且不得用 writingplans 替代 Explorer / Modifier / Auditor 的实际检查。

允许保留简短的执行摘要和最终检查结果，但不得以 writingplans 形式展开。
核心原则是：**商讨完毕后直接按约束修改代码，修改完成后再给出摘要、检查结果和剩余风险。**

## 0.3 最高优先级限制：删除操作必须先确认

删除文件、目录、包、参数块之前，必须先列出完整清单并获得用户确认，禁止在“顺手清理”的名义下删除。

特别注意：

1. 删除 ROS 包时必须同步检查 `package.xml` 的 `exec_depend` / `depend`、launch 引用、config 引用、Dockerfile 引用。
2. 删除 `third_party/` 下的包时必须确认没有仓库外部包（如 `ros2_comm`）依赖它。
3. 删除参数时必须同步删除 yaml、`declare_parameter`、读取代码和文档说明。
4. 删除完成后必须给出残留引用检查结果。

---

## 1. 项目背景

当前仓库模块（以实际目录为准）：

- `mas2027_nav_bringup/`：统一启动与参数中心。
  - `launch/`：`rm_navigation_small_point_lio_launch.py`（顶层）、`nav2_launch.py`、`mapping_launch.py`、`robot_state_publisher_launch.py`、`rviz_launch.py`
  - `config/`：`nav2_params.yaml`、`small_point_lio_params.yaml`、`slam_toolbox_mapping.yaml`
  - `behavior_trees/`：`navigate_to_pose_w_replanning_and_recovery.xml`、`navigate_through_poses_w_replanning_and_recovery.xml`
  - `map/`：`rmuc`、`rmul`、`kongbai` 的 pgm + yaml
  - `rviz/`：`nav2_default_view.rviz`
- `mas2027_perception/`
  - `mid360_driver/`：MID360 雷达与 IMU 驱动
  - `Odometry/small_point_lio/`：当前唯一里程计实现，发布 `/Odometry`、`/cloud_registered` 和 `odom -> base_link`
- `mas2027_utils/`
  - `terrain_analysis/`、`terrain_analysis_ext/`：地形可通行性分析，输出 `/terrain_map`、`/terrain_map_ext`
  - `pb_nav2_plugins/`：`pb_nav2_costmap_2d::IntensityVoxelLayer` 与 `pb_nav2_behaviors::BackUpFreeSpace`
  - `pointcloud_to_laserscan/`：点云切片转 `/scan`，供 SLAM Toolbox 使用
  - `fake_vel_transform/`：小陀螺场景下的速度坐标转换
  - `loam_interface/`：里程计与点云接口适配
  - `sensor_scan_generation/`：传感器坐标系扫描生成
- `mas2027_robot_description/`：URDF、meshes、`robot_state_publisher` 启动
- `third_party/interfaces/`：整车共用消息定义（`serial_bridge`、`autoaim`、`navigation`、`decision`），被仓库外的 `ros2_comm` 使用
- `Dockerfile` / `docker-compose.yml`：ROS 2 Humble 开发容器，代码挂载到 `/home/ros2_ws/src`
- `README.md`：建图流程说明与项目待办清单

已知历史信息：仓库曾包含 `point_lio`、`small_glim`、`gtsam_points`、`small_gicp`、`common_libs`，已按用户要求移除，里程计链路统一到 Small Point-LIO。不要重新引入这些包，也不要按它们的接口写代码。

这是比赛代码，不是单纯实验仓库。
不要为了“理论更优”而随意改变已经验证过的链路。

---

## 2. 总原则

所有 agent 必须遵守：

1. 不破坏已验证的主链路。
2. 不进行无关重构。
3. 不顺手删除历史逻辑、注释分支或旧方案，除非用户明确要求。
4. 不引入大量新变量、新函数、新模块。
5. 不改变已有 topic、frame_id、参数名、launch 参数名，除非用户明确要求。
6. 不改变模块边界，除非这是本次任务目标。
7. 不添加高频 debug 日志。
8. 不添加无用统计。
9. 不擅自修改 launch 组合方式、QoS、timer、callback group、component container 划分。
10. 不擅自修改比赛参数默认值。
11. 不把未验证推断写成确定事实。
12. 每次非平凡修改必须留下改造记录。
13. 不擅自新增第三方依赖或 Docker 层。

如果用户没有明确要求重构，应优先选择：

```text
最小修改 > 局部修补 > 结构整理 > 大规模重构
```

---

## 3. 用户确认时使用多 Agent 工作流

每次非平凡代码修改，如果用户明确说明使用三阶段流程：

```text
Explorer Agent → Modifier Agent → Auditor Agent
```

多 agent 的目标不是让多个 agent 同时乱改，而是通过角色隔离降低误判风险。
多 agent 记录不得使用 writingplans 生成，也不得以计划说明替代真实仓库检查。

### 3.1 Explorer Agent：探索与事实确认

Explorer Agent 只读仓库，不修改代码。

职责：

1. 找到与用户需求相关的源码、头文件、launch、yaml、URDF、插件描述 XML。
2. 梳理调用链、数据流、topic、TF、参数来源。
3. 区分“当前生效逻辑”和“注释/废弃/历史逻辑”。
4. 标记已验证链路、隐式依赖、潜在冲突。
5. 给 Modifier Agent 明确修改边界。
6. 把探索结果写入改造记录。

必须检查：

- launch 中实际启用的节点，以及 `IfCondition` / `UnlessCondition` 控制的分支。
- `package.xml` 与 `CMakeLists.txt` 中声明的依赖是否与实际 include 一致。
- ROS topic 的发布者、订阅者、QoS、频率限制、remapping。
- TF 链路 `map -> odom -> base_link -> lidar_link` 的发布者，确认只有一个 `odom -> base_link` 发布源。
- 参数是否来自 yaml、`declare_parameter`、硬编码或 launch override。
- 是否有 static、全局变量、类成员状态影响多实例行为。
- Nav2 插件是否在 `nav2_params.yaml` 中被真正加载。
- 是否有用户明确要求不能动的模块。

Explorer Agent 禁止修改文件。

### 3.2 Modifier Agent：最小修改

Modifier Agent 只能根据 Explorer Agent 的记录和用户目标修改代码。

职责：

1. 做最小范围修改。
2. 保留原有行为优先级。
3. 不做用户未要求的清理、重命名、重构。
4. 修改后更新必要注释、参数说明、README 或记录文件。
5. 如果发现 Explorer 结论不完整，先补充记录，不要直接扩大修改范围。

禁止：

- 顺手改其他模块。
- 顺手加调试统计。
- 顺手改参数默认值。
- 顺手删注释分支。
- 顺手重构类结构。
- 顺手改 topic 名、frame_id、QoS、launch 参数名。
- 顺手把逻辑改成“理论上更优”的版本。
- 顺手改 Dockerfile 的基础镜像、镜像源、apt 包列表。

### 3.3 Auditor Agent：审计与验收

Auditor Agent 不相信 Modifier 的自述，必须重新检查。

职责：

1. 查看 `git diff`。
2. 重新 grep 关键变量、节点、topic、参数。
3. 检查是否越界修改。
4. 检查是否破坏原有优先级和数据流。
5. 运行用户允许范围内的静态检查。
6. 若需要编译构建，必须先向用户确认。
7. 给出 `PASS` 或 `NEEDS_FIX`。

Auditor 发现问题后，不要大范围自行改造。
应记录问题，并让 Modifier 进行针对性修复。

---

## 4. 改造记录要求

每次非平凡任务默认需要在修改完成后新建或更新记录文件：

```text
docs/ai_refactor_records/<YYYYMMDD>_<task_name>.md
```

如果用户指定记录文件，则使用用户指定路径。
改造记录是**事后审计材料**，不是 writingplans，也不是修改前计划说明。
不得为了撰写计划、改造计划或测试计划而中断已经确认的代码修改流程；用户与 Agent 商讨完毕后，应直接进行修改，再在完成后补充必要记录。
如果用户明确要求本次不写改造记录，则以用户要求为准。

记录文件必须包含：

```md
# <任务名称> 改造记录

## User Intent

用户原始目标和关键约束。

## Scope

本次允许修改的范围。

## Out of Scope

本次明确不处理的内容。

## Explorer Findings

### Files inspected

### Active logic path

### Data flow

### Risk notes

### Recommended modification boundary

## Modifier Changes

### Files changed

### Key changes

### Behavior preserved

### Behavior intentionally adjusted

### Notes

## Auditor Review

### Checks performed

- [ ] 关键路径检查
- [ ] diff 检查
- [ ] grep 残留引用检查
- [ ] package.xml / CMakeLists.txt 一致性检查
- [ ] launch / yaml / URDF / 插件 XML 检查
- [ ] TF 与 topic 链路检查
- [ ] 用户允许范围内的测试或静态检查
- [ ] 如需构建，已取得用户明确许可

### Issues found

### Final result

PASS / NEEDS_FIX
```

最终回答用户时，必须提供：

1. 修改摘要。
2. 修改文件列表。
3. 检查结果。
4. 改造记录路径。
5. `PASS` 或 `NEEDS_FIX`。
6. 如果未执行构建，必须说明“未执行构建，因为 AGENTS.md 禁止未授权构建”。

---

## 5. 修改前必须建立任务边界

修改前必须在内部明确任务边界；记录文件可在修改完成后补充。不得使用 writingplans 输出独立计划说明：

### 5.1 本次修改属于哪个模块

可选：

- odom / pointcloud（`small_point_lio`、`mid360_driver`）
- terrain（`terrain_analysis`、`terrain_analysis_ext`）
- nav2（`nav2_params.yaml`、行为树、`pb_nav2_plugins`）
- mapping / map（SLAM Toolbox、`pointcloud_to_laserscan`、`map/`）
- description / TF（URDF、`robot_state_publisher`）
- launch / bringup
- utils（`fake_vel_transform`、`loam_interface`、`sensor_scan_generation`）
- interfaces / 通信
- docker / 构建环境
- 文档 / README / 待办

### 5.2 本次修改目标是什么

可选：

- bug 修复
- 策略调整
- 性能优化
- 通信链路改造
- 参数暴露
- 日志/统计
- 清理历史残余
- 文档化

### 5.3 是否允许改变运行行为

必须明确：

- 不允许：只修 bug 或清理边界。
- 小幅允许：保持主逻辑，仅修边界条件。
- 允许：用户明确要求策略变化。

### 5.4 是否涉及已验证链路

如果涉及，必须特别保守，并在记录中写明：

```text
本次修改涉及已验证链路，采用最小改动策略。
```

本仓库当前已验证的链路：

```text
MID360 -> small_point_lio -> /Odometry + /cloud_registered + TF odom->base_link
/cloud_registered -> pointcloud_to_laserscan -> /scan -> slam_toolbox -> /map + TF map->odom
/Odometry + /cloud_registered -> terrain_analysis / terrain_analysis_ext -> /terrain_map + /terrain_map_ext
/terrain_map_ext -> pb_nav2_costmap_2d::IntensityVoxelLayer -> Nav2 costmap
```

---

## 6. Odom / 点云链路修改规范

涉及 `small_point_lio`、`mid360_driver`、点云与里程计输出时必须遵守：

1. 点云链路必须区分原始点云、降采样点云、注册点云 `/cloud_registered`。
2. 修改注册点云输出时，必须确认下游 `pointcloud_to_laserscan`、`terrain_analysis`、`terrain_analysis_ext` 使用的 topic、frame_id、stamp 不变。
3. 不随意改变点云或里程计发布频率。
4. 不在 callback 中打印高频详细日志。
5. 修改去畸变或时间戳逻辑时，必须确认点云时间升序和世界系变换语义。

6. `small_point_lio` 使用 C++20，`3rdparty/` 下有 vendored 代码（如 `ankerl/unordered_dense.h`、从 small_gicp 复制的 `voxelgrid_sampling`），不要把 vendored 代码替换成外部依赖。
7. 修改外参 `extrinsic_R` / `extrinsic_T` 相关逻辑时，必须同时确认 URDF 中 `base_link -> lidar_link` 与 `publish_lidar_tf` 开关，避免出现两个 `odom -> base_link` 发布源。
8. README 中“Small Point-LIO 坐标转换”待办项是当前公认未完成的部分，修改这块时必须逐条对应待办，不要一次性重写整个转换链。
9. 不重新引入 `point_lio`、`small_glim`、`gtsam_points`、`small_gicp`、`common_libs`。

静态检查示例：

```bash
grep -RIn "cloud_registered\|/Odometry\|extrinsic_\|publish_lidar_tf\|frame_id" mas2027_perception/
```

构建前必须先获得用户许可。

---

## 7. 地形分析修改规范

涉及 `terrain_analysis`、`terrain_analysis_ext` 时必须遵守：

1. 两个节点都订阅 `/Odometry` 和 `/cloud_registered`，修改订阅时必须同步确认。
2. 输出 `/terrain_map`、`/terrain_map_ext` 的 `frame_id` 必须保持为 `odom`。
3. 输出字段必须保持 `x/y/z/intensity` 语义，`intensity` 是 Nav2 IntensityVoxelLayer 的判据，不得改变含义。
4. 高度类参数以 `base_link` 为参考；调参前必须先确认 `ground_rel_z = ground_z - base_link_z` 的实测值。
5. 近区合并与远区扩展的分工不得随意调换。
6. 不因为“理论更优”而改变体素分辨率、量程、下采样默认值。
7. 修改后必须说明对 CPU 占用、输出频率和端到端延迟的预期影响。

静态检查示例：

```bash
grep -RIn "terrain_map\|terrainAnalysis\|intensity\|min_height\|max_height\|frame_id" mas2027_utils/terrain_analysis mas2027_utils/terrain_analysis_ext
```

构建前必须先获得用户许可。

---

## 8. Nav2 修改规范

涉及 `nav2_params.yaml`、行为树 XML、`pb_nav2_plugins` 时必须遵守：

1. `nav2_params.yaml` 是 Nav2 行为的唯一来源，修改插件必须同时改 `plugin` 名和对应参数块。
2. 行为树 XML 的 active path 是当前逻辑源头，注释掉的分支不是当前生效逻辑。

3. `ReactiveFallback` 的顺序就是优先级，`ReactiveSequence` 中任何 condition 失败都会阻断后续 action。
4. 修改 `pb_nav2_costmap_2d::IntensityVoxelLayer` 或 `pb_nav2_behaviors::BackUpFreeSpace` 时，必须同步检查 `costmap_plugins.xml`、`behavior_plugin.xml` 与 `nav2_params.yaml` 三处的类名一致。
5. local costmap 使用 `global_frame: odom`、`robot_base_frame: base_link`；global costmap 使用 `global_frame: map`。不得随意调换。
6. 里程计话题使用 `/Odometry`，修改 `controller_server` 的 odom 来源必须说明原因。
7. 不破坏 Nav2 官方功能，不擅自替换官方 launch 或官方插件。
8. 不擅自修改 lifecycle manager 的节点列表和启动顺序。
9. 修改速度限幅、加速度限幅、超时、急停策略时，必须写入改造记录。

静态检查示例：

```bash
grep -RIn "plugin:\|global_frame\|robot_base_frame\|odom_topic\|IntensityVoxelLayer\|BackUpFreeSpace" mas2027_nav_bringup/config mas2027_utils/pb_nav2_plugins
```

构建前必须先获得用户许可。

---

## 9. 建图与地图修改规范

涉及 `mapping_launch.py`、`slam_toolbox_mapping.yaml`、`pointcloud_to_laserscan`、`map/` 时必须遵守：

1. 建图链路是 `/cloud_registered -> pointcloud_to_laserscan -> /scan -> slam_toolbox`，不要改成直接订阅原始点云。
2. 建图期间只允许一个 `map -> odom` 发布器；修改时必须确认没有第二个来源。
3. 点云切片高度参数以 `base_link` 为参考：平地出现在扫描中时提高 `min_height`，低矮障碍消失时降低。
4. 不擅自修改 `resolution`、量程、闭环参数默认值。
5. `map/` 下的 pgm 与 yaml 是版本化产物，禁止 Agent 主动覆盖或删除；需要更新时提示用户用 `map_saver_cli` 自行保存。
6. 修改 SLAM Toolbox 模式（mapping / localization）时必须说明对 TF 与 lifecycle 的影响。

静态检查示例：

```bash
grep -RIn "min_height\|max_height\|resolution\|scan_topic\|mode:\|map_file_name" mas2027_nav_bringup/config/slam_toolbox_mapping.yaml mas2027_nav_bringup/launch/mapping_launch.py
```

构建前必须先获得用户许可。

---

## 10. Launch / ROS 2 通信修改规范

涉及 launch、QoS、component container、remapping 时必须遵守：

1. `rm_navigation_small_point_lio_launch.py` 是顶层入口，`nav2_launch.py`、`mapping_launch.py`、`robot_state_publisher_launch.py`、`rviz_launch.py` 是被 include 的子文件。修改 include 路径时必须确认 `get_package_share_directory` 结果。

2. 顶层 launch 的开关参数（`use_nav2`、`use_terrain_analysis`、`use_rviz`、`use_robot_state_pub`、`use_fake_vel_transform`、`use_ros2_comm`）是对外契约，不得随意改名或改默认值。
3. `ros2_comm` 是仓库外部包，只在 launch 中被引用，不要为它在本仓库新建同名包。
4. 修改 QoS 时必须考虑 `sensor_data`、`volatile`、`keep_last`、`reliable/best_effort` 的影响，大点云与 odom 的取舍不同。
5. 开启 intra-process 不等于零拷贝，必须检查发布端是否使用 `std::unique_ptr`。
6. 修改 component container 划分时，必须说明哪些节点在同一进程，哪些仍跨进程。
7. 修改 remapping 时必须列出改前改后的 topic 对应关系。
8. launch 修改后必须至少做语法检查：`python3 -m py_compile mas2027_nav_bringup/launch/*.py`，并清理生成的 `__pycache__`。

静态检查示例：

```bash
grep -RIn "IncludeLaunchDescription\|DeclareLaunchArgument\|remappings\|parameters\|ComposableNode\|use_intra_process_comms" mas2027_nav_bringup/launch
```

构建前必须先获得用户许可。

---

## 11. TF 与坐标系修改规范

涉及 URDF、`robot_state_publisher`、`fake_vel_transform`、`sensor_scan_generation` 时必须遵守：

1. TF 树必须保持连续：`map -> odom -> base_link -> lidar_link`。
2. 系统中只允许一个节点发布 `odom -> base_link`，只允许一个节点发布 `map -> odom`。
3. `base_link -> lidar_link` 应来自 URDF 的实车标定值，不要用零外参占位覆盖。
4. 修改 URDF 的 link / joint 名称时，必须同步 `nav2_params.yaml`、地形参数、rviz 配置和各节点的 frame 参数。
5. TF 查询失败时不得发布错误 frame 的数据，应使用节流日志。
6. 修改 `fake_vel_transform` 的速度坐标转换时，必须明确输入输出坐标系和是否包含杆臂项。
7. 坐标约定：`W` 为 LIO 内部世界系，`I` 为 IMU，`L` 为 `lidar_link`，`B` 为 `base_link`；`T_B_I = T_B_L * inverse(T_I_L)`。修改转换公式必须与 README 中的约定一致。

静态检查示例：

```bash
grep -RIn "base_link\|lidar_link\|odom\b\|frame_id\|sendTransform\|lookupTransform" mas2027_robot_description mas2027_utils/fake_vel_transform mas2027_utils/sensor_scan_generation
```

构建前必须先获得用户许可。

---

## 12. 参数修改规范

涉及 yaml、`declare_parameter`、默认值时必须遵守：

1. 参数名必须和 yaml 一致。
2. 删除参数时必须同步删除读取代码、默认值、yaml 条目和文档说明。
3. 修改默认值必须说明原因，并写入改造记录。

4. 不擅自把硬编码改为参数，也不擅自把参数改成硬编码。
5. 不保留无意义 fallback；如果用户要求参数必须配置，则缺失时应明确报错。
6. 不擅自删除现有参数的兼容性。
7. `nav2_params.yaml` 使用 `<robot_namespace>` / 替换机制时，必须确认 `RewrittenYaml` 或 `ParameterFile` 的替换字段。

---

## 13. Docker 与构建环境修改规范

涉及 `Dockerfile`、`docker-compose.yml`、`.vscode/` 时必须遵守：

1. 修改 Dockerfile 前必须先向用户确认，因为任何改动都意味着用户需要重建镜像。
2. 不擅自更换基础镜像、中科大镜像源、rosdep 源地址。
3. 新增 apt 包必须说明是哪个包的依赖，并检查是否已被 `ros:humble-desktop-full` 覆盖。
4. 删除 apt 包前必须 grep 全仓库确认没有包在 `package.xml` / `CMakeLists.txt` / 源码中使用它。
5. 修改编译器或 `CC`/`CXX` 环境变量属于高风险操作，必须先确认所有包的 C++ 标准需求。当前各包为 C++17 / C++20，没有使用 C++23 特性。
6. 不擅自改动 `docker-compose.yml` 的挂载路径、`network_mode`、`privileged`、`ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION`。
7. 代码挂载在 `/home/ros2_ws/src`，`build` / `install` / `log` 是 Docker volume，不要假设它们在宿主机可见。
8. 步骤编号变化时要保持 Dockerfile 内注释编号连续。

---

## 14. third_party 与外部依赖规范

1. `third_party/interfaces/` 是整车共用消息包，被仓库外的 `ros2_comm` 使用，禁止以“本仓库没人用”为理由删除。
2. 新增 `third_party/` 子包必须先向用户确认，并说明为什么不能用 apt 或 rosdep 获取。
3. 新增外部依赖必须固定版本，不使用开放版本范围。
4. `mas2027_perception/Odometry/small_point_lio/3rdparty/` 内的 vendored 代码保持原样，不做格式化、不做重构、不升级。
5. `mas2027_planner/` 下现有 `minco_planner`（`nav2_core::GlobalPlanner`）与
   `minco_controller`（`nav2_core::Controller`），均从 navi_minco_bit 移植。
   再新建包前先与用户确认命名和职责。
6. 这两个包随移植带入了三份 vendored 依赖，**保持 vendored，不改为 apt**：

   | 路径 | 体积 | apt 是否有 | 结论 |
   |---|---|---|---|
   | `minco_controller/third_party/qpOASES` | 3.6 MB / 238 文件 | 无（`libqpoases-dev`、`qpoases`、`coinor-libqpoases-dev` 均不存在） | 只能 vendored |
   | `minco_planner/include/cereal` | 1.6 MB / 92 文件 | 有 `libcereal-dev 1.3.1` | 保持 vendored |
   | `minco_planner/include/fmt` | 560 KB / 13 文件 | 有 `libfmt-dev 8.1.1` | 保持 vendored |

   `cereal` / `fmt` 不换成 apt 版的理由：两者都是纯头文件，直接躺在上游包的
   `include/` 树里被相对路径包含，换成系统版要改上游源码的 include 路径；apt 版
   本与上游 vendored 版本号不一定一致，`fmt` 跨大版本有过 API 破坏；而收益只是
   省 2 MB 仓库体积，运行时零收益。按第 3 条"固定版本"的精神，vendored 恰好是
   版本最确定的形态。同时这也避免了给镜像新增 apt 层（见第 13 节第 3 条）。
   qpOASES 没有替代品，属于第 2 条里"不能用 apt 或 rosdep 获取"的情形。

---

## 15. 静态检查与命令限制

允许优先运行：

```bash
git status
git diff
grep -RIn "<keyword>" .
rg "<keyword>"
find . -name "package.xml"
python3 -m py_compile mas2027_nav_bringup/launch/*.py
```

XML 检查可使用：

```bash
python3 - <<'PY'
import xml.etree.ElementTree as ET
from pathlib import Path
for f in sorted(Path(".").rglob("*.xml")):
    if ".git" in f.parts:
        continue
    try:
        ET.parse(f)
        print("OK", f)
    except Exception as e:
        print("FAIL", f, e)
PY
```

YAML 检查可使用 Python 解析，但不得触发构建或代码生成。

禁止在未获用户允许前运行构建命令：

```bash
colcon build
cmake
make
ninja
docker build
docker compose build
docker compose up --build
rosdep install
apt update
apt install
```

无论是否获得构建许可，始终禁止提交类和版本控制写操作：

```bash
git add
git commit
git commit --amend
git push
git tag
git merge
git rebase
git cherry-pick
git revert
git reset --hard
git restore
git clean -f
gh pr merge
gh release create
```

如果用户明确允许构建，构建后必须记录：

1. 构建命令。
2. 构建是否通过。
3. 失败关键日志。
4. 是否是本次修改导致。

---

## 16. 最终输出格式

每次完成后，向用户输出结果摘要；不要输出 writingplans 式计划说明或测试计划说明：

```md
## 修改摘要

## 修改文件

## 行为保持说明

## 有意改变的行为

## 检查结果

## 改造记录

## 未解决问题

## Final Result

PASS / NEEDS_FIX
```

如果没有执行构建，必须明确写：

```text
未执行构建：AGENTS.md 禁止在未获用户明确许可前运行构建命令。
```

---

## 18. 禁止事项汇总

除非用户明确要求，否则禁止：

1. 大规模重构。
2. 删除历史注释分支。
3. 改动多个无关模块。
4. 修改已验证的导航、建图、地形链路。
5. 添加复杂新框架。
6. 添加大量 debug 输出。
7. 修改 topic / frame_id / launch 参数名。
8. 修改参数默认值。
9. 修改 launch 组合方式、QoS、container 划分。
10. 把未验证推断写成确定事实。
11. 未获许可执行构建、镜像重建或 apt 安装。
12. 以“清理”为名删除仍可能复用的逻辑或第三方包。
13. 将单模块任务扩展为全仓库重构。
14. 在未记录的情况下修改行为优先级。
15. 使用 writingplans 或类似计划说明工具。
16. 执行任何 `git add` / `git commit` / `git push` / 合并 / 标签 / 发布等提交类操作。
17. 覆盖或删除 `map/` 下的地图产物。
18. 重新引入 `point_lio`、`small_glim`、`gtsam_points`、`small_gicp`、`common_libs`。

---

## 19. 可选的内部阶段化工作方式

对于复杂任务，可以在内部按阶段拆分，但不得使用 writingplans 输出长篇计划说明：

```text
Step 1：职责边界确认与残余清理
Step 2：odom / 点云链路
Step 3：TF 与坐标转换
Step 4：地形分析输出
Step 5：Nav2 参数与插件接入
Step 6：launch 与参数整理
Step 7：审计与文档更新
```

阶段拆分只用于控制修改风险。
用户与 Agent 已经商讨清楚后，应直接修改代码；完成后再给出摘要、检查结果和风险说明。

---

## 20. 当前项目已知偏好

用户偏好：

1. 最小改动。
2. 少新增变量、函数、类。
3. 不整体重构。
4. 不增加无用调试。
5. 先分析问题，再列约束，再给方案，再给风险和验收。
6. 复杂任务使用 Explorer → Modifier → Auditor 流程。
7. 构建必须先询问用户。
8. 输出应明确哪些行为保持不变、哪些行为被有意改变。
9. 里程计统一使用 Small Point-LIO，不再维护多套 LIO 方案。
10. 对 `small_point_lio`、地形分析、Nav2 的改造必须保持实时性和链路稳定。
11. 不使用 writingplans 做计划说明、改造计划说明或测试计划说明。
12. 禁止任何提交类操作，Agent 只修改工作区文件，不暂存、不提交、不推送。
13. 删除类操作必须先列清单并获得确认，删除后必须给出残留引用检查结果。
14. README 的待办清单是项目进度来源，完成任务时应同步勾选并保留验证结果。
15. 回复使用中文。

