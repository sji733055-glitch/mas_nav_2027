# mas_nav_2027

# 开发环境配置
1. docker 安装
```bash
// docker 安装脚本
export DOWNLOAD_URL="https://mirrors.tuna.tsinghua.edu.cn/docker-ce"
wget -O- https://raw.githubusercontent.com/docker/docker-install/master/install.sh | sh
// 更换为国内docker镜像源
bash <(wget -qO- https://xuanyuan.cloud/docker.sh)
// 权限处理
sudo usermod -aG docker mas
```
2. 构建容器
```bash
sudo docker compose up -d --build
```
3. 编译
```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 4
```

4. 在宿主机内使用解决X11授权
```bash
xhost +si:localuser:root
```

## SLAM Toolbox 二维建图

建图链路使用 Small Point-LIO 的 `/Odometry` 和 `/cloud_registered`：

```text
/cloud_registered -> pointcloud_to_laserscan -> /scan -> slam_toolbox
/Odometry + TF: map -> odom -> base_link
```

启动建图：

```bash
ros2 launch mas2027_nav_bringup mapping_launch.py
```

建图期间不要启动其他 `map -> odom` 发布器。检查数据：

```bash
ros2 topic hz /scan
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
```

保存 Nav2 使用的二维地图：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/ros2_ws/src/mas2027_nav_bringup/map/field
```

同时保存 SLAM Toolbox 位姿图，供后续 localization 模式使用：

```bash
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/ros2_ws/src/mas2027_nav_bringup/map/field'}"
```

点云切片高度、量程、闭环与分辨率参数位于
`mas2027_nav_bringup/config/slam_toolbox_mapping.yaml`。高度参数以
`base_link` 为参考，平地出现在扫描中时提高 `min_height`，低矮障碍消失时降低。

# 项目待办

待办按依赖顺序推进。完成任务时同步勾选，并在提交或测试记录中保留验证结果。

## 当前进度

- [x] MID360 驱动接入 Small Point-LIO。
- [x] Small Point-LIO 发布 `/Odometry`、`/cloud_registered` 和 `odom -> base_link`。
- [x] 接入 `terrain_analysis`、`terrain_analysis_ext` 参数、节点和话题重映射。
- [x] 在地形参数中补充坐标与高度调参方法。
- [ ] 在 Docker/ROS 2 环境完成地形链路编译和运行验证。

## Small Point-LIO 坐标转换

坐标约定：`W` 为 LIO 内部世界系，`I` 为 IMU，`L` 为 `lidar_link`，`B` 为 `base_link`。`T_I_L` 来自 `extrinsic_R/T`，`T_B_L` 来自 URDF，机体到 IMU 的外参为 `T_B_I = T_B_L * inverse(T_I_L)`。

- [ ] 明确记录 `position`、`orientation`、`velocity`、`angular_velocity` 的坐标系语义。
- [ ] 从 Small Point-LIO 核心暴露当前 `T_I_L`，并兼容在线外参估计。
- [ ] 从 TF 查询真实 `T_B_L`，组合得到 `T_B_I`。
- [ ] 转换机体位姿：`T_odom_base = T_B_I * T_W_I * inverse(T_B_I)`。
- [ ] 使用 `T_B_I` 将内部注册点云从 `W` 转换到外部 `odom`。
- [ ] 转换线速度：`v_I = R_W_I.transpose() * v_W`。
- [ ] 转换角速度：`omega_B = R_B_I * omega_I`。
- [ ] 加入杆臂项：`v_B = R_B_I * v_I + t_B_I.cross(omega_B)`。
- [ ] 将 `v_B`、`omega_B` 写入 `/Odometry.twist`。
- [ ] 填写合理的 `pose.covariance` 和 `twist.covariance`。
- [ ] 删除或更新 `small_point_lio_node.cpp` 中过时的坐标转换 TODO。
- [ ] TF 查询失败时不发布错误 frame 数据，并使用节流日志。
- [ ] 核对 URDF 中 `base_link -> lidar_link` 是否来自实车标定。
- [ ] 使用统一启动文件时关闭零外参发布：`publish_lidar_tf:=false`。
- [ ] 确认系统中只有一个节点发布 `odom -> base_link`。
- [ ] 完成单位外参、非零平移、非零旋转和 TF 缺失单元测试。
- [ ] 完成静止、直行、横移和原地旋转实车测试。
- [ ] 验证 `/Odometry.pose`、TF 和 `/cloud_registered` 在同一时间戳下对齐。
- [ ] 使用 ROS bag 验证输出稳定且可重复。

## 地形分析验证

- [ ] 确认 `/terrainAnalysis` 和 `/terrainAnalysisExt` 正常启动并加载 YAML 参数。
- [ ] 确认两个节点订阅 `/Odometry` 和 `/cloud_registered`。
- [ ] 确认 `/terrain_map`、`/terrain_map_ext` 持续发布且 `frame_id == odom`。
- [ ] 检查输出包含 `x/y/z/intensity`，无 NaN 或异常空帧。
- [ ] 在平地测量 `ground_rel_z = ground_z - base_link_z` 并调整高度参数。
- [ ] 完成平地、台阶、斜坡、负障碍、低顶棚和动态行人测试。
- [ ] 验证 `terrain_map_ext` 的近区合并和远区扩展逻辑。
- [ ] 记录 CPU、内存、输出频率和端到端延迟。

## Nav2 基础接口

- [x] 确定使用 Small Point-LIO 转换后的速度作为 Nav2 里程计反馈。
- [ ] 确认底盘控制器订阅的话题、消息类型和速度单位。
- [ ] 打通 Nav2 `/cmd_vel` 到底盘控制器的指令链路。
- [ ] 配置急停、速度限幅、加速度限幅和指令超时。
- [ ] 确定 `base_link`、`base_footprint` 和机器人 footprint。
- [ ] 确认 TF 树连续包含 `odom -> base_link -> lidar_link`。

## 局部 Nav2

- [ ] 新建 Nav2 参数和启动文件。
- [ ] 配置 `controller_server`，里程计话题使用 `/Odometry`。
- [ ] 配置 `planner_server`、`behavior_server`、`bt_navigator` 和 lifecycle manager。
- [ ] local costmap 使用 `global_frame: odom`、`robot_base_frame: base_link`。
- [ ] 接入 `pb_nav2_costmap_2d::IntensityVoxelLayer`，订阅 `/terrain_map_ext`。
- [ ] 配置 inflation layer、footprint clearing 和障碍清除策略。
- [ ] 完成静态目标、路径跟踪、转弯、绕障和恢复行为测试。

## 地图与全局定位

- [ ] 确定全局定位方案：AMCL、SLAM Toolbox 或点云重定位。
- [ ] 准备并版本化二维地图或点云地图。
- [ ] 提供稳定且唯一的 `map -> odom` TF。
- [ ] global costmap 使用 `global_frame: map`。
- [ ] 配置 map server、定位节点及其 lifecycle manager。
- [ ] 验证初始位姿、重定位、断连恢复和长期漂移。
- [ ] 完成跨区域全局规划与动态重规划测试。

## 整车联调与可靠性

- [ ] 录制传感器、LIO、TF、地形图、Nav2 状态和控制指令标准数据包。
- [ ] 建立可重复的离线回放测试流程。
- [ ] 检查所有节点的 QoS、时间戳、时钟源和 namespace。
- [ ] 检查启动顺序、节点退出、自动重启和 lifecycle 故障恢复。
- [ ] 测试无雷达、无 IMU、TF 缺失、定位丢失和控制器失联场景。
- [ ] 完成 CPU、内存和网络带宽压力测试。
- [ ] 固化实车参数、base_link <-> lidar_link启动命令和故障排查文档。
- [ ] 在干净 Docker 环境完成全量构建和端到端验收。

## 待决策项

- [ ] 全局定位方案。
- [ ] Nav2 控制器和规划器插件。
- [ ] 底盘最终 `/cmd_vel` 接口及限速策略。
- [ ] 是否保留在线 LiDAR-IMU 外参估计能力。
```