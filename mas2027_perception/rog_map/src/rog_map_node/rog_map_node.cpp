// ROG-Map 独立宿主节点。
//
// rog_map 上游是纯库包，ROGMapROS 通过 bindNode() 挂到外部节点上，本身不提供
// 可执行文件。本文件是本仓库新增的最小宿主：建节点、加载 `rog_map` 前缀下的参数、
// 构造 ROGMapROS，然后用多线程执行器 spin。
//
// 上游库源码由 CMakeLists 中 src/rog_map/*.cpp 与 src/rog_map_ros/*.cpp 的 glob
// 收集，本目录不在其中，因此这个文件不会混进上游代码，方便后续同步上游。
//
// 输入（默认由 mas2027_nav_bringup/config/rog_map_params.yaml 配置）：
//   /Odometry         nav_msgs/Odometry     small_point_lio，父坐标系 odom
//   /cloud_registered sensor_msgs/PointCloud2  small_point_lio，odom 坐标系
//
// 输出：/rog_map/* 下的可视化点云、OccupancyGrid 与 MarkerArray，详见 README。

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rog_map/rog_map_core/config.hpp>
#include <rog_map_ros/rog_map_ros2.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("rog_map");

  // prior_map 启用且 prior_map_frame != frame_id 时，ROGMapROS 需要共享 TF buffer
  // 来查询二维先验地图变换；未启用时该 buffer 不会被使用。
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node);

  rog_map::Config cfg;
  cfg.loadFromRosNode(node, "rog_map");

  auto rog_map_ros = std::make_shared<rog_map::ROGMapROS>(node, cfg, tf_buffer);

  // ROGMapROS 为 odom、cloud、update 分别建了回调组，必须用多线程执行器，
  // 否则 update timer 会被点云回调阻塞。
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
