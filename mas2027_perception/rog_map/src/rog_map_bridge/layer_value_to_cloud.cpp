// 本仓库新增，不属于上游 ROG-Map。
//
// 作用：把 ProjectionLayer 的二维分类结果翻译成 Nav2 代价地图能读懂的格式，
// 用来替代 terrain_analysis 作为 local_costmap 的障碍来源。
//
//   输入  /rog_map/layer_value   nav_msgs/OccupancyGrid   best_effort + volatile
//   输出  /rog_map/terrain_map   sensor_msgs/PointCloud2  (pcl::PointXYZI)
//
// 为什么需要这一层翻译：
//   pb_nav2_costmap_2d::IntensityVoxelLayer 只接 PointCloud2，并且用 intensity
//   字段做筛选（nav2_params.yaml 里的 min/max_obstacle_intensity）。它读不了
//   OccupancyGrid，而 Nav2 自带的 StaticLayer 也接不了 layer_value：一是 rog_map
//   发 best_effort 而 StaticLayer 默认 reliable，QoS 不兼容；二是 layer_value 的
//   origin 每帧随滑窗移动，StaticLayer 会不停 resize 并搬移整个 costmap。
//
// layer_value 的取值语义（见 rog_map_ros2.hpp 的 fillLayerMaskGrid）：
//   栅格值 = (cell.mask == 0) ? 100 : 0，而 cell.mask 只有 OCCUPIED 才是 0。
//   所以它是严格二值的：100 = OCCUPIED，0 = UNKNOWN / FREE / PASSABLE。
//   UNKNOWN 落到 0 依赖 projection.unknown_as_occupied: False —— 若改成 True，
//   未知区会变成障碍，滑窗边缘和遮挡后方会被整片标记，务必同步确认。
//
// z 和 intensity 都是合成值，不携带真实高度信息：OccupancyGrid 本身没有高度维。
//   分类在 ProjectionLayer 里已经做完了，这里只需要让点落进
//   IntensityVoxelLayer 的体素带和强度窗口内，使其被计为障碍。

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace
{

class LayerValueToCloud : public rclcpp::Node
{
public:
  LayerValueToCloud()
  : rclcpp::Node("layer_value_to_cloud")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/rog_map/layer_value");
    output_topic_ = declare_parameter<std::string>("output_topic", "/rog_map/terrain_map");

    // layer_value 是二值的，默认 100 即"只有 OCCUPIED 参与标记"。
    // 若把输入换成 /rog_map/layer_type（UNKNOWN 0 / FREE 33 / PASSABLE 66 /
    // OCCUPIED 100），改这个阈值就能选择是否让 PASSABLE 也算障碍。
    obstacle_threshold_ = declare_parameter<int>("obstacle_threshold", 100);

    // 必须落在 nav2_params.yaml 的 [min_obstacle_intensity, max_obstacle_intensity]
    // 内，当前是 [0.1, 2.0]。这里是二值分类，强度没有梯度含义，取中间值即可。
    point_intensity_ = declare_parameter<double>("point_intensity", 1.0);

    // 必须落在体素带内：origin_z 0.0 + z_resolution 0.05 × z_voxels 16 = [0.0, 0.8]，
    // 同时要高于 min_obstacle_height 0.0。取 0.2 留足两端余量。
    point_z_ = declare_parameter<double>("point_z", 0.2);

    // 输出帧留空表示沿用输入栅格的 frame_id（rog_map 下是 odom）。
    frame_id_ = declare_parameter<std::string>("frame_id", "");

    // 与 rog_map 发布端一致：best_effort + volatile + depth 1。
    // 不匹配的话订阅建立不起来，收不到任何数据。
    const rclcpp::QoS sub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    // 输出用 reliable：Nav2 观测源按 sensor_data(best_effort) 订阅，
    // reliable 发布方能兼容 best_effort 订阅方，反过来则不行。
    const rclcpp::QoS pub_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, pub_qos);
    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      input_topic_, sub_qos,
      std::bind(&LayerValueToCloud::gridCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "layer_value_to_cloud: %s -> %s (threshold %d, z %.2f, intensity %.2f)",
      input_topic_.c_str(), output_topic_.c_str(), obstacle_threshold_, point_z_, point_intensity_);
  }

private:
  void gridCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
    const size_t width = msg->info.width;
    const size_t height = msg->info.height;
    const double resolution = msg->info.resolution;

    if (width == 0 || height == 0 || resolution <= 0.0) {
      return;
    }
    if (msg->data.size() != width * height) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "layer_value size mismatch: data %zu != %zu x %zu, skipping frame", msg->data.size(), width,
        height);
      return;
    }

    // rog_map 的 origin 始终是轴对齐的（orientation.w = 1）。若上游改了这一点，
    // 下面按轴对齐算出的点位会整体偏移，所以显式提醒而不是静默出错。
    const auto & q = msg->info.origin.orientation;
    if (std::abs(q.z) > 1e-6 || std::abs(q.w - 1.0) > 1e-6) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "layer_value origin is rotated (z=%.4f w=%.4f); cell centers assume axis-aligned origin",
        q.z, q.w);
    }

    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl_cloud.reserve(width * height / 8);

    const double origin_x = msg->info.origin.position.x;
    const double origin_y = msg->info.origin.position.y;

    for (size_t y = 0; y < height; ++y) {
      for (size_t x = 0; x < width; ++x) {
        const int8_t value = msg->data[y * width + x];
        // 负值是 OccupancyGrid 约定的未知。rog_map 不会发，但不能当成障碍。
        if (value < 0 || value < obstacle_threshold_) {
          continue;
        }
        pcl::PointXYZI point;
        point.x = static_cast<float>(origin_x + (static_cast<double>(x) + 0.5) * resolution);
        point.y = static_cast<float>(origin_y + (static_cast<double>(y) + 0.5) * resolution);
        point.z = static_cast<float>(point_z_);
        point.intensity = static_cast<float>(point_intensity_);
        pcl_cloud.push_back(point);
      }
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(pcl_cloud, cloud_msg);
    // 沿用输入时间戳，让 costmap 的 transform_tolerance 判断有意义。
    cloud_msg.header.stamp = msg->header.stamp;
    cloud_msg.header.frame_id = frame_id_.empty() ? msg->header.frame_id : frame_id_;

    // 即使没有障碍也要发空云：让 observation_persistence 正常老化掉上一帧的标记。
    cloud_pub_->publish(cloud_msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "layer_value_to_cloud: %zu obstacle cells of %zu",
      pcl_cloud.size(), width * height);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  int obstacle_threshold_{100};
  double point_intensity_{1.0};
  double point_z_{0.2};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LayerValueToCloud>());
  rclcpp::shutdown();
  return 0;
}
