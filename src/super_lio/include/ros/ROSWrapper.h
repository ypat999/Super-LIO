
#ifndef ROSWRAPPER_HPP_
#define ROSWRAPPER_HPP_

#include <map>
#include <tuple>
#include <deque>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <atomic>
#include <execution>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#ifdef LIVOX_SUPPORT
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

#include <std_srvs/srv/trigger.hpp>
#include <pcl_conversions/pcl_conversions.h>

/// msgs
#include "super_lio/msg/cloud_pose.hpp"
#include "super_lio/msg/cloud_pose2.hpp"


#include "lio/params.h"
#include "basic/alias.h"
#include "basic/logs.h"
#include "basic/Manifold.h"
#include "common/ds.h"
#include "common/timer.h"

#include "lio/ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"


namespace LI2Sup{

void LoadParamFromRos(rclcpp::Node& node);

#ifdef LIVOX_SUPPORT
void livox2pcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg, BASIC::CloudPtr& point_cloud);
#endif

class ROSWrapper : public rclcpp::Node {
public:
  /// IMU 输出数据（传递给独立发布线程）
  struct ImuOutputData {
    DynamicState imu_state;
    DynamicState robo_state;
    double timestamp;
  };

  explicit ROSWrapper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ROSWrapper() override;
  using Ptr = std::shared_ptr<ROSWrapper>;
  bool sync_measure(MeasureGroup&);

  void setESKF(ESKF::Ptr& eskf) { eskf_ = eskf;}
  
  // 设置SuperLIO实例的引用
  void setSuperLIO(std::shared_ptr<class SuperLIO> lio) { super_lio_ = lio; }

  // 回调组访问接口
  rclcpp::CallbackGroup::SharedPtr getProcessCallbackGroup() const { return cb_process_; }

  void clear(){
    std::lock_guard<std::mutex> lock(buffers_mutex_);
    lidar_buffer_.clear();
    imu_buffer_.clear();
    lidar_pushed_ = false;
    last_timestamp_imu_ = -1.0;
    last_timestamp_lidar_ = -1.0;
  }

  void pub_odom(const NavState&, const BASIC::V3& body_omega = BASIC::V3::Zero());
  void pub_cloud_world(const BASIC::CloudPtr& pc, double time);
  void pub_cloud_world_undistort_only(const BASIC::CloudPtr& pc, double time, const std::string& lidar_frame);
  void pub_cloud_body(const BASIC::CloudPtr& pc, double time);
  void pub_cloud_undistort_only(const BASIC::CloudPtr& pc, double time, const std::string& lidar_frame);
  void pub_cloud2planner(const BASIC::CloudPtr& pc, double time);
  void pub_cloud_world_pose(const BASIC::CloudPtr& pc, 
                            const NavState& state);
  void pub_cloud_body_pose(const BASIC::CloudPtr& pc, 
                           const NavState& state);
  void pub_cloud_body_pose( const BASIC::VV3& pc_body,
                            const NavState& state);  
  void pub_processing_time(double time, double current_time, double mean_time, double std_time);

  void recordLatency(const std::string& name, double latency_ms);
  void printLatencies();

  void set_global_map(const BASIC::CloudPtr& global_map);

  void set_initial_data(BASIC::SE3& init_pose, bool& flg_get_init_guess, bool flg_finish_init = false);

  rclcpp::CallbackGroup::SharedPtr getSensorCallbackGroup() {
    return cb_sensor_;
  }

  // 保存地图服务回调函数
  void saveMapServiceCallback(const std_srvs::srv::Trigger::Request::SharedPtr request, 
                              const std_srvs::srv::Trigger::Response::SharedPtr response);

private:
  void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg);
#ifdef LIVOX_SUPPORT
  void livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
#endif
  void stdMsgHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void setupParams();
  void setupIO();
  void setupServices();

private:
  rclcpp::CallbackGroup::SharedPtr cb_sensor_;
  rclcpp::CallbackGroup::SharedPtr cb_imu_;
  rclcpp::CallbackGroup::SharedPtr cb_lidar_;
  rclcpp::CallbackGroup::SharedPtr cb_process_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
#ifdef LIVOX_SUPPORT
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_lidar_;
#endif
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_std_;

  // 保存地图服务
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_service_;

  // 参数运行时修改回调句柄
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handler_;

  std::deque<IMUData>   imu_buffer_;
  std::deque<LidarData> lidar_buffer_;
  bool lidar_pushed_ = false;
  double last_timestamp_imu_ = -1.0;
  double last_timestamp_lidar_ = -1.0;

  // 多线程执行器下，IMU/lidar 回调线程与 process 线程并发访问缓冲区的保护锁
  std::mutex buffers_mutex_;

  ESKF::Ptr eskf_{nullptr};
  std::shared_ptr<class SuperLIO> super_lio_{nullptr};

  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped msg2uav_;
  sensor_msgs::msg::PointCloud2 global_map_msg_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  BASIC::V3 last_path_point_ = BASIC::V3(0, 0, -100);

/// 输出。
private:
  void imuOutputThread();

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;       /// lidar fre --> IMU frame
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu_odom_;   /// IMU fre   --> IMU frame
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_robo_odom_;  /// IMU fre   --> Robot frame
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_world_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_body_;

  Timer latency_timer_;

  // IMU 输出线程（将发布从 IMU 回调解耦，避免 WiFi 差时 RELIABLE 发布阻塞）
  std::thread imu_output_thread_;
  std::mutex imu_output_mutex_;
  std::condition_variable imu_output_cv_;
  std::queue<ImuOutputData> imu_output_queue_;
  std::atomic<bool> imu_output_running_{false};
};

} // namespace END.

#endif