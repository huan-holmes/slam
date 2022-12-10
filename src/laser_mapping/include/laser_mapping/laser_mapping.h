//*****************************************************************************
// Copyright (c) 2022 MOI SmartRobotTechnology Co.,Ltd.
//*****************************************************************************

#pragma once

#include <map>
#include <queue>
#include <vector>
#include <string>

// boost
#include <boost/thread.hpp>

// ros
#include <ros/node_handle.h>
// #include <message_filters/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
// #include <tf/message_filter.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <common_msg/common_type.h>
#include <common_msg/RotateMap.h>
#include <glog/logging.h>

// mapping
#include "david_karto/Karto.h"

namespace karto {
class Mapper;
class Dataset;
}  // namespace karto

class SpaSolver;

class LaserMapping {
 public:
  explicit LaserMapping(ros::NodeHandle &nh, bool is_simulation = false);
  ~LaserMapping();

  void Termination();
  void EndSLAM();
  bool GetStatusEnd() const;
  common_msg::SlamMode GetSlamMode() const { return slam_diagnostic_mode_; }

  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
  void OdomCallBack(const nav_msgs::Odometry::ConstPtr &odom_msg);
  int GetLaserBufferSize() const { return (int)laser_scans_buffer_.size(); }

 private:
  bool MapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);
  karto::LaserRangeFinder *GetLaserRangeFinder(const sensor_msgs::LaserScan::ConstPtr &scan);
  bool AddScan(karto::LaserRangeFinder *laser, sensor_msgs::LaserScan::ConstPtr &scan, karto::Pose2 &karto_pose);
  bool PublishRosMap();
  void PublishMapOnce();
  void RotateProcessedScans(double rad);
  void PublishTransform();
  void PublishTFLoop();
  void PublishGraphVisualization();
  void ProcessMainLoop();
  void RotateMapCallback(const common_msg::RotateMap &msg);
  void ControlPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);

 private:
  // ROS handles
  ros::NodeHandle &node_handle_;
  // tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  // message_filters::Subscriber<sensor_msgs::LaserScan> *scan_filter_sub_{nullptr};
  // tf::MessageFilter<sensor_msgs::LaserScan> *scan_filter_{nullptr};

  ros::Publisher map_pub_;
  ros::Publisher map_meta_pub_;
  ros::Publisher markers_pub_;
  ros::Publisher working_state_pub_;
  ros::ServiceServer map_srv_;
  ros::Subscriber odom_sub_;
  ros::Subscriber laser_sub_;
  ros::Subscriber control_pose_sub_;
  // 旋转地图
  ros::Subscriber rotate_map_sub_;

  boost::thread main_process_thread_;

  // 建图生过的地图
  nav_msgs::GetMap::Response map_;

  // Storage for ROS parameters
  std::string odom_frame_;
  std::string map_frame_;
  std::string base_frame_;
  int throttle_scans_;
  double map_update_interval_;
  double resolution_;
  boost::mutex map_mutex_;
  boost::mutex tf_odom2map_mutex_;

  karto::Mapper *mapper_;
  karto::Dataset *dataset_;
  SpaSolver *solver_;
  std::map<std::string, karto::LaserRangeFinder *> lasers_;
  std::map<std::string, bool> lasers_inverted_;

  // karto::Pose2 last_odom_pose_;
  bool is_simulation_{false};
  boost::mutex odom_mtx_;
  std::queue<nav_msgs::Odometry::ConstPtr> odom_pose_buffer_;
  boost::mutex laser_mtx_;
  std::queue<sensor_msgs::LaserScan::ConstPtr> laser_scans_buffer_;

  // 是否已经生成过地图
  bool first_map_updated_flag_{false};
  bool is_map_publishing_ = false;
  ros::WallTime last_map_update_;

  // 接收到激光的次数
  boost::atomic<int> laser_freq_count_;
  // boost::atomic<int> skip_scans_count_;
  // boost::atomic<int> processed_scans_count_;
  // boost::atomic<int> laser_callback_count_;
  double max_process_time_ = 0.0;
  // 发布TF线程
  boost::thread pub_tf_thread_;
  tf::Transform tf_odom2map_;
  unsigned marker_count_;
  // 是否发布建图可视化信息，只有在仿真模式会发布
  bool b_publish_graph_marker_{};
  bool b_exit_pub_tf_thread_;
  double scan_range_max_{};
  double laser_x_offset_, laser_y_offset_, laser_yaw_offset_;
  int keyframe_cnt_ = 0;
  bool slam_stop_request_flag_;
  bool slam_process_end_;
  bool use_imu_correct_;
  // bool laser_first_in_;
  // double current_odom_x_{0}, current_odom_y_{0}, current_odom_yaw_{0};
  double minimum_travel_distance_, minimum_travel_heading_;

  // mode
  common_msg::SlamMode slam_diagnostic_mode_;
};
