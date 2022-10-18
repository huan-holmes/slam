//*****************************************************************************
// Copyright (c) 2022 MOI SmartRobotTechnology Co.,Ltd.
//*****************************************************************************
#pragma once

#include <boost/thread.hpp>

#include <ros/node_handle.h>
#include <std_msgs/UInt16.h>
#include <common_msg/SlamControl.h>
#include <common_msg/common_type.h>
#include <common_msg/Diagnostic.h>

class LaserMapping;

class SlamManager {
 public:
  SlamManager();

  ~SlamManager();

 private:
  bool CommandCallBack(common_msg::SlamControlRequest &req, common_msg::SlamControlResponse &res);
  void EndSlamFunc();

  void KeyInputFunc();
  void PublishDiagnosticInfo();
  void UpdateSlamMode(common_msg::SlamMode mode) { current_slam_mode_ = mode; }
  common_msg::SlamMode GetSlamMode() { return current_slam_mode_; }

 private:
  bool is_simulation_{false};
  ros::NodeHandle node_;

  ros::ServiceServer mapping_service_;
  ros::Publisher diagnostic_pub_;

  // 结束结束建图可能需要比较长的时间，为了避免阻塞，使用单独的线程
  boost::thread end_slam_thread_;
  // 状态锁
  boost::mutex status_mutex_;
  common_msg::SlamMode current_slam_mode_;

  // 键盘监控线程
  boost::thread key_input_thread_;
  boost::atomic<bool> mapping_stopped_{false};

  std::shared_ptr<LaserMapping> laser_mapping_ptr_{nullptr};
};
