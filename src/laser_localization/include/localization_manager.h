//*****************************************************************************
// Copyright (c) 2022 MOI SmartRobotTechnology Co.,Ltd.
//*****************************************************************************

#pragma once

#include <boost/thread.hpp>

#include <ros/node_handle.h>
#include <common_msg/LocControl.h>
#include <common_msg/common_type.h>
#include <common_msg/Diagnostic.h>

class PFLocalizationNode;

class LocalizationManager {
 public:
  LocalizationManager();
  ~LocalizationManager();

 private:
  bool CommandCallBack(common_msg::LocControlRequest &req,
                       common_msg::LocControlResponse &res);
  void EndLocFunc();
  void UpdateLocMode(common_msg::LocMode mode) { cur_loc_mode_ = mode; }
  common_msg::LocMode GetLocMode() const { return cur_loc_mode_; }

 private:
  void PublishDiagnosticInfo();

  bool is_simulation_{false};
  ros::NodeHandle node_;

  ros::ServiceServer localization_service_;
  ros::Publisher loc_diagnostic_pub_;
  // 结束定位可能需要比较长的时间，为了避免阻塞，使用单独的线程
  boost::thread end_loc_thread_;

  // 状态锁
  boost::mutex status_mutex_;
  common_msg::LocMode cur_loc_mode_;
  std::shared_ptr<PFLocalizationNode> pf_loc_ptr_{nullptr};
};
