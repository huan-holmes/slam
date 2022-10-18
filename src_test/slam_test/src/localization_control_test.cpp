//*****************************************************************************
// Copyright (c) 2022 MOI SmartRobotTechnology Co.,Ltd.
//*****************************************************************************

#include <ros/ros.h>
#include <common_msg/LocControl.h>
#include <common_msg/common_type.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "localization_control_test");
  ros::NodeHandle nh1("loc");
  ros::ServiceClient loc_client = nh1.serviceClient<common_msg::LocControl>("mode_cmd");
  common_msg::LocControl loc_request_data;

  ROS_INFO("localization control test start!");

  // 1. 获取当前定位状态为空闲
  while (ros::ok()) {
    loc_request_data.request.request_type = common_msg::LocRequest::LOC_R_GET;
    ROS_INFO("Requset current localization status...");
    if (loc_client.call(loc_request_data)) {
      ROS_INFO("Current LOC State: %s", common_msg::GetLocModeCStr(loc_request_data));
      if (loc_request_data.response.result_mode != common_msg::LocMode::LOC_SLEEPING) {
        ROS_INFO("localization node is busy now! wait for sleeping");
      } else {
        ros::Duration(5).sleep();
        break;
      }
    } else {
      ROS_WARN("Call LOC_R_GET failed! after 3s will try again!");
    }
    ros::Duration(3).sleep();
  }

  // 2. 请求开始定位
  while (ros::ok()) {
    loc_request_data.request.request_type = common_msg::LocRequest::LOC_R_START;
    ROS_INFO("Requset to start localization...");
    if (loc_client.call(loc_request_data)) {
      ROS_INFO("Current LOC State: %s", common_msg::GetLocModeCStr(loc_request_data));
      if (loc_request_data.response.result_mode != common_msg::LocMode::LOC_RUNNING) {
        ROS_INFO("localization node can not start localization, please check!");
      } else {
        ros::Duration(5).sleep();
        break;
      }
    } else {
      ROS_WARN("Call LOC_R_START failed! after 3s will try again!");
    }
    ros::Duration(3).sleep();
  }

  // 3. 请求停止定位
  while (ros::ok()) {
    loc_request_data.request.request_type = common_msg::LocRequest::LOC_R_STOP;
    ROS_INFO("Requset to stop localization...");
    if (loc_client.call(loc_request_data)) {
      ROS_INFO("Current LOC State: %s", common_msg::GetLocModeCStr(loc_request_data));
      if (loc_request_data.response.result_mode != common_msg::LocMode::LOC_SLEEPING) {
        ROS_INFO("wait localization node stopped!");
      } else {
        ros::Duration(5).sleep();
        break;
      }
    } else {
      ROS_WARN("Call LOC_R_STOP failed! after 3s will try again!");
    }
    ros::Duration(3).sleep();
  }

  ROS_INFO("localization control test finished!");

  return 0;
}