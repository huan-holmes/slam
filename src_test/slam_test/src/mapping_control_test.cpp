//*****************************************************************************
// Copyright (c) 2022 MOI SmartRobotTechnology Co.,Ltd.
//*****************************************************************************

#include <ros/ros.h>
#include <common_msg/SlamControl.h>
#include <common_msg/common_type.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapping_control_test");
  ros::NodeHandle nh1("slam");
  ros::ServiceClient slam_client = nh1.serviceClient<common_msg::SlamControl>("mode_cmd");
  common_msg::SlamControl slam_request_data;

  ROS_INFO("mapping control test start!");

  // 1. 获取当前建图状态为空闲
  while (ros::ok()) {
    slam_request_data.request.request_type = common_msg::SlamRequest::SLAM_R_GET;
    ROS_INFO("Requset current mapping status...");
    if (slam_client.call(slam_request_data)) {
      ROS_INFO("Current SLAM State: %s", common_msg::GetSlamModeCStr(slam_request_data));
      if (slam_request_data.response.result_mode != common_msg::SlamMode::SLAM_SLEEPING) {
        ROS_INFO("mapping node is busy now! wait for sleeping");
      } else {
        ros::Duration(5).sleep();
        break;
      }
    } else {
      ROS_WARN("Call SLAM_R_GET failed! after 3s will try again!");
    }
    ros::Duration(3).sleep();
  }

  // 2. 请求开始建图
  while (ros::ok()) {
    slam_request_data.request.request_type = common_msg::SlamRequest::SLAM_R_START;
    ROS_INFO("Requset to start mapping...");
    if (slam_client.call(slam_request_data)) {
      ROS_INFO("Current SLAM State: %s", common_msg::GetSlamModeCStr(slam_request_data));
      if (slam_request_data.response.result_mode != common_msg::SlamMode::SLAM_RUNNING) {
        ROS_INFO("mapping node can not start mapping, please check!");
      } else {
        ros::Duration(5).sleep();
        break;
      }
    } else {
      ROS_WARN("Call SLAM_R_START failed! after 3s will try again!");
    }
    ros::Duration(3).sleep();
  }

  // 3. 请求停止建图
  while (ros::ok()) {
    slam_request_data.request.request_type = common_msg::SlamRequest::SLAM_R_STOP;
    ROS_INFO("Requset to stop mapping...");
    if (slam_client.call(slam_request_data)) {
      ROS_INFO("Current SLAM State: %s", common_msg::GetSlamModeCStr(slam_request_data));
      if (slam_request_data.response.result_mode != common_msg::SlamMode::SLAM_SLEEPING) {
        ROS_INFO("wait mapping node stopped!");
      } else {
        ros::Duration(5).sleep();
        break;
      }
    } else {
      ROS_WARN("Call SLAM_R_STOP failed! after 3s will try again!");
    }
    ros::Duration(3).sleep();
  }

  ROS_INFO("mapping control test finished!");

  return 0;
}