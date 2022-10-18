//*****************************************************************************
// Copyright (c) 2022 MOI SmartRobotTechnology Co.,Ltd.
//*****************************************************************************

#include <ros/ros.h>
#include <common_msg/SlamControl.h>
#include <common_msg/common_type.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "slam_client_test");
  ros::NodeHandle nh1("slam");
  ros::ServiceClient slam_client = nh1.serviceClient<common_msg::SlamControl>("mode_cmd");
  common_msg::SlamControl slam_request_data;
  slam_request_data.request.request_type = common_msg::SlamRequest::SLAM_R_GET;
  if (slam_client.call(slam_request_data)) {
    ROS_INFO("Current SLAM State: %s", common_msg::GetSlamModeCStr(slam_request_data));
  }

  ros::NodeHandle nh2("loc");
  ros::ServiceClient loc_client = nh2.serviceClient<common_msg::LocControl>("mode_cmd");
  common_msg::LocControl loc_request_data;
  loc_request_data.request.request_type = common_msg::LocRequest::LOC_R_GET;
  if (loc_client.call(loc_request_data)) {
    ROS_INFO("Current LOC State: %s", common_msg::GetLocModeCStr(loc_request_data));
  }

  return 0;
}