//*****************************************************************************
// Copyright (c) 2022 MOI SmartRobotTechnology Co.,Ltd.
//*****************************************************************************

#include <ros/ros.h>
#include <common_msg/SlamControl.h>
#include <common_msg/LocControl.h>
#include <common_msg/common_type.h>

bool ServiceCallBack(common_msg::SlamControlRequest &req, common_msg::SlamControlResponse &res) {
  printf("Server: query_all_parking_spot_info result is as following: \n");
  res.result_mode = common_msg::SlamMode::SLAM_SLEEPING;
  printf("ParkingSpot.Num = [%d] \n", res.result_mode);
  return true;
}

bool LocServiceCallBack(common_msg::LocControlRequest &req, common_msg::LocControlResponse &res) {
  printf("Server: query_all_parking_spot_info result is as following: \n");
  res.result_mode = common_msg::LocMode::LOC_SLEEPING;
  printf("ParkingSpot.Num = [%d] \n", res.result_mode);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "slam_server_test");
  ros::NodeHandle nh1("slam");
  ros::ServiceServer server = nh1.advertiseService("mode_cmd", ServiceCallBack);
  ros::NodeHandle nh2("loc");
  ros::ServiceServer server2 = nh2.advertiseService("mode_cmd", LocServiceCallBack);
  while (nh1.ok()) {
    ros::spinOnce();
  }
  return 0;
}