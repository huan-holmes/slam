//*****************************************************************************
// Copyright (c) 2022 MOI SmartRobotTechnology Co.,Ltd.
//*****************************************************************************

#ifndef COMMON_TYPE_H_
#define COMMON_TYPE_H_

#include <common_msg/SlamControl.h>
#include <common_msg/LocControl.h>

namespace common_msg {

#define NODE_ID_SLAM 11
#define NODE_ID_LOC 12

static const char *SlamRequestModeCStr[] = {"SLAM_R_GET",
                                            "SLAM_R_STOP",
                                            "SLAM_R_START",
                                            "SLAM_R_CLOSE_LOOP_IN_PLACE",
                                            "SLAM_R_CLOSE_LOOP_AT_SPEC",
                                            "SLAM_R_X_AXIS_SET",
                                            ""};

enum SlamRequest {
  SLAM_R_GET = 0,
  SLAM_R_STOP,
  SLAM_R_START,
  SLAM_R_CLOSE_LOOP_IN_PLACE,
  SLAM_R_CLOSE_LOOP_AT_SPEC,
  SLAM_R_X_AXIS_SET,
};

static const char *LocRequestModeCStr[] = {
    "LOC_R_GET", "LOC_R_STOP", "LOC_R_START", "LOC_MODE_GLOBAL", "RESVERED", "RESVERED", "", ""};

enum LocRequest {
  LOC_R_GET = 0,
  LOC_R_STOP,
  LOC_R_START,
  LOC_MODE_GLOBAL,
};

static const char *SlamModeCStr[] = {"SLAM_SLEEPING",
                                     "SLAM_RUNNING",
                                     "SLAM_UPDATING",
                                     "SLAM_CLOSING_LOOP",
                                     "SLAM_ERROR",
                                     "SLAM_ERROR_ODOM_NOT_EXISTED",
                                     "SLAM_ERROR_ODOM_LOW_FREQ",
                                     "SLAM_ERROR_SCAN_NOT_EXISTED",
                                     "SLAM_ERROR_SCAN_LOW_FREQ"};

enum SlamMode {
  SLAM_SLEEPING = 100,
  SLAM_RUNNING,
  SLAM_UPDATING,
  SLAM_CLOSING_LOOP,
  SLAM_ERROR = 200,
  SLAM_ERROR_ODOM_NOT_EXISTED,
  SLAM_ERROR_ODOM_LOW_FREQ,
  SLAM_ERROR_SCAN_NOT_EXISTED,
  SLAM_ERROR_SCAN_LOW_FREQ,
};

static const char *LocModeCStr[] = {"LOC_SLEEPING",
                                    "LOC_RUNNING",
                                    "LOC_UPDATING",
                                    "LOC_ERROR_MAP_NOT_RECEIVED",
                                    "LOC_ERROR_ODOM_NOT_EXISTED",
                                    "LOC_ERROR_ODOM_LOW_FREQ",
                                    "LOC_ERROR_SCAN_NOT_EXISTED",
                                    "LOC_ERROR_SCAN_LOW_FREQ",
                                    "LOC_ERROR_SCAN_TIME_TOO_LATE"};

enum LocMode {
  LOC_SLEEPING = 100,
  LOC_RUNNING,
  LOC_UPDATING,
  LOC_ERROR = 200,
  LOC_ERROR_MAP_NOT_RECEIVED,
  LOC_ERROR_ODOM_NOT_EXISTED,
  LOC_ERROR_ODOM_LOW_FREQ,
  LOC_ERROR_SCAN_NOT_EXISTED,
  LOC_ERROR_SCAN_LOW_FREQ,
  LOC_ERROR_SCAN_TIME_TOO_LATE,
};

inline const char *GetSlamModeCStr(int x) {
  if (x > SlamMode::SLAM_ERROR) {
    return SlamModeCStr[x - SlamMode::SLAM_ERROR];
  } else {
    return SlamModeCStr[x - SlamMode::SLAM_SLEEPING];
  }
}
inline const char *GetSlamModeCStr(SlamControl &msg) {
  return SlamModeCStr[msg.response.result_mode - SlamMode::SLAM_SLEEPING];
}
inline const char *GetSlamModeCStr(SlamControlResponse &response) {
  return SlamModeCStr[response.result_mode - SlamMode::SLAM_SLEEPING];
}

inline const char *GetLocModeCStr(int x) { return LocModeCStr[x - LocMode::LOC_SLEEPING]; }
inline const char *GetLocModeCStr(LocControl &msg) {
  return LocModeCStr[msg.response.result_mode - LocMode::LOC_SLEEPING];
}
inline const char *GetLocModeCStr(LocControlResponse &response) {
  return LocModeCStr[response.result_mode - LocMode::LOC_SLEEPING];
}

inline const char *GetSlamRequestCStr(int x) { return SlamRequestModeCStr[x - SlamRequest::SLAM_R_GET]; }
inline const char *GetSlamRequestCStr(SlamControl &msg) {
  return SlamRequestModeCStr[msg.request.request_type - SlamRequest::SLAM_R_GET];
}
inline const char *GetSlamRequestCStr(SlamControlRequest &request) {
  return SlamRequestModeCStr[request.request_type - SlamRequest::SLAM_R_GET];
}

inline const char *GetLocRequestCStr(int x) { return LocRequestModeCStr[x - LocRequest::LOC_R_GET]; }
inline const char *GetLocRequestCStr(LocControl &msg) {
  return LocRequestModeCStr[msg.request.request_type - LocRequest::LOC_R_GET];
}
inline const char *GetLocRequestCStr(LocControlRequest &request) {
  return LocRequestModeCStr[request.request_type - LocRequest::LOC_R_GET];
}

}  // namespace common_msg

#endif