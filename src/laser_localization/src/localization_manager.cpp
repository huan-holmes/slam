//*****************************************************************************
// Copyright (c) 2022 MOI SmartRobotTechnology Co.,Ltd.
//*****************************************************************************

#include "localization_manager.h"

#include "nlohmann/json.hpp"

#include "mrpt_localization_node.h"
#include "spd_logger.h"

// 系统相关

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#include <fstream>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#endif

LocalizationManager::LocalizationManager() : node_("~") {
  localization_service_ =
      node_.advertiseService<common_msg::LocControlRequest,
                             common_msg::LocControlResponse>(
          "mode_cmd",
          boost::bind(&LocalizationManager::CommandCallBack, this, _1, _2));

  ros::NodeHandle default_nh("/");
  loc_diagnostic_pub_ = default_nh.advertise<common_msg::Diagnostic>(
      "/system/diagnostics", 10, true);

  ros::Timer timer = default_nh.createTimer(
      ros::Duration(1),
      std::bind(&LocalizationManager::PublishDiagnosticInfo, this), false,
      true);

  UpdateLocMode(common_msg::LocMode::LOC_SLEEPING);

  auto log_path = node_.param<std::string>("log_path", "/mnt/logs/") + "/log/";

#ifdef USE_STACK_TRACE_LOGGER
  google::SetLogDestination(google::INFO, (log_path + "loc_").c_str());
  if (access(log_path.c_str(), 0) != 0) {
    if (mkdir(log_path.c_str(),
              S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO) == 0) {
      LOG(INFO) << "Create log_path:" << log_path;
    } else {
      LOG(ERROR) << "Cannot Create log_path:" << log_path;
    }
  } else {
    LOG(INFO) << "LOG_PATH:" << log_path.c_str();
  }
#endif

  LOG(INFO) << "LocManager:-----------------------start!!";
  is_simulation_ = node_.param("simulation", false);
  if (is_simulation_) {
    LOG(INFO) << "in sim mode";
    pf_loc_ptr_ = std::make_shared<PFLocalizationNode>();
  }
  LOG(INFO) << "LocManager Initialized, current mode:"
            << common_msg::GetLocModeCStr(GetLocMode());

  while (ros::ok()) {
    // 避免控制消息接收不到
    if (!pf_loc_ptr_) {
      ros::spinOnce();
    }
    ros::Duration(0.05).sleep();
  }
}

LocalizationManager::~LocalizationManager() {
  if (end_loc_thread_.joinable()) {
    end_loc_thread_.join();
  }

  if (pf_loc_ptr_) {
    pf_loc_ptr_->Termination();
  }
  LOG(INFO) << "LocManager:-----------------------end!!";
}

void LocalizationManager::PublishDiagnosticInfo() {
  common_msg::Diagnostic temp_info;
  temp_info.node_id = NODE_ID_LOC;
  if (pf_loc_ptr_ != nullptr) {
    UpdateLocMode(pf_loc_ptr_->GetLocMode());
  }
  temp_info.diagnostic = GetLocMode();
  loc_diagnostic_pub_.publish(temp_info);
}

void LocalizationManager::EndLocFunc() {
  pf_loc_ptr_->Termination();
  LOG(INFO) << "localization node going to end !";
  while (ros::ok()) {
    if (pf_loc_ptr_->GetStatusEnd()) {
      break;
    }
    ros::Duration(0.05).sleep();
  }
  pf_loc_ptr_ = nullptr;
  LOG(INFO) << "localization node ended !";
  boost::mutex::scoped_lock lock(status_mutex_);
  UpdateLocMode(common_msg::LocMode::LOC_SLEEPING);
}

bool LocalizationManager::CommandCallBack(common_msg::LocControlRequest& req,
                                          common_msg::LocControlResponse& res) {
  LOG(INFO) << "Ask command: " << GetLocRequestCStr(req);
  if (is_simulation_) {
    LOG(INFO) << "In simualtion mode, do not response!!";
    res.result_mode = GetLocMode();
    return true;
  }
  boost::mutex::scoped_lock lock(status_mutex_);
  switch (req.request_type) {
    case common_msg::LOC_R_GET:
      res.result_mode = GetLocMode();
      break;
    case common_msg::LOC_R_START:
      if (!pf_loc_ptr_) {
        pf_loc_ptr_ = std::make_shared<PFLocalizationNode>();
        LOG(INFO) << "localization node start!";
        UpdateLocMode(common_msg::LOC_RUNNING);
      }
      res.result_mode = GetLocMode();
      break;
    case common_msg::LOC_R_STOP:
      if (pf_loc_ptr_) {
        UpdateLocMode(common_msg::LOC_UPDATING);
        end_loc_thread_ = boost::thread([this] { EndLocFunc(); });
      }
      res.result_mode = GetLocMode();
      break;
    case common_msg::LOC_MODE_GLOBAL:
    default:
      res.result_mode = GetLocMode();
      break;
  }
  LOG(INFO) << "current mode:" << common_msg::GetLocModeCStr(res.result_mode);
  return true;
}

void CrashDump(const char* data, int size) { LOG(ERROR) << data; }

int main(int argc, char** argv) {
#ifdef USE_STACK_TRACE_LOGGER
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::InstallFailureWriter(&CrashDump);
  FLAGS_logbufsecs = 10;                    //日志实时输出会10s刷新到硬盘
  FLAGS_max_log_size = 10;                 //最大日志文件大小 10M
  FLAGS_logtostderr = false;               // log only to stdout
  FLAGS_stop_logging_if_full_disk = true;  // check full disk
  FLAGS_colorlogtostderr = true;           // color enable
  FLAGS_alsologtostderr = true;            // need stdout while log file
#endif

  ros::init(argc, argv, "localization");
  ros::NodeHandle nh;

  LocalizationManager loc_manager;
  return 0;
}
