//*****************************************************************************
// Copyright (c) 2022 MOI SmartRobotTechnology Co.,Ltd.
//*****************************************************************************
#include "laser_mapping/slam_manager.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>

#include "laser_mapping/laser_mapping.h"

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#include <fstream>
#include <unistd.h>
#include <sys/stat.h>
// #include <sys/types.h>
#endif

SlamManager::SlamManager() : node_("~") {
  UpdateSlamMode(common_msg::SLAM_SLEEPING);
  mapping_service_ = node_.advertiseService<common_msg::SlamControlRequest, common_msg::SlamControlResponse>(
      "mode_cmd", boost::bind(&SlamManager::CommandCallBack, this, _1, _2));
  ros::NodeHandle default_nh("/");
  diagnostic_pub_ = default_nh.advertise<common_msg::Diagnostic>("/system/diagnostics", 10, true);

  ros::Timer timer =
      default_nh.createTimer(ros::Duration(1), std::bind(&SlamManager::PublishDiagnosticInfo, this), false, true);

  auto log_path = node_.param<std::string>("log_path", "/mnt/logs/") + "/log/";

#ifdef USE_STACK_TRACE_LOGGER
  google::SetLogDestination(google::INFO, (log_path + "slam_").c_str());
  if (access(log_path.c_str(), 0) != 0) {
    if (mkdir(log_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO) == 0) {
      LOG(INFO) << "Create log_path:" << log_path;
    } else {
      LOG(ERROR) << "Cannot Create log_path:" << log_path;
    }
  } else {
    LOG(INFO) << "LOG_PATH:" << log_path.c_str();
  }
#endif

  LOG(INFO) << "SlamManager:-----------------------start!!";

  is_simulation_ = node_.param("simulation", false);
  std::string rosbag_path;
  if (is_simulation_) {
    LOG(INFO) << "in sim mode!!!";
    laser_mapping_ptr_ = std::make_shared<LaserMapping>(node_, is_simulation_);
    // 判断是否指定了rosbag
    rosbag_path = node_.param<std::string>("rosbag_path", "");
    if (!rosbag_path.empty()) {
      LOG(INFO) << "Using rosbag to create map!! rosbag path: " << rosbag_path;
    }
    key_input_thread_ = boost::thread(&SlamManager::KeyInputFunc, this);
  }
  LOG(INFO) << "SlamManager Initialized, current slam mode:" << common_msg::GetSlamModeCStr(GetSlamMode());

  if (is_simulation_ && !rosbag_path.empty()) {
    // 加载rosbag
    rosbag::Bag bag;
    bool use_bag{false};
    try {
      bag.open(rosbag_path, rosbag::bagmode::Read);
      use_bag = true;
    } catch (std::exception &ex) {
      ROS_ERROR("Unable to open rosbag [%s]", rosbag_path.c_str());
    }
    if (use_bag) {
      rosbag::View view(bag);

      auto start_real_time = boost::chrono::steady_clock::now();
      auto start_sim_time = view.getBeginTime();

      auto prev_real_time = start_real_time;
      auto prev_sim_time = start_sim_time;

      auto clock_publisher = node_.advertise<rosgraph_msgs::Clock>("/clock", 1);
      auto laser_publisher = node_.advertise<sensor_msgs::LaserScan>("scan", 1);
      auto odom_publisher = node_.advertise<nav_msgs::Odometry>("/odom", 1);
      rosgraph_msgs::Clock clock_msg;
      for (const rosbag::MessageInstance &m : view) {
        const sensor_msgs::LaserScanConstPtr &laser_ptr = m.instantiate<sensor_msgs::LaserScan>();
        if (laser_ptr != nullptr) {
          // laser_mapping_ptr_->LaserCallback(laser_ptr);
          laser_publisher.publish(laser_ptr);
        }

        const nav_msgs::OdometryConstPtr odom_ptr = m.instantiate<nav_msgs::Odometry>();
        if (odom_ptr != nullptr) {
          // laser_mapping_ptr_->OdomCallBack(odom_ptr);
          odom_publisher.publish(odom_ptr);
        }

        clock_msg.clock = m.getTime();
        clock_publisher.publish(clock_msg);

        auto real_time = boost::chrono::steady_clock::now();
        if (real_time - prev_real_time > boost::chrono::seconds(5)) {
          auto sim_time = m.getTime();
          double delta_real =
              boost::chrono::duration_cast<boost::chrono::milliseconds>(real_time - prev_real_time).count() * 0.001;
          auto delta_sim = (sim_time - prev_sim_time).toSec();
          LOG(WARNING) << "Processing the rosbag at " << delta_sim / delta_real << "X speed.";
          prev_sim_time = sim_time;
          prev_real_time = real_time;
        }
        ros::spinOnce();

        while (mapping_stopped_) {
          boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        }

        // 避免数据积压
        while (laser_mapping_ptr_->GetLaserBufferSize() > 40) {
          // 设置两级，避免卡顿
          while (laser_mapping_ptr_->GetLaserBufferSize() > 10) {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
          }
        }

        if (!ros::ok()) {
          break;
        }
      }

      bag.close();
      LOG(INFO) << "process bag finished! waiting slam end...";
      while (laser_mapping_ptr_->GetLaserBufferSize() != 0) {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
      }
      LOG(INFO) << "all lasers have been processed, slam end!!";
    }
  }

  ros::spin();
}

void SlamManager::KeyInputFunc() {
  std::string line;
  while (ros::ok()) {
    getline(std::cin, line);
    mapping_stopped_ = !mapping_stopped_;
    if(mapping_stopped_) {
    LOG(WARNING) << "Mapping stopped by key Interrupt, to start click Enter again";
    }
  }
}

void SlamManager::PublishDiagnosticInfo() {
  common_msg::Diagnostic temp_info;
  temp_info.node_id = NODE_ID_SLAM;
  if (laser_mapping_ptr_ != nullptr) {
    UpdateSlamMode(laser_mapping_ptr_->GetSlamMode());
  }
  temp_info.diagnostic = GetSlamMode();
  diagnostic_pub_.publish(temp_info);
}

void SlamManager::EndSlamFunc() {
  laser_mapping_ptr_->EndSLAM();
  LOG(INFO) << "mapping going to end";
  while (ros::ok()) {
    if (laser_mapping_ptr_->GetStatusEnd()) {
      break;
    }
    ros::Duration(0.05).sleep();
  }
  laser_mapping_ptr_ = nullptr;
  LOG(INFO) << "mapping ended";
  boost::mutex::scoped_lock lock(status_mutex_);
  UpdateSlamMode(common_msg::SLAM_SLEEPING);
}

bool SlamManager::CommandCallBack(common_msg::SlamControlRequest &req, common_msg::SlamControlResponse &res) {
  LOG(INFO) << "Ask command: " << GetSlamRequestCStr(req);
  if (is_simulation_) {
    LOG(INFO) << "In simualtion mode, do not response!!";
    res.result_mode = GetSlamMode();
    return true;
  }
  boost::mutex::scoped_lock lock(status_mutex_);
  switch (req.request_type) {
    case common_msg::SLAM_R_GET:
      res.result_mode = GetSlamMode();
      break;
    case common_msg::SLAM_R_START:
      if (!laser_mapping_ptr_) {
        laser_mapping_ptr_ = std::make_shared<LaserMapping>(node_);
        LOG(INFO) << "mapping start";
        UpdateSlamMode(common_msg::SLAM_RUNNING);
      }
      res.result_mode = GetSlamMode();
      break;
    case common_msg::SLAM_R_STOP:
      if (laser_mapping_ptr_ && GetSlamMode() != common_msg::SLAM_UPDATING) {
        UpdateSlamMode(common_msg::SLAM_UPDATING);
        end_slam_thread_ = boost::thread(boost::bind(&SlamManager::EndSlamFunc, this));
      }
      res.result_mode = GetSlamMode();
      break;
    case common_msg::SLAM_R_CLOSE_LOOP_IN_PLACE:
    case common_msg::SLAM_R_CLOSE_LOOP_AT_SPEC:
    case common_msg::SLAM_R_X_AXIS_SET:
    default:
      res.result_mode = GetSlamMode();
      break;
  }
  LOG(INFO) << "curretn slam mode:" << common_msg::GetSlamModeCStr(res.result_mode);
  return true;
}

SlamManager::~SlamManager() {
  if (end_slam_thread_.joinable()) {
    end_slam_thread_.join();
  }

  if (laser_mapping_ptr_) {
    laser_mapping_ptr_->Termination();
  }

  if (key_input_thread_.joinable()) {
    end_slam_thread_.join();
  }

  LOG(INFO) << "SlamManager:-----------------------end!!";
}

void CrashDump(const char *data, int size) { LOG(ERROR) << data; }

int main(int argc, char **argv) {
#ifdef USE_STACK_TRACE_LOGGER
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::InstallFailureWriter(&CrashDump);
  FLAGS_logbufsecs = 10;                   // 日志实时输出会10s刷新到硬盘
  FLAGS_max_log_size = 10;                 // 最大日志文件大小 10M
  FLAGS_logtostderr = false;               // log only to stdout
  FLAGS_stop_logging_if_full_disk = true;  // check full disk
  FLAGS_colorlogtostderr = true;           // color enable
  FLAGS_alsologtostderr = true;            // need stdout while log file
#endif
  ros::init(argc, argv, "laser_mapping");
  SlamManager slam_manager;

  return 0;
}
