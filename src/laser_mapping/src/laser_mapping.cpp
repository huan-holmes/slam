//*****************************************************************************
// Copyright (c) 2022 MOI SmartRobotTechnology Co.,Ltd.
//*****************************************************************************

#include "laser_mapping/laser_mapping.h"

// ros
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>
// json
#include "nlohmann/json.hpp"

#include "spa_solver.h"
#include "auto_rotation.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

LaserMapping::LaserMapping(ros::NodeHandle &nh, bool is_simulation)
    : node_handle_(nh),
      first_map_updated_flag_(false),
      is_simulation_(is_simulation),
      laser_freq_count_(0),
      marker_count_(0),
      b_exit_pub_tf_thread_(false) {
  tf_odom2map_.setIdentity();
  last_map_update_ = ros::WallTime::now();
  // Retrieve parameters
  ros::NodeHandle private_nh("~");
  if (!private_nh.getParam("odom_frame", odom_frame_)) odom_frame_ = "odom";
  if (!private_nh.getParam("map_frame", map_frame_)) map_frame_ = "map";
  if (!private_nh.getParam("base_frame", base_frame_)) base_frame_ = "base_link";

  // 初始化诊断诊断信息
  slam_diagnostic_mode_ = common_msg::SlamMode::SLAM_RUNNING;

  // 手动回环控制
  control_pose_sub_ = node_handle_.subscribe("control_pose", 1, &LaserMapping::ControlPoseCallback, this);

  // 订阅odom
  odom_sub_ = node_handle_.subscribe("/odom", 100, &LaserMapping::OdomCallBack, this);

  // 订阅激光
  laser_sub_ = node_handle_.subscribe("scan", 20, &LaserMapping::LaserCallback, this);

  // 可视化
  markers_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("visualization_array", 1);
  // 建图busy状态
  working_state_pub_ = node_handle_.advertise<std_msgs::UInt8>("mapping_status", 5);

  // 发布地图
  map_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  map_meta_pub_ = node_handle_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  map_srv_ = node_handle_.advertiseService("dynamic_map", &LaserMapping::MapCallback, this);

  // 手动回环控制
  control_pose_sub_ = node_handle_.subscribe("control_pose", 1, &LaserMapping::ControlPoseCallback, this);
  // 自动旋转地图
  rotate_map_sub_ = node_handle_.subscribe("rotate_map", 1, &LaserMapping::RotateMapCallback, this);

  // 发布TF
  pub_tf_thread_ = boost::thread(boost::bind(&LaserMapping::PublishTFLoop, this));

  // Initialize Karto structures
  mapper_ = new karto::Mapper();
  dataset_ = new karto::Dataset();

  // Setting General Parameters from the Parameter Server

  private_nh.param("throttle_scans", throttle_scans_, 1);
  private_nh.param("resolution", resolution_, 0.05);
  double map_update_interval;
  private_nh.param("map_update_interval", map_update_interval, 5.0);
  map_update_interval_ = map_update_interval;
  private_nh.param("publish_graph_marker", b_publish_graph_marker_, false);
  // private_nh_.param("loop_check_peirod", loop_check_peirod_, 1.0);
  private_nh.param("mapping_max_scanRange", scan_range_max_, 8.0);

  // 获取激光外参数
  private_nh.param("laser_x_offset", laser_x_offset_, 0.0);
  private_nh.param("laser_y_offset", laser_y_offset_, 0.0);
  private_nh.param("laser_yaw_offset", laser_yaw_offset_, 0.0);
  LOG(INFO) << "laser param laser_x_offset: " << laser_x_offset_ << " laser_y_offset: " << laser_y_offset_
            << " laser_yaw_offset: " << laser_yaw_offset_;

  LOG(INFO) << "scan_range max" << scan_range_max_;
  bool use_scan_matching;
  if (private_nh.getParam("use_scan_matching", use_scan_matching)) {
    mapper_->setParamUseScanMatching(use_scan_matching);
  }

  bool use_scan_barycenter;
  if (node_handle_.getParam("use_scan_barycenter", use_scan_barycenter))
    mapper_->setParamUseScanBarycenter(use_scan_barycenter);

  minimum_travel_distance_ = 0.3;
  if (node_handle_.getParam("minimum_travel_distance", minimum_travel_distance_))
    mapper_->setParamMinimumTravelDistance(minimum_travel_distance_);

  minimum_travel_heading_ = 0.628;
  if (node_handle_.getParam("minimum_travel_heading", minimum_travel_heading_))
    mapper_->setParamMinimumTravelHeading(minimum_travel_heading_);
  LOG(INFO) << "minimum_travel_distance: " << minimum_travel_distance_
            << " minimum_travel_heading: " << minimum_travel_heading_;

  int scan_buffer_size;
  if (node_handle_.getParam("scan_buffer_size", scan_buffer_size)) mapper_->setParamScanBufferSize(scan_buffer_size);

  double scan_buffer_maximum_scan_distance;
  if (node_handle_.getParam("scan_buffer_maximum_scan_distance", scan_buffer_maximum_scan_distance))
    mapper_->setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);

  double link_match_minimum_response_fine;
  if (node_handle_.getParam("link_match_minimum_response_fine", link_match_minimum_response_fine))
    mapper_->setParamLinkMatchMinimumResponseFine(link_match_minimum_response_fine);

  double link_scan_maximum_distance;
  if (node_handle_.getParam("link_scan_maximum_distance", link_scan_maximum_distance))
    mapper_->setParamLinkScanMaximumDistance(link_scan_maximum_distance);

  double loop_search_maximum_distance;
  if (node_handle_.getParam("loop_search_maximum_distance", loop_search_maximum_distance))
    mapper_->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);

  bool do_loop_closing;
  if (node_handle_.getParam("do_loop_closing", do_loop_closing)) mapper_->setParamDoLoopClosing(do_loop_closing);

  int loop_match_minimum_chain_size;
  if (node_handle_.getParam("loop_match_minimum_chain_size", loop_match_minimum_chain_size))
    mapper_->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);

  double loop_match_maximum_variance_coarse;
  if (node_handle_.getParam("loop_match_maximum_variance_coarse", loop_match_maximum_variance_coarse))
    mapper_->setParamLoopMatchMaximumVarianceCoarse(loop_match_maximum_variance_coarse);

  double loop_match_minimum_response_coarse;
  if (node_handle_.getParam("loop_match_minimum_response_coarse", loop_match_minimum_response_coarse))
    mapper_->setParamLoopMatchMinimumResponseCoarse(loop_match_minimum_response_coarse);

  double loop_match_minimum_response_fine;
  if (node_handle_.getParam("loop_match_minimum_response_fine", loop_match_minimum_response_fine))
    mapper_->setParamLoopMatchMinimumResponseFine(loop_match_minimum_response_fine);

  // Setting Correlation Parameters from the Parameter Server

  double correlation_search_space_dimension;
  if (node_handle_.getParam("correlation_search_space_dimension", correlation_search_space_dimension))
    mapper_->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);

  double correlation_search_space_resolution;
  if (node_handle_.getParam("correlation_search_space_resolution", correlation_search_space_resolution))
    mapper_->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);

  double correlation_search_space_smear_deviation;
  if (node_handle_.getParam("correlation_search_space_smear_deviation", correlation_search_space_smear_deviation))
    mapper_->setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);

  // Setting Correlation Parameters, Loop Closure Parameters from the Parameter Server

  double loop_search_space_dimension;
  if (node_handle_.getParam("loop_search_space_dimension", loop_search_space_dimension))
    mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);

  double loop_search_space_resolution;
  if (node_handle_.getParam("loop_search_space_resolution", loop_search_space_resolution))
    mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);

  double loop_search_space_smear_deviation;
  if (node_handle_.getParam("loop_search_space_smear_deviation", loop_search_space_smear_deviation))
    mapper_->setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);

  // Setting Scan Matcher Parameters from the Parameter Server

  double distance_variance_penalty;
  if (node_handle_.getParam("distance_variance_penalty", distance_variance_penalty))
    mapper_->setParamDistanceVariancePenalty(distance_variance_penalty);

  double angle_variance_penalty;
  if (node_handle_.getParam("angle_variance_penalty", angle_variance_penalty))
    mapper_->setParamAngleVariancePenalty(angle_variance_penalty);

  double fine_search_angle_offset;
  if (node_handle_.getParam("fine_search_angle_offset", fine_search_angle_offset))
    mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);

  double coarse_search_angle_offset;
  if (node_handle_.getParam("coarse_search_angle_offset", coarse_search_angle_offset))
    mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);

  double coarse_angle_resolution;
  if (node_handle_.getParam("coarse_angle_resolution", coarse_angle_resolution))
    mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);

  double minimum_angle_penalty;
  if (node_handle_.getParam("minimum_angle_penalty", minimum_angle_penalty))
    mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);

  double minimum_distance_penalty;
  if (node_handle_.getParam("minimum_distance_penalty", minimum_distance_penalty))
    mapper_->setParamMinimumDistancePenalty(minimum_distance_penalty);

  bool use_response_expansion;
  if (node_handle_.getParam("use_response_expansion", use_response_expansion))
    mapper_->setParamUseResponseExpansion(use_response_expansion);

  double minimumMatchScore = 0.0;
  if (private_nh.getParam("minimumMatchScore", minimumMatchScore)) {
    mapper_->setminimumMatchScanScore(minimumMatchScore);
  }
  double validLaserPercentage = 0.0;
  if (node_handle_.getParam("validLaserPercentage", validLaserPercentage))
    mapper_->setValidLaserPercent(validLaserPercentage);

  use_imu_correct_ = node_handle_.param("use_imu_correct", false);
  mapper_->setParamUseYawLineComp(use_imu_correct_);
  if (use_imu_correct_) {
    LOG(WARNING) << "use yaw line comp!!";
  }

  // Set solver to be used in loop closure
  solver_ = new SpaSolver();
  mapper_->SetScanSolver(solver_);
  slam_stop_request_flag_ = false;
  slam_process_end_ = false;
  // laser_first_in_ = false;
  main_process_thread_ = boost::thread(boost::bind(&LaserMapping::ProcessMainLoop, this));
}

void LaserMapping::Termination() {
  slam_stop_request_flag_ = true;
  b_exit_pub_tf_thread_ = true;
  if (pub_tf_thread_.joinable()) {
    pub_tf_thread_.join();
  }

  if (main_process_thread_.joinable()) {
    main_process_thread_.join();
  }

  delete solver_;
  delete mapper_;
  delete dataset_;
}

LaserMapping::~LaserMapping() { Termination(); }

void LaserMapping::PublishTFLoop() {
  int publish_tf_freq = 20;
  int time_step = 0;
  LOG(INFO) << "PublishTFLoop start publish_tf_freq:" << publish_tf_freq;
  // ros::Rate r(publish_tf_freq);

  using clock = boost::chrono::steady_clock;
  using timestamp = boost::chrono::time_point<clock>;

  timestamp start{clock::now()};
  timestamp next{start + boost::chrono::milliseconds(50)};

  while (ros::ok() && !b_exit_pub_tf_thread_) {
    time_step++;
    if (time_step == publish_tf_freq) {
      time_step = 0;
      if (laser_freq_count_ < 5) {
        LOG(WARNING) << "scan low freq ,current is: " << laser_freq_count_;
        slam_diagnostic_mode_ = common_msg::SlamMode::SLAM_ERROR_SCAN_LOW_FREQ;
      } else {
        if (slam_stop_request_flag_) {
          slam_diagnostic_mode_ = common_msg::SlamMode::SLAM_UPDATING;
        } else {
          slam_diagnostic_mode_ = common_msg::SlamMode::SLAM_RUNNING;
        }
      }
      laser_freq_count_ = 0;
    }
    PublishTransform();
    // 更新时间
    if (clock::now() < next) {
      boost::this_thread::sleep_until(next);
    }
    start = clock::now();
    next = start + boost::chrono::milliseconds(50);
  }
  LOG(INFO) << "PublishTFLoop exit";
}

void LaserMapping::PublishTransform() {
  static uint32_t cnt = 0;

  auto stamp = ros::Time::now();
  if (stamp.nsec != cnt) {
    cnt = stamp.nsec;
    boost::mutex::scoped_lock lock(tf_odom2map_mutex_);
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_odom2map_, ros::Time::now(), map_frame_, odom_frame_));
  }
}

void LaserMapping::ControlPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
  LOG(INFO) << "ControlPose: " << msg.pose.pose.position.x << " " << msg.pose.pose.position.y;
  if (mapper_->GetGraph()) {
    mapper_->GetGraph()->manual_loop_closure_flag = true;
    mapper_->GetGraph()->loop_pos_x = msg.pose.pose.position.x;
    mapper_->GetGraph()->loop_pos_y = msg.pose.pose.position.y;
  } else {
    LOG(INFO) << "Graph is null!";
  }
}

karto::LaserRangeFinder *LaserMapping::GetLaserRangeFinder(const sensor_msgs::LaserScan::ConstPtr &scan) {
  // Check whether we know about this laser yet
  if (lasers_.find(scan->header.frame_id) == lasers_.end()) {
    // New laser; need to create a Karto device for it.

    // // Get the laser's pose, relative to base.
    // tf::Stamped<tf::Pose> ident;
    // tf::Stamped<tf::Transform> laser_pose;
    // ident.setIdentity();
    // ident.frame_id_ = scan->header.frame_id;
    // ident.stamp_ = scan->header.stamp;
    // try {
    //   tf_listener_.transformPose(base_frame_, ident, laser_pose);
    // } catch (tf::TransformException &e) {
    //   LOG(WARNING) << "Failed to compute laser pose, aborting initialization " << e.what();
    //   return nullptr;
    // }
    //
    // tf::StampedTransform laser_pose;
    // while (true) {
    //     try {
    //         tf_.waitForTransform(base_frame_, scan->header.frame_id, ros::Time(0), ros::Duration(1.0));
    //         tf_.lookupTransform(base_frame_, scan->header.frame_id, ros::Time(0), laser_pose);
    //         break;
    //     } catch (tf::TransformException &ex) {
    //         ROS_WARN_STREAM(ex.what());
    //         continue;
    //     }
    // }
    // laser_pose.setIdentity();

    // double yaw = tf::getYaw(laser_pose.getRotation());
    //
    // LOG(INFO) << "laser" << scan->header.frame_id.c_str() << " pose wrt base: " << laser_pose.getOrigin().x() << " "
    //           << laser_pose.getOrigin().y() << " " << yaw;
    // // To account for lasers that are mounted upside-down,
    // // we create a point 1m above the laser and transform it into the laser frame
    // // if the point's z-value is <=0, it is upside-down
    //
    // tf::Vector3 v;
    // v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
    // tf::Stamped<tf::Vector3> up(v, scan->header.stamp, base_frame_);
    //
    // try {
    //   tf_listener_.transformPoint(scan->header.frame_id, up, up);
    //   LOG(INFO) << "Z-Axis in sensor frame:" << up.z();
    // } catch (tf::TransformException &e) {
    //   LOG(WARNING) << "Unable to determine orientation of laser: " << e.what();
    //   return nullptr;
    // }
    //
    // bool inverse = lasers_inverted_[scan->header.frame_id] = up.z() <= 0;
    // if (inverse) LOG(INFO) << ("laser is mounted upside-down");

    lasers_inverted_[scan->header.frame_id] = false;

    // Create a laser range finder device and copy in data from the first
    // scan
    std::string name = scan->header.frame_id;
    karto::LaserRangeFinder *laser =
        karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, karto::Name(name));
    laser->SetOffsetPose(karto::Pose2(laser_x_offset_, laser_y_offset_, laser_yaw_offset_));
    laser->SetMinimumRange(scan->range_min);
    // range max
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);

    laser->SetRangeThreshold(scan_range_max_);
    // Store this laser device for later
    lasers_[scan->header.frame_id] = laser;

    // Add it to the dataset, which seems to be necessary
    dataset_->Add(laser);
  }

  return lasers_[scan->header.frame_id];
}

void LaserMapping::PublishGraphVisualization() {
  if (!b_publish_graph_marker_) return;
  std::vector<float> graph;
  solver_->getGraph(graph);

  visualization_msgs::MarkerArray marray;

  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.ns = "karto";
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.0;
  m.pose.orientation.x = 0.;
  m.pose.orientation.y = 0.;
  m.pose.orientation.z = 0.;
  m.pose.orientation.w = 1.;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.color.r = 1.0;
  m.color.g = 0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.lifetime = ros::Duration(0);

  visualization_msgs::Marker edge;
  edge.header.frame_id = "map";
  edge.header.stamp = ros::Time::now();
  edge.action = visualization_msgs::Marker::ADD;
  edge.ns = "karto";
  edge.id = 0;
  edge.type = visualization_msgs::Marker::LINE_STRIP;
  edge.pose.orientation.x = 0.;
  edge.pose.orientation.y = 0.;
  edge.pose.orientation.z = 0.;
  edge.pose.orientation.w = 1.;
  edge.scale.x = 0.1;
  edge.scale.y = 0.1;
  edge.scale.z = 0.1;
  edge.color.a = 1.0;
  edge.color.r = 0.0;
  edge.color.g = 0.0;
  edge.color.b = 1.0;

  m.action = visualization_msgs::Marker::ADD;
  uint id = 0;
  for (uint i = 0; i < graph.size() / 2; i++) {
    m.id = id;
    m.pose.position.x = graph[2 * i];
    m.pose.position.y = graph[2 * i + 1];
    marray.markers.push_back(visualization_msgs::Marker(m));
    id++;

    if (i > 0) {
      edge.points.clear();

      geometry_msgs::Point p;
      p.x = graph[2 * (i - 1)];
      p.y = graph[2 * (i - 1) + 1];
      edge.points.push_back(p);
      p.x = graph[2 * i];
      p.y = graph[2 * i + 1];
      edge.points.push_back(p);
      edge.id = id;

      marray.markers.push_back(visualization_msgs::Marker(edge));
      id++;
    }
  }

  m.action = visualization_msgs::Marker::DELETE;
  for (; id < marker_count_; id++) {
    m.id = id;
    marray.markers.push_back(visualization_msgs::Marker(m));
  }

  marker_count_ = marray.markers.size();

  // // 直线可视化
  // visualization_msgs::Marker lines_marker;
  // lines_marker.ns = "line_extraction";
  // lines_marker.id = 0;
  // lines_marker.type = visualization_msgs::Marker::LINE_LIST;
  // lines_marker.scale.x = 0.1;
  // lines_marker.color.r = 1.0;
  // lines_marker.color.g = 0.0;
  // lines_marker.color.b = 0.0;
  // lines_marker.color.a = 1.0;
  // for (auto &line : lines_vec_) {
  //   geometry_msgs::Point p_start;
  //   p_start.x = line[0];
  //   p_start.y = line[1];
  //   p_start.z = 0;
  //   lines_marker.points.push_back(p_start);
  //   geometry_msgs::Point p_end;
  //   p_end.x = line[2];
  //   p_end.y = line[3];
  //   p_end.z = 0;
  //   lines_marker.points.push_back(p_end);
  // }
  // lines_marker.header.frame_id = "base_laser_link";
  // lines_marker.header.stamp = ros::Time::now();
  // marray.markers.push_back(visualization_msgs::Marker(lines_marker));

  markers_pub_.publish(marray);
}

void LaserMapping::OdomCallBack(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  static tf::StampedTransform b2o;
  // printf("odom time: %.3f\n", odom_msg->header.stamp.toSec());

  {
    boost::mutex::scoped_lock lock(odom_mtx_);
    odom_pose_buffer_.push(odom_msg);
  }

  b2o.stamp_ = odom_msg->header.stamp;
  b2o.frame_id_ = odom_frame_;
  b2o.child_frame_id_ = base_frame_;

  b2o.setRotation(tf::Quaternion(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
                                 odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w));
  b2o.setOrigin(
      tf::Vector3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z));

  boost::mutex::scoped_lock lock(tf_odom2map_mutex_);
  tf_broadcaster_.sendTransform(b2o);
}

void LaserMapping::LaserCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
  laser_freq_count_++;
  if ((laser_freq_count_ % throttle_scans_) != 0 && !slam_stop_request_flag_) return;
  {
    boost::mutex::scoped_lock lock(laser_mtx_);
    laser_scans_buffer_.push(scan);
  }
}

void LaserMapping::RotateProcessedScans(double rad) {
  auto scan_vector = mapper_->GetAllProcessedScans();
  for (auto &scan : scan_vector) {
    auto pose2 = scan->GetSensorPose();
    karto::Pose2 new_pose;
    new_pose.SetX(cos(rad) * pose2.GetX() - sin(rad) * pose2.GetY());
    new_pose.SetY(sin(rad) * pose2.GetX() + cos(rad) * pose2.GetY());
    new_pose.SetHeading(karto::math::NormalizeAngle(pose2.GetHeading() + rad));
    scan->SetSensorPose(new_pose);
  }
}

void LaserMapping::RotateMapCallback(const common_msg::RotateMap &msg) {
  LOG(INFO) << "[RotateMapCallback]  " << (int)msg.type << " - " << msg.rotate_angle_deg;
  if (!mapper_) return;

  if (msg.rotate_angle_deg < -180. || msg.rotate_angle_deg > 180.) {
    LOG(ERROR) << "rotate angle should in range [-180, 180]";
    return;
  }

  boost::mutex::scoped_lock lock(map_mutex_);

  karto::OccupancyGrid *pOccupancyGrid =
      karto::OccupancyGrid::CreateFromScans(mapper_->GetAllProcessedScans(), resolution_);

  if (!pOccupancyGrid) {
    LOG(WARNING) << "OccupancyGrid is null";
    return;
  }

  auto bounding_box = pOccupancyGrid->GetBoundingBox();
  kt_int32s width = pOccupancyGrid->GetWidth();
  kt_int32s height = pOccupancyGrid->GetHeight();
  karto::Vector2<kt_double> offset = pOccupancyGrid->GetCoordinateConverter()->GetOffset();

  if (width <= 0 || height <= 0) {
    delete pOccupancyGrid;
    LOG(WARNING) << "cannot rotate! width: " << width << ", height: " << height;
    return;
  }

  LOG(INFO) << "OccupancyGrid - width: " << width << ", height: " << height << ", resolution: " << resolution_;
  LOG(INFO) << "origin: [" << offset.GetX() << ", " << offset.GetY() << "]";
  LOG(INFO) << "BoundingBox - min: [" << bounding_box.GetMinimum().GetX() << ", " << bounding_box.GetMinimum().GetY()
            << "], max: [" << bounding_box.GetMaximum().GetX() << ", " << bounding_box.GetMaximum().GetY() << "]";

  // 点集合
  std::vector<tools::Point> occ_points;
  std::vector<tools::Point> free_points;
  for (kt_int32s y = 0; y < height; y++) {
    for (kt_int32s x = 0; x < width; x++) {
      // Getting the value at position x, y
      kt_int8u value = pOccupancyGrid->GetValue(karto::Vector2<kt_int32s>(x, y));

      if (value == karto::GridStates_Free) {
        free_points.emplace_back(x, y);
      } else if (value == karto::GridStates_Occupied) {
        occ_points.emplace_back(x, y);
      }
    }
  }

  // LOG(WARNING) << "occ points: " << occ_points.size() << ", free points: " << free_points.size();
  LOG(WARNING) << "occ points: " << occ_points.size();

  // if (occ_points.size() < 100 || free_points.size() < 100) {
  if (occ_points.size() < 100) {
    delete pOccupancyGrid;
    LOG(WARNING) << "too small map!";
    return;
  }

  double rotate_angle_rad = msg.rotate_angle_deg / 180. * M_PI;

  LOG(INFO) << "map points: " << occ_points.size();
  if (msg.type == 1) {
    auto rotate_angle_deg_in_image = tools::AutoRotation::Rotation(width, height, resolution_, occ_points);
    // 图像坐标系到地图坐标系转换
    rotate_angle_rad = karto::math::NormalizeAngle((rotate_angle_deg_in_image - 270.) / 180. * M_PI);
    printf("final rotate angle: %.2f\n", rotate_angle_rad * 180. / M_PI);
  }

  auto cos_r = cos(rotate_angle_rad);
  auto sin_r = sin(rotate_angle_rad);

  // 计算旋转后的bounding_box
  double min_x_rot = std::numeric_limits<double>::max();
  double max_x_rot = std::numeric_limits<double>::lowest();
  double min_y_rot = std::numeric_limits<double>::max();
  double max_y_rot = std::numeric_limits<double>::lowest();

  double left_top_x = bounding_box.GetMinimum().GetX();
  double left_top_y = bounding_box.GetMaximum().GetY();
  double x_rot = left_top_x * cos_r - left_top_y * sin_r;
  double y_rot = left_top_x * sin_r + left_top_y * cos_r;
  if (x_rot < min_x_rot) {
    min_x_rot = x_rot;
  }

  if (x_rot > max_x_rot) {
    max_x_rot = x_rot;
  }

  if (y_rot < min_y_rot) {
    min_y_rot = y_rot;
  }

  if (y_rot > max_y_rot) {
    max_y_rot = y_rot;
  }

  double left_bot_x = bounding_box.GetMinimum().GetX();
  double left_bot_y = bounding_box.GetMinimum().GetY();
  x_rot = left_bot_x * cos_r - left_bot_y * sin_r;
  y_rot = left_bot_x * sin_r + left_bot_y * cos_r;
  if (x_rot < min_x_rot) {
    min_x_rot = x_rot;
  }

  if (x_rot > max_x_rot) {
    max_x_rot = x_rot;
  }

  if (y_rot < min_y_rot) {
    min_y_rot = y_rot;
  }

  if (y_rot > max_y_rot) {
    max_y_rot = y_rot;
  }

  double right_top_x = bounding_box.GetMaximum().GetX();
  double right_top_y = bounding_box.GetMaximum().GetY();
  x_rot = right_top_x * cos_r - right_top_y * sin_r;
  y_rot = right_top_x * sin_r + right_top_y * cos_r;
  if (x_rot < min_x_rot) {
    min_x_rot = x_rot;
  }

  if (x_rot > max_x_rot) {
    max_x_rot = x_rot;
  }

  if (y_rot < min_y_rot) {
    min_y_rot = y_rot;
  }

  if (y_rot > max_y_rot) {
    max_y_rot = y_rot;
  }

  double right_bot_x = bounding_box.GetMaximum().GetX();
  double right_bot_y = bounding_box.GetMinimum().GetY();
  x_rot = right_bot_x * cos_r - right_bot_y * sin_r;
  y_rot = right_bot_x * sin_r + right_bot_y * cos_r;
  if (x_rot < min_x_rot) {
    min_x_rot = x_rot;
  }

  if (x_rot > max_x_rot) {
    max_x_rot = x_rot;
  }

  if (y_rot < min_y_rot) {
    min_y_rot = y_rot;
  }

  if (y_rot > max_y_rot) {
    max_y_rot = y_rot;
  }

  // 旋转后的原点应该为左下角 最小点
  // double origin_x_rot = offset.GetX() * cos_r - offset.GetY() * sin_r;
  // double origin_y_rot = offset.GetX() * sin_r + offset.GetY() * cos_r;
  double origin_x_rot = min_x_rot;
  double origin_y_rot = min_y_rot;

  int width_rot = (int)((max_x_rot - min_x_rot) / resolution_) + 1;
  int height_rot = (int)((max_y_rot - min_y_rot) / resolution_) + 1;

  LOG(INFO) << "Rotate width: " << width_rot << ", height: " << height_rot;
  LOG(INFO) << "Rotate origin: [" << origin_x_rot << ", " << origin_y_rot << "]";
  LOG(INFO) << "Rotate BoundingBox - min: [" << min_x_rot << ", " << min_y_rot << "], max: [" << max_x_rot << ", "
            << max_y_rot << "]";

  // 创建旋转后的地图
  map_.map.info.origin.position.x = origin_x_rot;
  map_.map.info.origin.position.y = origin_y_rot;
  map_.map.info.width = width_rot;
  map_.map.info.height = height_rot;
  map_.map.data.resize(map_.map.info.width * map_.map.info.height);
  // 填充0
  map_.map.data.assign(map_.map.data.size(), -1);

  // 变换所有的点
  int x, y;
  double x_in_orig, y_in_orig;
  double x_in_rot, y_in_rot;

  // 填充free
  for (const auto &p : free_points) {
    // 计算在原地图中的实际位置
    x_in_orig = p.x * resolution_ + offset.GetX();
    y_in_orig = p.y * resolution_ + offset.GetY();
    // 计算在旋转后地图中的位置
    x_in_rot = x_in_orig * cos_r - y_in_orig * sin_r;
    y_in_rot = x_in_orig * sin_r + y_in_orig * cos_r;
    // 计算数组位置
    x = (int)((x_in_rot - origin_x_rot) / resolution_ + 0.5);
    y = (int)((y_in_rot - origin_y_rot) / resolution_ + 0.5);
    if (x < 0 || y < 0) {
      printf("free [x, y] = [%d, %d]\n", x, y);
    } else {
      map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }

    if (y - 1 > 0) {
      map_.map.data[MAP_IDX(map_.map.info.width, x, y - 1)] = 0;
    }

    if (y + 1 < height_rot) {
      map_.map.data[MAP_IDX(map_.map.info.width, x, y + 1)] = 0;
    }
  }

  // 填充occ
  for (const auto &p : occ_points) {
    // 计算在原地图中的实际位置
    x_in_orig = p.x * resolution_ + offset.GetX();
    y_in_orig = p.y * resolution_ + offset.GetY();
    // 计算在旋转后地图中的位置
    x_in_rot = x_in_orig * cos_r - y_in_orig * sin_r;
    y_in_rot = x_in_orig * sin_r + y_in_orig * cos_r;
    // 计算数组位置
    x = (int)((x_in_rot - origin_x_rot) / resolution_ + 0.5);
    y = (int)((y_in_rot - origin_y_rot) / resolution_ + 0.5);
    if (x < 0 || y < 0) {
      // printf("p.x: %d, p.y: %d -- ", p.x, p.y);
      // printf("x_in_orig: %f, y_in_orig: %f \n", x_in_orig, y_in_orig);
      // printf("x_in_rot: %f, y_in_rot: %f -- ", x_in_rot, y_in_rot);
      printf("occ [x, y] = [%d, %d]\n", x, y);
    } else {
      map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
    }
  }

  // Set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = map_frame_;

  map_pub_.publish(map_.map);
  map_meta_pub_.publish(map_.map.info);

  delete pOccupancyGrid;
  LOG(INFO) << "Rotate Map Done!";
}

void LaserMapping::PublishMapOnce() {
  if (is_map_publishing_) return;
  if (!first_map_updated_flag_ || (ros::WallTime::now() - last_map_update_).toSec() > map_update_interval_) {
    // 创建后台线程去发布地图
    std::thread publish_thread(std::bind(&LaserMapping::PublishRosMap, this));
    publish_thread.detach();
  }
}

bool LaserMapping::PublishRosMap() {
  if (!mapper_) return false;
  is_map_publishing_ = true;
  boost::mutex::scoped_lock lock(map_mutex_);

  auto start_time = ros::WallTime::now();

  karto::OccupancyGrid *pOccupancyGrid =
      karto::OccupancyGrid::CreateFromScans(mapper_->GetAllProcessedScans(), resolution_);

  if (!pOccupancyGrid) {
    LOG(WARNING) << "OccupancyGrid is null";
    return false;
  }

  if (!first_map_updated_flag_) {
    map_.map.info.resolution = resolution_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  }

  // Translate to ROS format
  kt_int32s width = pOccupancyGrid->GetWidth();
  kt_int32s height = pOccupancyGrid->GetHeight();
  karto::Vector2<kt_double> offset = pOccupancyGrid->GetCoordinateConverter()->GetOffset();

  if (map_.map.info.width != (unsigned int)width || map_.map.info.height != (unsigned int)height ||
      map_.map.info.origin.position.x != offset.GetX() || map_.map.info.origin.position.y != offset.GetY()) {
    map_.map.info.origin.position.x = offset.GetX();
    map_.map.info.origin.position.y = offset.GetY();
    map_.map.info.width = width;
    map_.map.info.height = height;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
  }

  for (kt_int32s y = 0; y < height; y++) {
    for (kt_int32s x = 0; x < width; x++) {
      // Getting the value at position x,y
      kt_int8u value = pOccupancyGrid->GetValue(karto::Vector2<kt_int32s>(x, y));

      switch (value) {
        case karto::GridStates_Unknown:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
          break;
        case karto::GridStates_Occupied:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
          break;
        case karto::GridStates_Free:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
          break;
        default:
          LOG(WARNING) << "Encountered unknown cell value at " << x << " " << y;
          break;
      }
    }
  }

  // Set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = map_frame_;

  map_pub_.publish(map_.map);
  map_meta_pub_.publish(map_.map.info);

  delete pOccupancyGrid;
  auto end_time = ros::WallTime::now();
  double delta_time = (end_time - start_time).toSec() * 1e3;

  LOG(INFO) << "Update ros map process time : " << delta_time << " ms";
  is_map_publishing_ = false;
  first_map_updated_flag_ = true;
  last_map_update_ = ros::WallTime::now();
  return true;
}

void LaserMapping::ProcessMainLoop() {
  // 如果添加新的scan，设置为true
  bool new_scan_added = false;
  // 帧缓存大于2认为忙
  bool last_mapping_busy_flag = false;
  bool mapping_busy_flag = false;

  std_msgs::UInt8 working_state_msg;
  // 初始化发送正常
  working_state_msg.data = 1;
  working_state_pub_.publish(working_state_msg);

  LOG(INFO) << "ProcessMainLoop start";

  bool is_exit = false;

  sensor_msgs::LaserScan::ConstPtr scan_ptr;
  nav_msgs::Odometry::ConstPtr odom_ptr;

  while (!is_exit) {
    auto start_time = ros::WallTime::now();
    if (!laser_scans_buffer_.empty() && !odom_pose_buffer_.empty()) {
      {
        laser_mtx_.lock();
        odom_mtx_.lock();

        auto laser_time = laser_scans_buffer_.front()->header.stamp.toSec();
        double odom_time = 0;
        // 找到比laser_time新的里程计数据
        while (!odom_pose_buffer_.empty()) {
          odom_time = odom_pose_buffer_.front()->header.stamp.toSec();
          if (odom_time > laser_time) {
            break;
          } else {
            odom_pose_buffer_.pop();
          }
        }

        if (odom_time < laser_time || fabs(odom_time - laser_time) > 0.1) {
          laser_mtx_.unlock();
          odom_mtx_.unlock();
          usleep(10 * 1000);
          continue;
        }
        scan_ptr = laser_scans_buffer_.front();
        odom_ptr = odom_pose_buffer_.front();
        laser_scans_buffer_.pop();
        odom_pose_buffer_.pop();
        laser_mtx_.unlock();
        odom_mtx_.unlock();
      }

      karto::Pose2 odom_pose(odom_ptr->pose.pose.position.x, odom_ptr->pose.pose.position.y,
                             tf::getYaw(odom_ptr->pose.pose.orientation));

      // Check whether we know about this laser yet
      karto::LaserRangeFinder *laser_finder = GetLaserRangeFinder(scan_ptr);

      if (!laser_finder) {
        LOG(WARNING) << "Failed to create laser device discarding scan, :" << scan_ptr->header.frame_id.c_str();
        is_exit = true;
        continue;
      }

      if (AddScan(laser_finder, scan_ptr, odom_pose)) {
        new_scan_added = true;
        auto end_time = ros::WallTime::now();
        double delta_time = (end_time - start_time).toSec() * 1e3;
        if (delta_time > max_process_time_) {
          max_process_time_ = delta_time;
        }

        LOG(INFO) << fixed << setprecision(3) << "SCAN_ID: " << keyframe_cnt_ << " process time ms: " << delta_time
                  << " max_process_time: " << max_process_time_ << " scan buffer: " << laser_scans_buffer_.size()
                  << ", laser time: " << scan_ptr->header.stamp.toSec()
                  << ", odom time: " << odom_ptr->header.stamp.toSec();
        keyframe_cnt_++;

        if (is_simulation_) {
          PublishGraphVisualization();
        }
        PublishMapOnce();
      }
    } else {
      // 等待新的激光进来更新
      usleep(10 * 1000);
      // 等待建图结束信号
      if (slam_stop_request_flag_) {
        if (!first_map_updated_flag_) {
          LOG(ERROR) << "no map updated";
        } else {
          // Publish final ros map.
          PublishRosMap();
        }
        // 此处结束建图
        mapper_->TriggerFinishSlam(0);
        slam_process_end_ = true;
        slam_stop_request_flag_ = false;
        keyframe_cnt_ = 0;
        is_exit = true;
      }
    }

    // Publish Working state.
    mapping_busy_flag = laser_scans_buffer_.size() > 2;
    if (last_mapping_busy_flag != mapping_busy_flag) {
      last_mapping_busy_flag = mapping_busy_flag;
      if (last_mapping_busy_flag) {
        // 状态忙
        working_state_msg.data = 2;
      } else {
        // 状态正常
        working_state_msg.data = 1;
      }
      working_state_pub_.publish(working_state_msg);
    }
  }

  LOG(INFO) << ("ProcessMainLoop end!!");
}

bool LaserMapping::AddScan(karto::LaserRangeFinder *laser, sensor_msgs::LaserScan::ConstPtr &scan,
                           karto::Pose2 &karto_pose) {
  // Create a vector of doubles for karto
  std::vector<kt_double> readings;

  if (lasers_inverted_[scan->header.frame_id]) {
    for (std::vector<float>::const_reverse_iterator it = scan->ranges.rbegin(); it != scan->ranges.rend(); ++it) {
      readings.push_back(*it);
    }
  } else {
    for (std::vector<float>::const_iterator it = scan->ranges.begin(); it != scan->ranges.end(); ++it) {
      readings.push_back(*it);
    }
  }

  // create localized range scan
  auto *range_scan = new karto::LocalizedRangeScan(laser->GetName(), readings);
  range_scan->SetOdometricPose(karto_pose);
  range_scan->SetCorrectedPose(karto_pose);

  // Add the localized range scan to the mapper
  // bool processed = mapper_->Process(range_scan, lines_vec_);
  bool processed = mapper_->Process(range_scan, scan->header.stamp.toSec());

  if (processed) {
    karto::Pose2 corrected_pose = range_scan->GetCorrectedPose();
    LOG(INFO) << "SCAN_ID: " << keyframe_cnt_ << " add corrected pose, x, y, yaw: " << corrected_pose.GetX() << " "
              << corrected_pose.GetY() << " " << corrected_pose.GetHeading();

    // ---------------------- Compute the map->odom transform

    // 当前scan在map坐标系下的位置
    tf::Pose tf_baselink2map(tf::createQuaternionFromRPY(0, 0, corrected_pose.GetHeading()),
                             tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0));
    // tf_baselink2odom
    tf::Pose tf_baselink2odom(tf::createQuaternionFromRPY(0, 0, karto_pose.GetHeading()),
                              tf::Vector3(karto_pose.GetX(), karto_pose.GetY(), 0.0));

    // 将修正量更新上去
    tf_odom2map_mutex_.lock();
    tf_odom2map_ = tf_baselink2map * tf_baselink2odom.inverse();
    tf_odom2map_mutex_.unlock();

    // Add the localized range scan to the dataset (for memory management)
    dataset_->Add(range_scan);
    return true;
  } else {
    delete range_scan;
    return false;
  }
}

bool LaserMapping::MapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res) {
  boost::mutex::scoped_lock lock(map_mutex_);
  if (first_map_updated_flag_ && map_.map.info.width && map_.map.info.height) {
    res = map_;
    return true;
  } else
    return false;
}

void LaserMapping::EndSLAM() { slam_stop_request_flag_ = true; }

bool LaserMapping::GetStatusEnd() const { return slam_process_end_; }
