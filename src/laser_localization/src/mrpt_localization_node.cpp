//*****************************************************************************
// Copyright (c) 2022 MOI SmartRobotTechnology Co.,Ltd.
//*****************************************************************************

#include "mrpt_localization_node.h"

#include <fstream>

#include <geometry_msgs/PoseArray.h>

#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/time.h>  // NOLINT
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/beacon.h>

#include <mrpt/version.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#if MRPT_VERSION >= 0x199
#include <mrpt/system/COutputLogger.h>
using namespace mrpt::system;
#else
#include <mrpt/utils/COutputLogger.h>
using namespace mrpt::utils;
#endif

using mrpt::maps::COccupancyGridMap2D;
using namespace mrpt::obs;

#include "nlohmann/json.hpp"

#include "pose_cov_ops.h"

PFLocalizationNode::PFLocalizationNode()
    : PFLocalization(new PFLocalizationNode::Parameters(this)),
      nh_("~"),
      first_map_received_(false),
      loop_count_(0) {
  LOG(INFO) << ("localization_node:-----------------------start!!");
  // 初始化tf_odom2map_
  latest_tf_odom2map_.setIdentity();

  loop_thread_ = boost::thread([this] {
    init();
    PublishLoop();
  });
}

PFLocalizationNode::~PFLocalizationNode() {
  Termination();
  LOG(INFO) << ("localization_node:-----------------------exit!!");
}

void PFLocalizationNode::Termination() {
  if (b_exit_) {
    return;
  }

  b_exit_ = true;
  loc_status_end_ = false;
  if (loop_thread_.joinable()) {
    loop_thread_.join();
  }

  loc_status_end_ = true;
}

void PFLocalizationNode::init() {
  // 设置日志等级 初始化定位节点模式
  pdf_.setVerbosityLevel(VerbosityLevel::LVL_INFO);
  loc_diagnostic_mode_ = common_msg::LocMode::LOC_RUNNING;

  // 标记未初始化里程计值
  odom_last_observation_.x() = 10000000.;

  // 2, set a series of callback for localization
  // initailpose callback
  sub_init_pose_ = nh_.subscribe(
      "initialpose", 1, &PFLocalizationNode::callbackInitialpose, this);
  LOG(INFO) << ("Subscribed to initialpose topic.");

  // odometry callback
  sub_odometry_ =
      nh_.subscribe("odom", 1, &PFLocalizationNode::callbackOdometry, this);
  LOG(INFO) << ("Subscribed to odom topic.");

  // map callback
  sub_map_ = nh_.subscribe("map", 1, &PFLocalizationNode::callbackMap, this);
  LOG(INFO) << ("Subscribed to map topic.");

  // static_map service
  client_map_ = nh_.serviceClient<nav_msgs::GetMap>("static_map");
  LOG(INFO) << ("Subscribed to static_map service.");

  // laser callback
  // Subscribe to one or more laser sources:
  std::vector<std::string> sources;
  mrpt::system::tokenize(getLocalParam()->sensor_sources, " ,\t\n", sources);

  sub_sensors_.resize(sources.size());
  for (size_t i = 0; i < sources.size(); i++) {
    if (sources[i].find("scan") != std::string::npos) {
      sub_sensors_[i] = nh_.subscribe(sources[i], 1,
                                      &PFLocalizationNode::callbackLaser, this);
      LOG(INFO) << "Subscribed to " << sources[i];
    } else {
      sub_sensors_[i] = nh_.subscribe(
          sources[i], 1, &PFLocalizationNode::callbackRobotPose, this);
    }
  }

  // 3, advertise message from localization
  pub_particles_ =
      nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 1, true);

  pub_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "loc_pose", 2, true);

  // wait map here until new map come in
  PFLocalization::init();

  if (!getLocalParam()->use_map_topic) {
    // 尝试从文件中恢复位姿
    read_pose_from_file();
  }
}

void PFLocalizationNode::PublishLoop() {
  LOG(INFO) << ("PublishLoop thread start!");
  // 参数更新
  getLocalParam()->update(loop_count_);
  loop_count_++;
  for (ros::Rate rate(getLocalParam()->rate); ros::ok() && !b_exit_;
       loop_count_++) {
    if (update_counter_) {
      if (loop_count_ % getLocalParam()->particlecloud_update_skip == 0)
        publishParticles();
      if (getLocalParam()->tf_broadcast) publishTF();
      if (getLocalParam()->pose_broadcast) publishPose();
    }
    ros::spinOnce();
    rate.sleep();
  }
  LOG(INFO) << ("PublishLoop thread exit!");
}

// 从文件初始化位姿协方差需要设置很小，因为位置是确定的
bool PFLocalizationNode::read_pose_from_file() {
  std::ifstream pose_file(getLocalParam()->pose_record_file);
  if (pose_file.is_open()) {
    nlohmann::json pose_json;
    try {
      pose_file >> pose_json;
    } catch (const nlohmann::json::exception& e) {
      LOG(INFO) << ("pose data null");
      return false;
    }

    mrpt::math::CMatrixDouble33 cov;
    mrpt::poses::CPose2D p;
    cov(0, 0) = 0.1;
    cov(1, 1) = 0.1;
    cov(2, 2) = 0.1;

    if (pose_json["x"].is_null())
      p.x() = 0.0;
    else
      p.x() = pose_json["x"];
    if (pose_json["y"].is_null())
      p.y() = 0.0;
    else
      p.y() = pose_json["y"];
    if (pose_json["yaw"].is_null())
      p.phi() = 0.0;
    else
      p.phi() = pose_json["yaw"];

    initial_pose_ = mrpt::poses::CPosePDFGaussian(p, cov);
    update_counter_ = 0;
    state_ = INIT;
    init_filter_failed_ = false;
    odom_last_observation_.x() = 10000000.;
    LOG(INFO) << "get inital pose from file(x, y, yaw):" << p.x() << p.y()
              << p.phi();
    return true;
  }

  return false;
}

bool PFLocalizationNode::write_pose_to_file(mrpt::poses::CPose2D& pose) {
  std::ofstream pose_file(getLocalParam()->pose_record_file);
  if (pose_file.is_open()) {
    nlohmann::json pose_json;
    pose_json["x"] = pose.x();
    pose_json["y"] = pose.y();
    pose_json["yaw"] = pose.phi();
    pose_file << pose_json << std::endl;
    pose_file.close();
    return true;
  }
  return false;
}

bool PFLocalizationNode::waitForTransform(
    mrpt::poses::CPose3D& des, const std::string& target_frame,
    const std::string& source_frame, const ros::Time& time,
    const ros::Duration& timeout, const ros::Duration& polling_sleep_duration) {
  tf::StampedTransform transform;
  try {
    tf_listener_.waitForTransform(target_frame, source_frame, time, timeout,
                                  polling_sleep_duration);
    tf_listener_.lookupTransform(target_frame, source_frame, time, transform);
  } catch (tf::TransformException& e) {
    LOG(WARNING) << "Failed to get transform target_frame to source_frame "
                 << target_frame.c_str() << " ->" << source_frame.c_str() << " "
                 << e.what();
    return false;
  }
  mrpt_bridge::convert(transform, des);
  return true;
}

bool PFLocalizationNode::waitForMap() {
  LOG(INFO) << "waitFor ROS Map...";
  while (!first_map_received_ && ros::ok() && !b_exit_) {
    if (!getLocalParam()->use_map_topic) {
      // use service clent to call map
      nav_msgs::GetMap srv;
      if (client_map_.call(srv)) {
        client_map_.shutdown();
        LOG(INFO) << "Map service complete..";
        updateMap(srv.response.map);
        break;
      }
    }
    LOG(INFO) << "waiting for map callback..";
    ros::Duration(1).sleep();
    ros::spinOnce();
  }
  return true;
}

void PFLocalizationNode::callbackMap(const nav_msgs::OccupancyGrid& msg) {
  LOG(INFO) << "callbackMap";
  if (getLocalParam()->first_map_only && first_map_received_) {
    return;
  }
  // return if empty
  if (msg.data.empty()) {
    LOG(INFO) << "map side null";
    return;
  }
  LOG(INFO) << "New Map received";
  updateMap(msg);
  LOG(INFO) << "Map updated";
  // try to get initial pose from local file for this new map
  read_pose_from_file();
  LOG(INFO) << "initial pose updated from local file!!";
  first_map_received_ = true;
}

void PFLocalizationNode::callbackLaser(const sensor_msgs::LaserScan& _msg) {
  using namespace mrpt::maps;
  using namespace mrpt::obs;

  // static uint32_t laser_cnt{0};

  if (b_exit_ || !first_map_received_) {
    UpdateLocMode(common_msg::LOC_ERROR_MAP_NOT_RECEIVED);
    return;
  }

  // laser_cnt++;
  // if (laser_cnt % 2 != 0) {
  //   return;
  // }

  // 激光时间超时报错，use_sim = ture, 从仿真时钟源读当前时间， use_sim = false
  // , 读系统当前时间
  scan_time_last_input_ = ros::Time::now();
  if (scan_time_last_input_ > _msg.header.stamp + ros::Duration(0.5)) {
    LOG(ERROR) << "laserscan time too late, late more than 0.3s";
    UpdateLocMode(common_msg::LOC_ERROR_SCAN_TIME_TOO_LATE);
    return;
  }

  auto marpt_laser = CObservation2DRangeScan::Create();

  // LOG(INFO)("init_filter_failed_:%d, state_:%d", init_filter_failed_,
  // state_);

  if (laser_poses_.find(_msg.header.frame_id) == laser_poses_.end()) {
    // 更新激光传感器中心点位置(通常为base_laser_link) 在 base_frame(通常为
    // base_link) 中的位置
    updateSensorPoseTransform(_msg.header.frame_id);
  } else if (!init_filter_failed_ && state_ != IDLE) {
    // 是不是持续在tf_tree 中更新激光传感器在本体上的位置 默认 false
    if (getLocalParam()->update_sensor_pose) {
      updateSensorPoseTransform(_msg.header.frame_id);
    }
    // sensor_msgs::LaserScan 类型 转换为 mrpt 格式 scan
    mrpt_bridge::convert(_msg, laser_poses_[_msg.header.frame_id],
                         *marpt_laser);

    auto sf = CSensoryFrame::Create();
    CObservationOdometry::Ptr odometry;

    if (getOdometryTransform(odometry, _msg.header)) {
      CObservation::Ptr obs = CObservation::Ptr(marpt_laser);
      sf->insert(obs);
      // 更新粒子
      observation(sf, odometry);

      // 更新 map -> odom 位置关系
      mrpt::poses::CPose2D robot_pose;
      pdf_.getMean(robot_pose);
      tf::Transform tf_baselink2map, tf_baselink2odom;
      mrpt_bridge::convert(robot_pose, tf_baselink2map);
      mrpt_bridge::convert(odometry->odometry, tf_baselink2odom);
      latest_tf_odom2map_ = tf_baselink2map * tf_baselink2odom.inverse();

      // // publish map->odom here
      // static std::string odom_frame_id = tf::resolve(
      //     getLocalParam()->tf_prefix, getLocalParam()->odom_frame_id);
      // static std::string global_frame_id = tf::resolve(
      //     getLocalParam()->tf_prefix, getLocalParam()->global_frame_id);
      // tf::StampedTransform tmp_tf_stamped(
      //     latest_tf_odom2map_, _msg.header.stamp + ros::Duration(0.1),
      //     global_frame_id, odom_frame_id);
      // tf_broadcaster_.sendTransform(tmp_tf_stamped);
      std::lock_guard<std::mutex> lock(mtx_);
      is_pose_updated_ = true;

    } else {
      LOG(WARNING)
          << "Couldn't determine robot's pose associated with laser scan";
    }
  }
}

void PFLocalizationNode::callbackRobotPose(
    const geometry_msgs::PoseWithCovarianceStamped& _msg) {
  using namespace mrpt::maps;
  using namespace mrpt::obs;

  if (b_exit_ || !first_map_received_) {
    return;
  }

  odom_time_last_input_ = ros::Time::now();

  // Robot pose externally provided; we update filter regardless state_
  // attribute's value, as these
  // corrections are typically independent from robot motion (e.g. inputs from
  // GPS or tracking system)
  // XXX admittedly an arbitrary choice; feel free to open an issue if you
  // think it doesn't make sense

  static std::string base_frame_id =
      tf::resolve(getLocalParam()->tf_prefix, getLocalParam()->base_frame_id);
  static std::string global_frame_id =
      tf::resolve(getLocalParam()->tf_prefix, getLocalParam()->global_frame_id);

  tf::StampedTransform map_to_obs_tf;
  try {
    tf_listener_.waitForTransform(global_frame_id, _msg.header.frame_id,
                                  ros::Time(0.0), ros::Duration(0.5));
    tf_listener_.lookupTransform(global_frame_id, _msg.header.frame_id,
                                 ros::Time(0.0), map_to_obs_tf);
  } catch (tf::TransformException& ex) {
    LOG(ERROR) << ex.what();
    return;
  }

  // Transform observation into global frame, including covariance. For that,
  // we must first obtain
  // the global frame -> observation frame tf as a Pose msg, as required by
  // pose_cov_ops::compose
  geometry_msgs::Pose map_to_obs_pose;
  tf::pointTFToMsg(map_to_obs_tf.getOrigin(), map_to_obs_pose.position);
  tf::quaternionTFToMsg(map_to_obs_tf.getRotation(),
                        map_to_obs_pose.orientation);
  geometry_msgs::PoseWithCovarianceStamped obs_pose_world;
  obs_pose_world.header.stamp = _msg.header.stamp;
  obs_pose_world.header.frame_id = global_frame_id;
  pose_cov_ops::compose(map_to_obs_pose, _msg.pose, obs_pose_world.pose);

  // Ensure the covariance matrix can be inverted (no zeros in the diagonal)
  for (unsigned int i = 0; i < obs_pose_world.pose.covariance.size();
       ++i) {  // NOLINT
    if (i / 6 == i % 6 && obs_pose_world.pose.covariance[i] <= 0.0)
      obs_pose_world.pose.covariance[i] =
          std::numeric_limits<double>::infinity();
  }

  // Covert the received pose into an observation the filter can integrate
  auto feature = CObservationRobotPose::Create();

  feature->sensorLabel = _msg.header.frame_id;
  mrpt_bridge::convert(_msg.header.stamp, feature->timestamp);
  mrpt_bridge::convert(obs_pose_world.pose, feature->pose);

  auto sf = CSensoryFrame::Create();
  CObservationOdometry::Ptr odometry;
  getOdometryTransform(odometry, _msg.header);

  CObservation::Ptr obs = CObservation::Ptr(feature);
  sf->insert(obs);
  observation(sf, odometry);
}

void PFLocalizationNode::callbackInitialpose(
    const geometry_msgs::PoseWithCovarianceStamped& _msg) {
  const geometry_msgs::PoseWithCovariance& pose = _msg.pose;
  mrpt_bridge::convert(pose, initial_pose_);
  auto p = initial_pose_.mean;
  LOG(INFO) << "callbackInitialpose:" << p.x() << p.y() << p.phi()
            << initial_pose_.cov(0, 0) << initial_pose_.cov(1, 1)
            << initial_pose_.cov(2, 2);
  update_counter_ = 0;
  state_ = INIT;
  init_filter_failed_ = false;
  odom_last_observation_.x() = 10000000.;
}

void PFLocalizationNode::callbackOdometry(const nav_msgs::Odometry& _msg) {
  // 记录上次里程计的值
  static mrpt::poses::CPose2D last_odom{0, 0, 0};
  // 静止计数器
  static int no_motion_cnt{0};

  if (b_exit_ || !first_map_received_) {
    return;
  }

  // 启用强制更新
  if (getLocalParam()->update_while_stopped) {
    if (state_ == IDLE) {
      state_ = RUN;
    }
    return;
  }

  bool moving{false};
  mrpt::poses::CPose2D cur_odom;
  mrpt_bridge::convert(_msg.pose.pose, cur_odom);

  auto diff = cur_odom - last_odom;
  if (std::sqrt(diff.x() * diff.x() + diff.y() * diff.y()) >
          getLocalParam()->update_min_dist ||
      diff.phi() > getLocalParam()->update_min_angle) {
    moving = true;
    last_odom = cur_odom;
    // LOG(INFO)("moving");
  } else {
    no_motion_cnt++;
  }

  // 加载地图后state_被设置为INIT状态，然后在接收到激光数据后会执行initializeFilter后，状态变为RUN

  if (moving || no_motion_cnt > getLocalParam()->nomotion_update_skip) {
    no_motion_cnt = 0;
    if (state_ == IDLE) {
      state_ = RUN;
    }
  } else if (state_ == RUN &&
             update_counter_ >=
                 (size_t)(getLocalParam()->init_process_update_cnt)) {
    // 前100次接收到激光数据都会参与更新（激光频率10Hz的话，在接收重定位消息后大约10s，无论机器运动与否
    // 都会使用激光数据更新位姿），之后才开始根据运动与否选择是否使用激光数据更新位姿
    state_ = IDLE;
  }
}

void PFLocalizationNode::updateSensorPoseTransform(
    const std::string& _frame_id) {
  mrpt::poses::CPose3D pose;
  tf::StampedTransform transform;
  try {
    std::string base_frame_id =
        tf::resolve(getLocalParam()->tf_prefix, getLocalParam()->base_frame_id);
    tf_listener_.lookupTransform(base_frame_id, _frame_id, ros::Time(0),
                                 transform);
    tf::Vector3 translation = transform.getOrigin();
    tf::Quaternion quat = transform.getRotation();
    pose.x() = translation.x();
    pose.y() = translation.y();
    pose.z() = translation.z();
    tf::Matrix3x3 Rsrc(quat);
    mrpt::math::CMatrixDouble33 Rdes;
    for (int c = 0; c < 3; c++)
      for (int r = 0; r < 3; r++) Rdes(r, c) = Rsrc.getRow(r)[c];
    pose.setRotationMatrix(Rdes);
    laser_poses_[_frame_id] = pose;
  } catch (tf::TransformException& ex) {
    LOG(ERROR) << ex.what();
    ros::Duration(1.0).sleep();
  }
}

void PFLocalizationNode::updateMap(const nav_msgs::OccupancyGrid& _msg) {
#if MRPT_VERSION >= 0x199
  ASSERT_(metric_map_ptr_->countMapsByClass<COccupancyGridMap2D>());
  mrpt_bridge::convert(_msg,
                       *metric_map_ptr_->mapByClass<COccupancyGridMap2D>());
#else
  ASSERT_(metric_map_ptr_->m_gridMaps.size() == 1);
  mrpt_bridge::convert(_msg, *metric_map_ptr_->m_gridMaps[0]);
#endif
}

bool PFLocalizationNode::getOdometryTransform(
    CObservationOdometry::Ptr& _odometry, const std_msgs::Header& _msg_header) {
  std::string base_frame_id =
      tf::resolve(getLocalParam()->tf_prefix, getLocalParam()->base_frame_id);
  std::string odom_frame_id =
      tf::resolve(getLocalParam()->tf_prefix, getLocalParam()->odom_frame_id);
  mrpt::poses::CPose3D poseOdom;
  if (this->waitForTransform(poseOdom, odom_frame_id, base_frame_id,
                             _msg_header.stamp, ros::Duration(1.0))) {
    _odometry = CObservationOdometry::Create();
    _odometry->sensorLabel = odom_frame_id;
    _odometry->hasEncodersInfo = false;
    _odometry->hasVelocities = false;
    _odometry->odometry.x() = poseOdom.x();
    _odometry->odometry.y() = poseOdom.y();
    _odometry->odometry.phi() = poseOdom.yaw();
    return true;
  }
  return false;
}

void PFLocalizationNode::publishParticles() {
  if (pub_particles_.getNumSubscribers() > 0) {
    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = tf::resolve(getLocalParam()->tf_prefix,
                                            getLocalParam()->global_frame_id);
    poseArray.header.stamp = ros::Time::now();
    poseArray.header.seq = loop_count_;
    poseArray.poses.resize(pdf_.particlesCount());
    for (size_t i = 0; i < pdf_.particlesCount(); i++) {
      mrpt::math::TPose2D p = pdf_.getParticlePose(i);
      mrpt_bridge::convert(p, poseArray.poses[i]);
    }
    mrpt::poses::CPose2D p;
    pub_particles_.publish(poseArray);
  }
  // LOG(INFO)("particles cnt: %zu", pdf_.particlesCount());
}

/**
 * @brief Publish map -> odom tf; as the filter provides map -> base, we
 * multiply it by base -> odom
 */
void PFLocalizationNode::publishTF() {
  static std::string base_frame_id =
      tf::resolve(getLocalParam()->tf_prefix, getLocalParam()->base_frame_id);
  static std::string odom_frame_id =
      tf::resolve(getLocalParam()->tf_prefix, getLocalParam()->odom_frame_id);
  static std::string global_frame_id =
      tf::resolve(getLocalParam()->tf_prefix, getLocalParam()->global_frame_id);

  // todo , publish more presice time stamp
  tf::StampedTransform tmp_tf_stamped(
      latest_tf_odom2map_,
      ros::Time::now() + ros::Duration(getLocalParam()->transform_tolerance),
      global_frame_id, odom_frame_id);
  tf_broadcaster_.sendTransform(tmp_tf_stamped);

  // 保存位置
  if (loop_count_ % 20 == 0) {
    // 获取当前机器人位置
    mrpt::poses::CPose3D pose_3d;
    if (waitForTransform(pose_3d, global_frame_id, base_frame_id,
                         ros::Time(0.0), ros::Duration(0.1))) {
      mrpt::poses::CPose2D robot_pose(pose_3d);
      write_pose_to_file(robot_pose);
    } else {
      LOG(WARNING) << "get pose failed!!!";
    }
  }

  // 清除标记
  std::lock_guard<std::mutex> lock(mtx_);
  is_pose_updated_ = false;
}

/**
 * @brief Publish the current pose of the robot
 **/
void PFLocalizationNode::publishPose() {
  // cov for x, y, phi (meter, meter, radian)
#if MRPT_VERSION >= 0x199
  const auto [cov, mean] = pdf_.getCovarianceAndMean();
#else
  mrpt::math::CMatrixDouble33 cov;
  mrpt::poses::CPose2D mean;
  pdf_.getCovarianceAndMean(cov, mean);
#endif

  geometry_msgs::PoseWithCovarianceStamped p;

  // Fill in the header
  p.header.frame_id =
      tf::resolve(getLocalParam()->tf_prefix, getLocalParam()->global_frame_id);
  if (loop_count_ < 10 || state_ == IDLE) {
    p.header.stamp = ros::Time::now();  // on first iterations timestamp
  } else {
    // differs a lot from ROS time
    mrpt_bridge::convert(time_last_update_, p.header.stamp);
  }

  // Copy in the pose
  mrpt_bridge::convert(mean, p.pose.pose);

  // Copy in the covariance, converting from 3-D to 6-D
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      int ros_i = i;
      int ros_j = j;
      if (i == 2 || j == 2) {
        ros_i = i == 2 ? 5 : i;
        ros_j = j == 2 ? 5 : j;
      }
      p.pose.covariance[ros_i * 6 + ros_j] = cov(i, j);
    }
  }

  pub_pose_.publish(p);
}
