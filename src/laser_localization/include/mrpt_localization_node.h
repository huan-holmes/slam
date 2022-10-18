#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt16.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <common_msg/common_type.h>
#include <dynamic_reconfigure/server.h>
#include <glog/logging.h>

#include "laser_localization/MotionConfig.h"
#include "mrpt_localization/mrpt_localization.h"

/// ROS Node
class PFLocalizationNode : public PFLocalization {
 public:
  struct Parameters : public PFLocalization::Parameters {
    static const int MOTION_MODEL_GAUSSIAN = 0;
    explicit Parameters(PFLocalizationNode* p);
    ros::NodeHandle node;
    void callbackParameters(mrpt_localization::MotionConfig& config,
                            uint32_t level);
    dynamic_reconfigure::Server<mrpt_localization::MotionConfig>
        reconfigure_server_;
    dynamic_reconfigure::Server<mrpt_localization::MotionConfig>::CallbackType
        reconfigure_cb_;
    void update(const unsigned long& loop_count);
    double rate{};
    double transform_tolerance{};  ///< projection into the future added to
    /// the published tf to extend its validity
    double no_update_tolerance{};  ///< maximum time without updating we keep
    /// using filter time instead of Time::now
    double no_inputs_tolerance{};  ///< maximum time without any observation
    /// we wait before start complaining
    int parameter_update_skip{};
    int particlecloud_update_skip{};
    // int map_update_skip{};
    std::string tf_prefix;
    std::string base_frame_id;
    std::string odom_frame_id;
    std::string global_frame_id;
    bool update_while_stopped{};
    bool update_sensor_pose{};
    bool pose_broadcast{};
    bool tf_broadcast{};
    bool use_map_topic{};
    bool first_map_only{};
    // 记录位置文件路径
    std::string pose_record_file{};
    // 前进多远更新
    double update_min_dist{};
    // 旋转多少更新
    double update_min_angle{};
    // 机器静止的时候，使用激光更新位姿频率控制。例如设置为10，则在静止状态下，每接收到10个里程计数据就是使用激光更新当前位姿一次
    int nomotion_update_skip{};
    // 初始化阶段更新数，每次使用激光数据更新一次位姿算作一次更新，系统设置初始化位姿后，开始计数，计数值小于该值之前，每次激光数据都会用于更新
    int init_process_update_cnt{};
    // 日志路径
    std::string log_path{};
  };

  explicit PFLocalizationNode();
  ~PFLocalizationNode();
  bool GetStatusEnd() const { return loc_status_end_; }

  void init() override;
  void PublishLoop();
  void callbackLaser(const sensor_msgs::LaserScan&);
  void callbackRobotPose(const geometry_msgs::PoseWithCovarianceStamped&);
  bool getOdometryTransform(CObservationOdometry::Ptr&,
                            const std_msgs::Header&);
  void callbackInitialpose(const geometry_msgs::PoseWithCovarianceStamped&);
  void callbackOdometry(const nav_msgs::Odometry&);
  void callbackMap(const nav_msgs::OccupancyGrid&);
  void updateMap(const nav_msgs::OccupancyGrid&);
  void publishTF();
  void publishPose();
  /// 从文件加载初始位姿
  bool read_pose_from_file();
  /// 保存当前位姿
  bool write_pose_to_file(mrpt::poses::CPose2D&);

  common_msg::LocMode GetLocMode() { return loc_diagnostic_mode_; }
  void UpdateLocMode(common_msg::LocMode mode) { loc_diagnostic_mode_ = mode; }

  void Termination();

 private:
  boost::atomic<bool> b_exit_{false};
  ros::NodeHandle nh_;
  boost::atomic<bool> first_map_received_{false};
  ros::Time scan_time_last_input_, odom_time_last_input_,
      tf_lastest_publish_time_;
  unsigned long long loop_count_;
  nav_msgs::GetMap::Response resp_;
  std::vector<ros::Subscriber> sub_sensors_;
  ros::Subscriber sub_init_pose_;
  ros::Subscriber sub_odometry_;
  ros::Subscriber sub_map_;
  ros::ServiceClient client_map_;
  ros::Publisher pub_particles_;
  ros::Publisher pub_pose_;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  std::map<std::string, mrpt::poses::CPose3D> laser_poses_;

  /// 定位模块停止标志
  boost::atomic<bool> loc_status_end_{false};

  /// ---- 公共资源
  std::mutex mtx_;
  /// 位姿更新标记
  bool is_pose_updated_{false};
  tf::Transform latest_tf_odom2map_;

  boost::thread loop_thread_;

  common_msg::LocMode loc_diagnostic_mode_;

 private:
  // methods
  Parameters* getLocalParam() const {
    return (PFLocalizationNode::Parameters*)param_;
  }
  void updateSensorPoseTransform(const std::string& frame_id);

  void publishParticles();

  bool waitForTransform(
      mrpt::poses::CPose3D& des, const std::string& target_frame,
      const std::string& source_frame, const ros::Time& time,
      const ros::Duration& timeout,
      const ros::Duration& polling_sleep_duration = ros::Duration(0.01));

  bool waitForMap() override;
};
