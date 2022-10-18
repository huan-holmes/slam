#include <mutex>
#include <thread>
#include <deque>
#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

#include "channel.h"
#include "utility.h"

class DavidDeskewer {
 public:
  explicit DavidDeskewer(ros::NodeHandle &node_handle);

  ~DavidDeskewer();

 private:
  static constexpr int kImuQueueLength = 100;
  std::mutex imu_lock_;
  ros::Subscriber sub_imu_;
  std::deque<sensor_msgs::Imu> imu_data_queue_;

  ros::Publisher laser_deskewed_pub_;
  ros::Subscriber sub_laser_;

  sensor_msgs::LaserScan::ConstPtr laser_msg_ptr_;

  std::array<double, kImuQueueLength> imu_time_{};
  std::array<double, kImuQueueLength> imu_rot_X_{};
  std::array<double, kImuQueueLength> imu_rot_Y_{};
  std::array<double, kImuQueueLength> imu_rot_Z_{};

  int current_imu_pointer_{};
  bool first_point_flag_{};
  Eigen::Affine3f trans_start_inverse_;

  pcl::PointCloud<PointXYZIT>::Ptr laser_cloud_input_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr deskewed_cloud_;

  double time_packet_{};
  double time_scan_cur_{};
  double time_scan_next_{};
  bool is_imu_available_{false};

  ros::NodeHandle &nh_;

  std::thread deskew_thread_;
  std::unique_ptr<Channel<sensor_msgs::LaserScan::ConstPtr>> deskew_data_ch_{};
  std::atomic<bool> is_termination_{false};

 private:
  void ResetSize();

  void ResetParameters();

  void ImuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg);

  void LaserScanHandler(const sensor_msgs::LaserScan::ConstPtr &laser_info_msg);

  bool GetLaserInfo(const sensor_msgs::LaserScan::ConstPtr &laser_info_msg);

  int DeskewInfo();

  void ImuDeskewInfo();

  void FindRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur);

  pcl::PointXYZI DeskewPoint(pcl::PointXYZI *point, double relTime);

  void ProjectPointcloud();

  void PublishScan();

  void DeskewThreadFunc();
};
