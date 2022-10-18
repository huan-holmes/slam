#include "deskew_node.h"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#include <fstream>
#include <unistd.h>
#include <sys/stat.h>
#endif

template <typename T>
void GetAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z) {
  *angular_x = thisImuMsg->angular_velocity.x;
  *angular_y = thisImuMsg->angular_velocity.y;
  *angular_z = thisImuMsg->angular_velocity.z;
}

LaserDeskewer::LaserDeskewer(ros::NodeHandle &node_handle) : nh_(node_handle) {
  // 订阅IMU数据
  sub_imu_ = nh_.subscribe<sensor_msgs::Imu>("/imu/raw_data", 50, &LaserDeskewer::ImuHandler, this);

  // 订阅激光数据
  auto laser_topic_name = nh_.param<std::string>("laser_topic_name", "/scan");
  sub_laser_ = nh_.subscribe<sensor_msgs::LaserScan>(laser_topic_name, 5, &LaserDeskewer::LaserScanHandler, this);

  // 发布运动补偿后的激光
  auto deskewed_topic_name = nh_.param<std::string>("deskewed_topic_name", "/scan_deskewed");
  laser_deskewed_pub_ = nh_.advertise<sensor_msgs::LaserScan>(deskewed_topic_name, 5);

  // 参数初始化
  AllocateMemory();
  ResetParameters();

  // 创建数据通道
  deskew_data_ch_ = std::make_unique<Channel<sensor_msgs::LaserScan::ConstPtr>>(false);

  // 创建单独的线程处理数据
  deskew_thread_ = std::thread(&LaserDeskewer::DeskewThreadFunc, this);

  // 打印日志
  LOG(INFO) << "laser_topic_name: " << laser_topic_name;
  LOG(INFO) << "deskewed_topic_name: " << deskewed_topic_name;
}

LaserDeskewer::~LaserDeskewer() {
  // 设置结束
  is_termination_ = true;
  deskew_data_ch_->send({});
  deskew_thread_.join();

  std::cout << "~LaserDeskewer" << std::endl;
  LOG(INFO) << "~LaserDeskewer";
}

void LaserDeskewer::PublishScan() {
  // 发送LaserScan
  sensor_msgs::LaserScan scan_msg;
  scan_msg.header.stamp = laser_msg_ptr_->header.stamp;
  scan_msg.header.frame_id = laser_msg_ptr_->header.frame_id;
  scan_msg.angle_min = laser_msg_ptr_->angle_min;
  scan_msg.angle_max = laser_msg_ptr_->angle_max;
  scan_msg.angle_increment = laser_msg_ptr_->angle_increment;
  scan_msg.scan_time = laser_msg_ptr_->scan_time;
  scan_msg.time_increment = laser_msg_ptr_->time_increment;
  scan_msg.range_min = laser_msg_ptr_->range_min;
  scan_msg.range_max = laser_msg_ptr_->range_max;
  int size = (scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment + 1;
  scan_msg.ranges.resize(size, 0.0);
  // scan_msg.intensities.resize(size);
  // std::cout << "s:" << size << std::endl;

  float x, y, angle;
  for (auto &p : deskewed_cloud_->points) {
    x = p.x;
    y = p.y;
    angle = atan2f(y, x);
    int index = std::ceil((angle - scan_msg.angle_min) / scan_msg.angle_increment);
    if (index >= 0 && index < size) {
      scan_msg.ranges[index] = sqrtf(x * x + y * y);
      // scan_msg.intensities[index] = p.intensity;
    }
  }
  laser_deskewed_pub_.publish(scan_msg);
}

void LaserDeskewer::AllocateMemory() {
  laser_cloud_in_.reset(new pcl::PointCloud<PointXYZIT>());
  deskewed_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

void LaserDeskewer::ResetParameters() {
  laser_cloud_in_->clear();
  deskewed_cloud_->clear();

  imu_pointer_cur_ = 0;
  first_point_flag_ = true;

  for (int i = 0; i < kImuQueueLength; ++i) {
    imu_time_[i] = 0;
    imu_rot_X_[i] = 0;
    imu_rot_Y_[i] = 0;
    imu_rot_Z_[i] = 0;
  }
}

void LaserDeskewer::ImuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg) {
  std::lock_guard<std::mutex> lock(imu_lock_);
  imu_queue_.push_back(*imuMsg);

  // 防止队列无限增大
  while (imu_queue_.size() > 500) {
    imu_queue_.pop_front();
  }
}

void LaserDeskewer::DeskewThreadFunc() {
  LOG(INFO) << "deskew_thread_func start!";

  int handle_cnt = 0;
  int res;
  while (ros::ok() && !is_termination_) {
    sensor_msgs::LaserScan::ConstPtr laser_msg;
    deskew_data_ch_->receive(laser_msg);

    // 退出检测
    if (is_termination_) {
      break;
    }

    // 解析信息
    GetLaserInfo(laser_msg);
    handle_cnt++;

    // 获取有效的IMU信息
    while ((res = DeskewInfo()) == 1) {
      // 由于不缓存数据，这里需要延时10ms，才能收到比较及时的IMU数据
      ros::Duration(0.01).sleep();
    }

    if (handle_cnt == 50) {
      handle_cnt = 0;
      LOG(WARNING) << "no good imu data for a long time!";
    }

    // 如果队列中最早的imu都比lidar晚, 放弃这一帧激光数据
    if (res == 2) {
      continue;
    }

    // ROS_INFO("pc time: %.6f - %.6f", time_scan_cur_, time_scan_next_);
    ProjectPointcloud();

    PublishScan();

    ResetParameters();
    handle_cnt = 0;
  }

  LOG(INFO) << "deskew_thread_func exit!";
}

void LaserDeskewer::LaserScanHandler(const sensor_msgs::LaserScan::ConstPtr &laser_info_msg) {
  static int cnt = 0;
  // 发送数据
  deskew_data_ch_->send(laser_info_msg);
  cnt++;

  if (cnt == 100) {
    cnt = 0;
    LOG(INFO) << "laser data ok!";
  }
}

bool LaserDeskewer::GetLaserInfo(const sensor_msgs::LaserScan::ConstPtr &laser_info_msg) {
  // 保存前值
  laser_msg_ptr_ = laser_info_msg;

  // 转换到带时间戳的点云
  int size = (int)laser_info_msg->ranges.size();
  laser_cloud_in_->reserve(size);
  float angle = laser_info_msg->angle_min;
  float offset_time = 0;
  for (auto i = 0; i < size; i++) {
    PointXYZIT p{};
    p.x = std::cos(angle) * laser_info_msg->ranges.at(i);
    p.y = std::sin(angle) * laser_info_msg->ranges.at(i);
    p.z = 0;
    p.time = offset_time;
    p.intensity = laser_info_msg->intensities.at(i);
    laser_cloud_in_->push_back(p);

    angle += laser_info_msg->angle_increment;
    offset_time += laser_info_msg->time_increment;
  }

  // 单线激光timePacket就是第一个点的时间
  time_packet_ = laser_msg_ptr_->header.stamp.toSec();
  // 时间改为第一个点的时间
  time_scan_cur_ = time_packet_;
  // 时间改为最后一个点的时间
  time_scan_next_ = time_packet_ + laser_cloud_in_->back().time;

  // ROS_INFO("time_scan_cur_:%.6f, time_scan_next_:%.6f", time_scan_cur_, time_scan_next_);

  return true;
}

int LaserDeskewer::DeskewInfo() {
  static int cnt = 0;
  std::lock_guard<std::mutex> lock(imu_lock_);

  // 没有imu数据 或者 最早的imu数据比当前第一个激光点数据新 或者 最新的imu数据比当前最后一个激光点旧
  if (imu_queue_.empty() || imu_queue_.front().header.stamp.toSec() > time_scan_cur_ ||
      imu_queue_.back().header.stamp.toSec() < time_scan_next_) {
    cnt++;
    // 2s输出一次
    if (cnt == 200) {
      cnt = 0;
      LOG(WARNING) << "Waiting for IMU data ...";
      // 时间过长则放弃这一帧
      return 2;
    } else if (cnt % 30 == 0) {
      // 300ms放弃一帧
      return 2;
    }

    // 如果队列中最早的imu都比lidar新,放弃这一帧激光数据
    if (imu_queue_.front().header.stamp.toSec() > time_scan_cur_) {
      return 2;
    }

    // 等待数据
    return 1;
  }

  // ROS_INFO("imu time:%.6f - %.6f", imu_queue_.front().header.stamp.toSec(), imu_queue_.back().header.stamp.toSec());
  ImuDeskewInfo();
  cnt = 0;

  return 0;
}

void LaserDeskewer::ImuDeskewInfo() {
  is_imu_available_ = false;

  while (!imu_queue_.empty()) {
    if (imu_queue_.front().header.stamp.toSec() < time_scan_cur_ - 0.01)
      imu_queue_.pop_front();
    else
      break;
  }

  if (imu_queue_.empty()) return;

  imu_pointer_cur_ = 0;

  for (int i = 0; i < (int)imu_queue_.size(); ++i) {
    sensor_msgs::Imu thisImuMsg = imu_queue_[i];
    double currentImuTime = thisImuMsg.header.stamp.toSec();

    if (currentImuTime > time_scan_next_ + 0.01) break;

    if (imu_pointer_cur_ == 0) {
      imu_rot_X_[0] = 0;
      imu_rot_Y_[0] = 0;
      imu_rot_Z_[0] = 0;
      imu_time_[0] = currentImuTime;
      ++imu_pointer_cur_;
      continue;
    }

    // get angular velocity
    double angular_x, angular_y, angular_z;
    GetAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

    // integrate rotation
    double timeDiff = currentImuTime - imu_time_[imu_pointer_cur_ - 1];
    imu_rot_X_[imu_pointer_cur_] = imu_rot_X_[imu_pointer_cur_ - 1] + angular_x * timeDiff;
    imu_rot_Y_[imu_pointer_cur_] = imu_rot_Y_[imu_pointer_cur_ - 1] + angular_y * timeDiff;
    imu_rot_Z_[imu_pointer_cur_] = imu_rot_Z_[imu_pointer_cur_ - 1] + angular_z * timeDiff;
    imu_time_[imu_pointer_cur_] = currentImuTime;
    // ROS_INFO("imu: (%.2f, %.2f, %.2f)", RAD2DEG(imu_rot_X_[imu_pointer_cur_]), RAD2DEG(imu_rot_Y_[imu_pointer_cur_]),
    // RAD2DEG(imu_rot_Z_[imu_pointer_cur_]));
    ++imu_pointer_cur_;
  }

  --imu_pointer_cur_;
  // ROS_INFO("imu_pointer_cur_: %d, (%.2f, %.2f, %.2f)", imu_pointer_cur_, RAD2DEG(imu_rot_X_[imu_pointer_cur_]),
  // RAD2DEG(imu_rot_Y_[imu_pointer_cur_]), RAD2DEG(imu_rot_Z_[imu_pointer_cur_]));

  if (imu_pointer_cur_ <= 0) return;

  is_imu_available_ = true;
}

void LaserDeskewer::FindRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur) {
  *rotXCur = 0;
  *rotYCur = 0;
  *rotZCur = 0;

  int imuPointerFront = 0;
  while (imuPointerFront < imu_pointer_cur_) {
    if (pointTime < imu_time_[imuPointerFront]) break;
    ++imuPointerFront;
  }

  if (pointTime > imu_time_[imuPointerFront] || imuPointerFront == 0) {
    *rotXCur = imu_rot_X_[imuPointerFront];
    *rotYCur = imu_rot_Y_[imuPointerFront];
    *rotZCur = imu_rot_Z_[imuPointerFront];
  } else {
    int imuPointerBack = imuPointerFront - 1;
    double ratioFront =
        (pointTime - imu_time_[imuPointerBack]) / (imu_time_[imuPointerFront] - imu_time_[imuPointerBack]);
    double ratioBack =
        (imu_time_[imuPointerFront] - pointTime) / (imu_time_[imuPointerFront] - imu_time_[imuPointerBack]);
    *rotXCur = imu_rot_X_[imuPointerFront] * ratioFront + imu_rot_X_[imuPointerBack] * ratioBack;
    *rotYCur = imu_rot_Y_[imuPointerFront] * ratioFront + imu_rot_Y_[imuPointerBack] * ratioBack;
    *rotZCur = imu_rot_Z_[imuPointerFront] * ratioFront + imu_rot_Z_[imuPointerBack] * ratioBack;
  }
}

pcl::PointXYZI LaserDeskewer::DeskewPoint(pcl::PointXYZI *point, double relTime) {
  // static uint64_t cnt = 0;
  if (!is_imu_available_) return *point;

  // cnt++;
  double pointTime = time_packet_ + relTime;

  float rotXCur, rotYCur, rotZCur;
  FindRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

  // if (cnt % 200 == 0) {
  //     ROS_INFO("t: %.6f - %.2f", pointTime, RAD2DEG(rotZCur));
  // }

  float posXCur = 0.0, posYCur = 0.0, posZCur = 0.0;

  if (first_point_flag_) {
    trans_start_inverse_ = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
    first_point_flag_ = false;
  }

  // transform points to start
  Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
  Eigen::Affine3f transBt = trans_start_inverse_ * transFinal;

  pcl::PointXYZI newPoint;
  newPoint.x = transBt(0, 0) * point->x + transBt(0, 1) * point->y + transBt(0, 2) * point->z + transBt(0, 3);
  newPoint.y = transBt(1, 0) * point->x + transBt(1, 1) * point->y + transBt(1, 2) * point->z + transBt(1, 3);
  // newPoint.z = transBt(2, 0) * point->x + transBt(2, 1) * point->y + transBt(2, 2) * point->z + transBt(2, 3);
  newPoint.z = 0.;

  // // 不去畸变
  // PointType newPoint;
  // newPoint.x = point->x;
  // newPoint.y = point->y;
  // newPoint.z = point->z;

  newPoint.intensity = point->intensity;

  return newPoint;
}

void LaserDeskewer::ProjectPointcloud() {
  int cloudSize = laser_cloud_in_->points.size();

  // int modify_cnt = 0;
  // range image projection
  for (int i = 0; i < cloudSize; ++i) {
    pcl::PointXYZI thisPoint;
    thisPoint.x = laser_cloud_in_->points[i].x;
    thisPoint.y = laser_cloud_in_->points[i].y;
    thisPoint.z = laser_cloud_in_->points[i].z;
    thisPoint.intensity = laser_cloud_in_->points[i].intensity;

    thisPoint = DeskewPoint(&thisPoint, laser_cloud_in_->points[i].time);

    // modify_cnt++;
    deskewed_cloud_->points.push_back(thisPoint);
  }
  // ROS_INFO("modify_cnt: %d", modify_cnt);
}

void CrashDump(const char *data, int size) { LOG(ERROR) << data; }

int main(int argc, char *argv[]) {
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

  ros::init(argc, argv, "laser_deskew_node");
  ros::NodeHandle nh("~");

  LOG(INFO) << "\033[1;32m Laser deskew node Started.\033[0m";

  LaserDeskewer laser_deskewer(nh);

  // 两个接收订阅
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  std::cout << "laser_deskew_node exited!!!" << std::endl;
  LOG(INFO) << "laser_deskew_node:-----------------------exited!!";
  return 0;
}
