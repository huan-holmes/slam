# 安装

```bash
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt install libeigen3-dev
# BLAS & LAPACK
sudo apt-get install libatlas-base-dev
sudo apt-get install libsuitesparse-dev
```

# 编译

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="laser_mapping" -j8
```

# 运行

## 使用rosbag建图

```bash
# 启动节点
roslaunch laser_mapping bag_mapping.launch

# 播放测试rosbag
rosbag play --clock ./datasets/aces.bag -r 3.0 /scan:=/slam/scan
rosbag play --clock ./datasets/mit-killian.bag -r 3.0 /scan:=/slam/scan

```

## 实时建图

```bash
# 启动节点
roslaunch laser_mapping real_time_mapping.launch
```

## 手动闭环

```bash
rostopic pub -1 /slam/control_pose geometry_msgs/Pose "position:
  x: -53.0
  y: 31.0
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0" 

```

## 旋转地图

```bash
source devel/setup.bash

# 自动旋转
rostopic pub -1 /slam/rotate_map common_msg/RotateMap "type: 1
rotate_angle_deg: 0.0" 

# 手动旋转
rostopic pub -1 /slam/rotate_map common_msg/RotateMap "type: 2
rotate_angle_deg: 30.0" 

```

