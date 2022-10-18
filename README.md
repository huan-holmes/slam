# david 

## 1, 介绍
david 是2D激光SLAM建图定位与仿真系统
主要包含
  - 建图算法包
  - 定位算法包
  - 通信文件定义包
  - 测试包

项目进度链接
https://www.teambition.com/project/62a45cd0159cbbe06765a8cb/app/5eba5fc96a92214d420a321c

## 2, 软件环境
### 2.1, Linunx 测试环境
Ubntu20.04
### 2.2, ROS 版本
ROS1.0 noetic 安装教程
http://wiki.ros.org/noetic/Installation/Ubuntu
### 2.3, 仿真环境数据集原始地址
http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php
### 2.4, 百度网盘链接
链接: https://pan.baidu.com/s/1raybn0LjzIUCDvaiQODt4w 提取码: bec6 

## 3, 安装编译

安装依赖库--很重要

```bash
./tools/environment_setup.sh
```
一键编译

```bash
./tools/compile_all.sh
```

## 4, 测试
### 4.1 通讯测试
通信消息 service 测试
```
source devel/setup.bash
rossrv show common_msg/LocControl
rossrv show common_msg/SlamControl
屏幕打印
uint8 request_type
---
uint8 result_mode
```

```
source devel/setup.bash
roscore
#打开新的窗口
rosrun slam_test slam_test_server_node
#打开新的窗口
rosrun slam_test slam_test_client_node 
屏幕打印
[ INFO] [1655524334.339143098]: Current SLAM State: SLAM_SLEEPING
```

### 4.2 建图模块
**建图模块输入输出**

默认输入
+ 激光数据接口
  - topic: /slam/scan
  - type: sensor_msgs::LaserScan
  - frame_id : /base_laser_link
+ 里程计数据接口
  - topic: /odom
  - type: nav_msgs::Odometry
  - frame_id : /odom
+ 模式控制接口
  - topic: /slam/mode_cmd
  - type: common_msg::SlamControl
+ TF输入
  - odom -> base_link
  - base_link -> base_laser_link
+ 手动闭环位置输入
  -topic: /slam/control_pose
  - type: geometry_msgs::PoseWithCovarianceStamped

默认输出
+ 地图发布
  - topic: /slam/map
  - type: nav_msgs::OccupancyGrid
+ tf发布
  - map -> odom
+ 诊断输出
  - topic: system/diagnostics
  - type: common_msg::Diagnostic (slam 默认NODE_ID 11)


**4.2.1 建图bag回放建图**

注意事项

1，通过配置参数文件中 simulation 可以设定是否启动仿真
2，对于不同的bag, 需要调整不同参数回环距离和匹配阈值值，具体去看配置的参数文件位置, 
install/share/laser_mapping/config/bag.yaml
默认为intel.bag 的参数

# 自动加载ros bag 建图
先在 bag_mapping.launch 中 更改默认ros bag的地址，比如 
```
<param name="rosbag_path" type="string" value="/home/moi/Downloads/kk/2022-08-25-16-30-15.bag" />
roslaunch laser_mapping bag_mapping.launch
```
# 使用rosbag play 回放建图

```bash
# 需要保持 rosbag_path 参数 为空，否则会自动加载bag
roslaunch laser_mapping bag_mapping.launch
# 使用rosbag play播放测试rosbag
目前默认激光外参为默认的0,0,0  base_link -> base_link
rosbag play --clock  your.bag -r 5.0 --topics /slam/scan /scan /odom 
```

**4.2.2 真实模式控制建图**

```bash
# 启动建图节点
roslaunch laser_mapping real_time_mapping.launch

## 使用service 

### 直接请求建图开始
rosservice call /slam/mode_cmd "request_type: 2"
### 直接获取当前SLAM模式
rosservice call /slam/mode_cmd "request_type: 0"
### 直接结束建图
rosservice call /slam/mode_cmd "request_type: 1"


## 使用测试程序 执行测试程序
rosrun slam_test mapping_control_test

# 正常显示如下
[ INFO] [1656227643.791842430]: mapping control test start!
[ INFO] [1656227643.792252794]: Requset current mapping status...
[ INFO] [1656227643.793048719]: Current SLAM State: SLAM_SLEEPING
[ INFO] [1656227648.793196198]: Requset to start mapping...
[ INFO] [1656227648.810930394]: Current SLAM State: SLAM_RUNNING
[ INFO] [1656227653.811069918]: Requset to stop mapping...
[ INFO] [1656227653.812636909]: Current SLAM State: SLAM_UPDATING
[ INFO] [1656227653.812664604]: wait mapping node stopped!
[ INFO] [1656227656.812804357]: Requset to stop mapping...
[ INFO] [1656227656.815433520]: Current SLAM State: SLAM_SLEEPING
[ INFO] [1656227661.815615402]: mapping control test finished!
```

### 4.3 定位模块

默认输入
+ 初始位置
  - topic: initialpose
  - type: geometry_msgs::PoseWithCovarianceStamped
+ 激光数据接口
  - topic: /loc/scan
  - type: sensor_msgs::LaserScan
  - frame_id : /base_laser_link
+ 里程计数据接口
  - topic: /odom
  - type: nav_msgs::Odometry
  - frame_id : /odom
+ 地图接口
  - topic: /map
  - type: nav_msgs::OccupancyGrid
+ TF输入
  - odom -> base_link
  - base_link -> base_laser_link

默认输出
+ 位置信息
  - topic: /loc/loc_pose
  - type: geometry_msgs::PoseWithCovarianceStamped
+ tf发布
  - map -> odom
+ 诊断输出
  - topic: system/diagnostics
  - type: common_msg::Diagnostic (loc 默认NODE_ID 12)

**4.3.1 仿真测试**

```
roslaunch slam_test loc_stage_sim.launch
```

**4.3.2 真实模式定位**

```bash
# 启动定位节点
roslaunch laser_localization run.launch

### 直接获取当前LOC模式
rosservice call /loc/mode_cmd "request_type: 0"
### 直接请求定位模块启动
rosservice call /loc/mode_cmd "request_type: 2"
### 直接结束
rosservice call /loc/mode_cmd "request_type: 1"

# 执行测试程序
rosrun slam_test localization_control_test

# 正常显示如下
[ INFO] [1656833546.006659056]: localization control test start!
[ INFO] [1656833546.006970408]: Requset current localization status...
[ INFO] [1656833546.054329522]: Current LOC State: LOC_SLEEPING
[ INFO] [1656833551.054616566]: Requset to start localization...
[ INFO] [1656833551.094806235]: Current LOC State: LOC_RUNNING
[ INFO] [1656833556.095059696]: Requset to stop localization...
[ INFO] [1656833556.099746216]: Current LOC State: LOC_UPDATING
[ INFO] [1656833556.099813795]: wait localization node stopped!
[ INFO] [1656833559.099960713]: Requset to stop localization...
[ INFO] [1656833559.112247474]: Current LOC State: LOC_SLEEPING
[ INFO] [1656833564.112409476]: localization control test finished!
```

## 5, git merge

开发分支必须自己创建新的分支
git branch -b mdev
合并上传前解决好冲突
1,首先 fetch 最新代码
git fetch
2, rebase 最新分支 
git rebase maser 或者 git rebase dev
在自己的分支解决好冲突
3，合并到dev 或者master
git checkout dev
git merge mdev
git push origin dev

## ssh 
ssh 生成秘钥
ssh-keygen -t rsa



