
# 依赖
编译时提示mrpt-msgs找不到，需要安装如下内容：
```bash
sudo apt install ros-$ROS_DISTRO-mrpt-msgs
```

# 编译

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="laser_localization" -j8
```

# 启动脚本

```bash
# 启动实时
roslaunch laser_localization run.launch

# 启动仿真
roslaunch laser_localization stage_sim.launch
```

# call service

```bash
rosservice call /loc/mode_cmd "request_type: 0"
```
