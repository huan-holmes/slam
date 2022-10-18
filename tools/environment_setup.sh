echo "current ros version ${ROS_DISTRO}"

# for mapping
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt install libeigen3-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libsuitesparse-dev
# for localization
sudo apt install ros-$ROS_DISTRO-mrpt2
sudo apt install ros-$ROS_DISTRO-mrpt-msgs
sudo apt install ros-${ROS_DISTRO}-marker-msgs

# for simulation
sudo apt-get install ros-${ROS_DISTRO}-stage-ros
sudo apt-get install ros-${ROS_DISTRO}-navigation
