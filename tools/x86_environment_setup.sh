echo "current ros version ${ROS_DISTRO}"

# for mapping
sudo apt-get install libgoogle-glog-dev=0.4.0-1build1
sudo apt-get install libgflags-dev=2.2.2-1build1
sudo apt install libeigen3-dev=3.3.7-2
sudo apt-get install libatlas-base-dev=3.10.3-8ubuntu7
sudo apt-get install libsuitesparse-dev=1:5.7.1+dfsg-2
# for localization
sudo apt install ros-noetic-mrpt2=2.4.10-1focal.20220630.135834
sudo apt install ros-noetic-mrpt-msgs=0.4.4-1focal.20220613.174008
sudo apt install ros-noetic-marker-msgs=0.0.6-7focal.20210423.223919

# for simulation
sudo apt-get install ros-${ROS_DISTRO}-stage-ros
sudo apt-get install ros-${ROS_DISTRO}-navigation
