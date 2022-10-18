echo "current ros version ${ROS_DISTRO}"

# for mapping
sudo apt-get install libgoogle-glog-dev=0.3.5-1
sudo apt-get install libgflags-dev=2.2.1-1
sudo apt install libeigen3-dev=3.3.4-4
sudo apt-get install libatlas-base-dev=3.10.3-5
sudo apt-get install libsuitesparse-dev=1:5.1.2-2
# for localization
sudo apt install ros-melodic-mrpt2=2.4.10-1bionic.20220624.184112
sudo apt install ros-melodic-mrpt-msgs=0.4.4-1bionic.20220615.170028
sudo apt install ros-melodic-marker-msgs=0.0.6-0bionic.20210505.092339

# for simulation
sudo apt-get install ros-${ROS_DISTRO}-stage-ros
sudo apt-get install ros-${ROS_DISTRO}-navigation
