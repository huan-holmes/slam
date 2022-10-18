#https://blog.csdn.net/qq_44711932/article/details/122502596
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
sudo sh -c '. /etc/lsb-release && echo "deb http://packages.ros.org/ros/ubuntu $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
