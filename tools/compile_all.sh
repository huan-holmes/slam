echo "current ros version ${ROS_DISTRO}"

catkin_make -DCATKIN_WHITELIST_PACKAGES="common_msg" install
catkin_make -DCATKIN_WHITELIST_PACKAGES="laser_mapping" -j8 install
catkin_make -DCATKIN_WHITELIST_PACKAGES="david_deskew"  -j8 install
catkin_make -DCATKIN_WHITELIST_PACKAGES="laser_localization" -j8 install
# test node
catkin_make --source src_test -DCATKIN_WHITELIST_PACKAGES="slam_test" install
