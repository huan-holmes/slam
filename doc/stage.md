
# 官网链接

http://wiki.ros.org/stage_ros

# 原文链接 https://blog.csdn.net/weixin_42222609/article/details/95030957

## 介绍
stageros 是一个2D的机器人模拟器，主要通过.world文件来定义这个仿真世界。包括机器人，lidar，camera和障碍物等等。
Stage在设计中就考虑到了多智能体系统的问题，可以提供对多机器人系统的测试仿真。需要了解的是Stage只提供了真正简单，可计算的廉价的设备模式，而无法非常精密地仿真任何具体的设备终端。

## 使用
 rosrun stage_ros stageros [-g runs headless] < world > [standard ROS args]
world : 要加载的.world文件
-g : 如果设置，此选项将以“headless”运行模拟器，不显示任何GUIsgare
还有其他选项:usage/option

上面的的是wiki上说的用法
我自己使用的
rosrun stage_ros stageros < world >
world:要加载的.world文件路径
ex：
rosrun stage_ros stageros /opt/ros/${ROS_DISTRO}/share/stage_ros/world/willow-erratic.world 
stage是ros自带的，所以直接取根目录下能找到对应的功能包
