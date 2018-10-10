# stair_perception

#### 硬件环境：
- Kinect V2
- IMU

#### 系统环境：
- Ubuntu16.04
- ROS kinetic
- [libfreenect2](https://github.com/OpenKinect/libfreenect2)
- opencv(ROS included)
- pcl(ROS included)

#### 编译步骤：

```
cd ~/catkin_ws/src
git clone https://gitee.com/Shiaoming/stair_perception.git
cd ~/catkin_ws
catkin_make --pkg plane_msg stair_info_msg
catkin_make
```