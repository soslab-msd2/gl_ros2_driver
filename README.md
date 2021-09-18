# GL-3 (Mechanical scanning 2D LiDAR)

## Guide
* Installation
```
$ cd ${ROS2 workspace}/src
$ git clone --recurse-submodules https://github.com/soslab-project/gl_ros2_driver.git
$ cd $(ROS2 workspace)
$ colcon build --symlink-install
```
- Set permission of USB port
```
$ sudo chmod a+rw /dev/ttyUSB0
```
- Set permission of USB port permanently
```
$ sudo usermod -a -G dialout $USER
```
and reboot.
- Run GL-3 publisher node
```
$ ros2 launch gl_ros2_driver gl_ros2_driver.py
```
- Run GL-3 publisher node with RViz
```
$ ros2 launch gl_ros2_driver view_gl_ros2_driver.py
```
- Change setting parameters in `gl_ros2_driver/launch/gl_ros2_driver.py`

## Published Topics
- _scan_ (sensor_msgs/LaserScan): it publishes scan topic from the laser.

## Test environment
- ROS2 Dashing Diademata
- Ubuntu 18.04 LTS

## Application demo
- [GL-3, Demo] 2D LiDAR, Mapping (https://youtu.be/AfsqlU_f-Go)
- [GL-3, Demo] Create 3D Point Cloud with 2D LiDAR (https://youtu.be/_HBWe95GqXM)
- [GL-3, Demo] Create 3D Point Cloud with 2D LiDAR (pulse-width version) (https://youtu.be/q4MeeS9eP4c)
- [GL-3, Demo] Human Tracking Demo of Multiple Mobile Robots (https://youtu.be/ZzEvm8gMPaA)
- [GL-3, Demo] 2D LiDAR Object Detection (https://youtu.be/V29QzU9AcQo)
