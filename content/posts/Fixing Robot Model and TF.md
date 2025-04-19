---
title: Fixing Robot Model and TF - Rotated by 180 Degrees
date: 2024-12-07
---
<!--more-->
```
sudo apt-get install ros-humble-irobot-create-common-bringup
sudo apt-get install ros-humble-irobot-create-description
```

To rotate the Lidar frame, replaces the sensor launch file
```
ros2 run tf2_ros static_transform_publisher 0 0 0.2 3.14159 0 0 base_link laser
```
To display the robot model by adding robot description
```
ros2 launch irobot_create_common_bringup robot_description.launch.py
```

### Adding tools to throttle the LiDAR, needed on Raspberry Pi
```
https://github.com/ros-tooling/topic_tools/tree/humble
Do symlink install with these
Throttle the scan topic into another topic called /scan_throttled

ros2 run topic_tools throttle messages /scan 1.0 /scan_throttled
```

To drive with keyboard
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

To launch the SLAM toolbox, have edited topic to `/scan_throttled`
```
ros2 launch create3_lidar_slam slam_toolbox_launch.py
```
Adding `scan_frequency` to sllidar did not work.