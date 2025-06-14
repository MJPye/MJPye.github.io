---
title: Adding Autonomy with Nav2
date: 2025-06-07
---
After making some hardware & software changes to the robot since the last time I used the Nav2 stack, I first need to make a couple quick fixes.

1. The ROS2 node for PTZ control is still not in a launch script, so need to add that somewhere.
2. After adding the PTZ camera mount behind the LiDAR, we need to filter out the reading from it.

We can see the points in red from the camera mount in the image below. The effect is that Nav2 goals don't work because the created `costmaps` all feature unreachable areas.

![Image Description](/images/lidar_points_camera_mount.png)

#### Cleaning up LiDAR and SLAM repo use
On the rpi I had a folder at `/home/rpi/create3_ros2_ws/src` with SLAM and LiDAR related stuff. Going to clean this up and add as Submodules.
The dependencies can be found in the following folder as submodules:
```
/Users/matthewpye/Documents/WebRTC_learning/robot_with_webrtc/create3_ros2_ws
```

#### Fix for LiDAR points self detection camera mount
To fix the issue number 2 above, was able to just change minimum sensing distance from 5cm to 15cm via the `range_min` parameter in [this commit](https://github.com/Slamtec/sllidar_ros2/commit/6aba4079579ee883e2889273d8f02ba137351656).

### Recurring issue with nav2_bringup
I keep getting this when launching `nav2_bringup`:
```
[controller_server-1] [INFO] [1749907661.205011809] [local_costmap.local_costmap]: Timed out waiting for transform from base_link to odom to become available, tf error: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
```
Firstly I'm just going to try restarting the robot application as I see nothing being published on the `odom` topic:
```
curl -X POST http://192.168.186.2/api/restart-app
```
Restarting the application did fix the issue. Perhaps a check that runs could be one for the `ros2 topic hz` of `odom` to make sure we don't need to restart.

#### Current state with Nav2 goals possible
In the video, you can see my driving the robot around manually using an xbox controller.
Once the Robot has mapped it's surroundings, we can sent it autonomously to a goal position.
<video src="/images/robot_nav_goal_demo.mov" autoplay muted loop playsinline style="max-width:100%; height:auto;"></video>