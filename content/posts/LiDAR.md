---
title: LiDAR and Initial Vizanti setup
date: 2024-08-28
---
Github repo for slamtech LiDARs: https://github.com/Slamtec/sllidar_ros2
Link to purchase in Germany: https://eu.robotshop.com/de/products/rplidar-a1m8-360-grad-laserscanner-entwicklungskit 
or 
[Amazon C1](https://www.amazon.de/youyeetoo-C1-Positioning-Navigation-Avoidance/dp/B0CNXLJJ61/ref=sr_1_1_sspa?crid=8T6Y9NZNH9JP&dib=eyJ2IjoiMSJ9.8SC3iX_muyEsnFGhG_NTuS0zrsEVQFm3KbafzIH6jEJ_Aw5xVIsdHUo85x6mLPoxv2J32CnrrP8ncvcGfhYBoAWGvMIN2OhdGzD4-MMG5Aa1WJHCzZjduAqCFiDGC4sUKehR6NbjBn9CUAa1fNHFQnZbIuOMDSJiA70bUGqVD-Wb7sqbOmALoqc2u2SMVqjR9yoGCyxNwHgL2W_7toe49nbyuDCtkWLzC86cZsO18EcrUgEBtIRr3XCFOelcJCCg99e5fPslOxjk5t9lvQlLa_flW9cvkrHgssWfvsZDKXg.lmPKa2EOj5Gw99nz2Pz89Np7w4vJomJgA-TLYUNLD7U&dib_tag=se&keywords=rplidar+c1&qid=1724879421&sprefix=rplidar+c1%2Caps%2C106&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1)

Following the instructions on the GitHub page for the C1 LiDAR was enough to get the /scan showing in Rviz.
Is running at 10Hz, look around in the GitHub after downloading to see where these parameters are set.

<!--more-->

Create3 has a repo for 2D LiDAR SLAM already: https://github.com/iRobotEducation/create3_examples/tree/humble/create3_lidar_slam
Looks like I just need to switch the `rplidar` package to `sllidar_ros2`, they both seem to publish to the scan topic.

```
Ran
rosdep install --from-path src --ignore-src -yi

which does
sudo -H apt-get install -y ros-humble-slam-toolbox
```

Example Occupancy grid map in JS, also allows you to click the canvas to set a new goal pose: https://tommycohn.com/Occupancy-Grid-SLAM-JS/

We want a canvas drawn in JS, which shows the occupancy grid data, Robot position. We then want to be able to click on the canvas and the robot will drive to that position. It seems like writing all this will be hard, but will give full flexibility.

LiDAR attached to RPi
Can now start the LiDAR like so (without Rviz):
```
ros2 launch sllidar_ros2 sllidar_c1_launch.py
```
And with Thincast Remote Desktop client, we can launch the one with Rviz:
```
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py
```


### Vizanti
https://github.com/MoffKalast/vizanti/tree/ros2
```
sudo apt install ros-humble-rqt-reconfigure
```

Next try to embed this page in an iframe and see what is possible to control, first through direct connection to Pi and page, then through laptop with no VPN and reverse proxy.

With fastrtps we get the battery level.
With cyclone we get the TF. <- Not sure if that is confirmed, need to echo topic.

Note: (this gets reverted back to `cyclonedds`)
Changed to:
```
sudo apt install ros-$ROS_DISTRO-rmw-fastrtps-dynamic-cpp
export RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp
```
Then ran with the RWS launch instead, seems to work for TF, battery has wrong QoS it seems:
```
ros2 launch vizanti_server vizanti_rws.launch.py base_url:=/public
```

Until laser is added to TF
```
ros2 launch sllidar_ros2 sllidar_c1_launch.py frame_id:=base_link
```

Error with battery:
```
[rws_server-1] [WARN] [1731805610.686256228] [vizanti_rws_server]: New publisher discovered on topic '/battery_state', offering incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```

Note: Later on we switched back to `cyclonedds`.