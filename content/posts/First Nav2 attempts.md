---
title: First attempts with Nav2
date: 2024-12-20
---
Started SLAM from systemd folder script.
Went to `/opt/ros/humble/share/nav2_bringup/params` and made sure costmaps were generated using `scan_throttled`. Then ran with `ros2 launch nav2_bringup navigation_launch.py` from [here](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html).
Can also try `ros2 launch nav2_bringup slam_launch.py`.

```
ros2 launch nav2_bringup navigation_launch.py
```

<!--more-->
Now struggling a bit with the RWM implementation. The create3 robot doesn't have `rmw_fastrtps_dynamic_cpp`, and this seems to be why I cannot get/set parameters. Vizanti won't work with just `rmw_fastrtps_cpp`, so now trying `rmw_cyclonedds_cpp` instead, but Vizanti still throws an error.

Had a typo in `.bashrc`, `RWM` instead of `RMW` was used, fixed now. We use `cyclonedds`.

Try navigating to goal again, try changing all instances in the params file of `base_footprint` to `base_link`.

A guide to [ROS2 Navigation tuning](https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/#slam_toolbox).

First, check about synchronising the clock between robot base and Rpi, might be affecting transforms. <- Doesn't seem to be that just by checking if `ntp` is working.

Looking at the `frames.pdf` output, we see `map -> odom` is only published at 1.1Hz. This is likely the issue. Find who is publishing this. For now try increasing the value of `transform_tolerance` to check if Nav goal actually works. Got from this [forum post](https://answers.ros.org/question/344688/). <- Increased `transform tolerance`, helped for sure, not a long term solution though.
