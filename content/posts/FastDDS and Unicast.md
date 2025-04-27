---
title: FastDDS and Unicast - Solutions for missing ros2 topics
date: 2025-03-10
---
### FastDDS and Unicast
Had an ongoing issue with the robot where `ros2 topic list` shows only 2 topics, when I did `usb0` interface down then up it would sometimes help, sometimes not.
The error can be seen in the create3 logs in the web interface:
```
user.notice create-platform: 1742579888.164076 [0] tev: ddsi_udp_conn_write to udp/239.255.0.1:7400 failed with retcode -1
```
This suggests that **ROS 2's DDS (Data Distribution Service) communication is failing**.
The address **239.255.0.1:7400** is a multicast address used by DDS for discovery and communication.

<!--more-->
#### Solutions with Unicast and FastDDS server
To try and fix this, and given the robot only needs to communicate with the RaspberryPi, I now run this on a screen in the raspberry pi:
```
fastdds discovery -i 0 -l <rpi_ip> -p 11811
```
And in the create3 web interface I turned on `Enable Fast DDS discovery server?` and gave it the following address: `<rpi_ip>:11811`.
Also added the following to `.bashrc` on the raspberry pi so nodes on the pi can communicate:
```
export ROS_DISCOVERY_SERVER=<rpi_ip>:11811
```
Have been running the robot for multiple weeks and `ros2 topic list` continuously provides the desired output.

### Permanent Fix
Have added this to the `robot-sleep` bash script. So far to me it seems this works best if ran before `usb0` is even brought up.