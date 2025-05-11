---
title: CycloneDDS and Unicast - Solutions for missing ROS2 topics
date: 2025-05-11
---
### CycloneDDS and Unicast
Had an ongoing issue with the robot where `ros2 topic list` shows only 2 topics, when I did `usb0` interface down then up it would sometimes help, sometimes not.
The error can be seen in the create3 logs in the web interface:
```
user.notice create-platform: 1742579888.164076 [0] tev: ddsi_udp_conn_write to udp/239.255.0.1:7400 failed with retcode -1
```
This suggests that **ROS 2's DDS (Data Distribution Service) communication is failing**.
The address **239.255.0.1:7400** is a multicast address used by DDS for discovery and communication.

<!--more-->
#### Solution with Unicast and FastDDS server - Not used
To try and fix this, and given the robot only needs to communicate with the RaspberryPi, I tried to run this on a screen in the raspberry pi:
```
fastdds discovery -i 0 -l <rpi_ip> -p 11811
```
And in the create3 web interface I turned on `Enable Fast DDS discovery server?` and gave it the following address: `<rpi_ip>:11811`.
Also added the following to `.bashrc` on the raspberry pi so nodes on the pi can communicate:
```
export ROS_DISCOVERY_SERVER=<rpi_ip>:11811
```
This works when used with FastDDS, however Vizanti does not work with this RMW implementation, so I needed to switch back to CycloneDDS

### Fixing CycloneDDS
The solution that worked for me in the end was to use the following setting in the `cyclonedds.xml` configuration file:
```
<AllowMulticast>spdp</AllowMulticast>
```
I also hardcoded the peer addresses for the Raspberry Pi and Create 3 robot interfaces. Some reading suggests `usb0` interface cannot handle multicast properly.
##### Configuration on the Raspberry Pi 
File located at 
```
/home/rpi/systemd-services/testing-sleep-wake/cyclonedds.xml
```
We can reference this as follows in `.bashrc`:
```
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/rpi/systemd-services/testing-sleep-wake/cyclonedds.xml
source ~/ros2_ws/install/setup.bash
```
Contents of the `xml` file are as follows:
```
<CycloneDDS>
  <Domain>
    <General>
      <AllowMulticast>spdp</AllowMulticast>
      <Interfaces>
        <NetworkInterface name="usb0" priority="10"/>
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.186.2"/>  <!-- Create 3 IP -->
      </Peers>
    </Discovery>
    <Tracing>
      <Verbosity>info</Verbosity>        <!-- Options: trace, debug, info, warning, error -->
      <OutputFile>/home/rpi/cyclonedds.log</OutputFile> <!-- Make sure the path is writable -->
    </Tracing>
  </Domain>
</CycloneDDS>
```

##### Configuration on the Create3 Robot
This configuration can be found in the beta features of the Create3 web interface:
```
<CycloneDDS>
  <Domain>
    <General>
      <AllowMulticast>spdp</AllowMulticast>
    <Interfaces>
      <NetworkInterface name="usb0" priority="10"/>
    </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.186.3"/>  <!-- RPi IP -->
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```