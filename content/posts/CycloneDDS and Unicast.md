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

### usb0 interface
Was having trouble with the usb0 interface not activating on boot. Tried some different options in the connection profile, but in the end just made a quick `systemd` service to bring it up if it is down.
##### check-usb0.timer - this is the part you enable
```
[Unit]
Description=Run usb0 nmcli check every minute, starting 1 minute after boot

[Timer]
OnBootSec=60s
OnUnitActiveSec=60s
Persistent=true
Unit=check-usb0.service

[Install]
WantedBy=timers.target
```

##### check-usb0.service
```
[Unit]
Description=Check and bring up usb0 via nmcli if down
Wants=network.target

[Service]
Type=oneshot
ExecStart=/home/rpi/systemd-services/check-usb0.sh
```

##### check-usb0.sh - stored in `/home/rpi/systemd-services`
```
#!/bin/bash

# Set up logging
LOG_TAG="check-usb0"

# Check if usb0 connection exists
if nmcli connection show usb0 &>/dev/null; then
    echo "usb0 connection exists" | systemd-cat -t "$LOG_TAG"

    # Check if usb0 is connected
    if nmcli -t -f NAME,DEVICE connection show --active | grep -q '^usb0:'; then
        echo "usb0 is connected" | systemd-cat -t "$LOG_TAG"
    else
        echo "usb0 exists but is not connected. Attempting to bring it up..." | systemd-cat -t "$LOG_TAG"
        if nmcli connection up usb0; then
            echo "Successfully brought up usb0 connection" | systemd-cat -t "$LOG_TAG"
        else
            echo "Failed to bring up usb0 connection" | systemd-cat -t "$LOG_TAG"
        fi
    fi
else
    echo "usb0 connection does not exist" | systemd-cat -t "$LOG_TAG"
fi
```

### Restarting Create3 application with API
The buttons in the WebServer don't work due to a text alignment issue, but you can trigger an application restart from the Raspberry Pi like so:
```
curl -X POST http://192.168.186.2/api/restart-app
```

Should respond like so if successful, and the chime from the robot or WebServer logs show the restart:
```
{"status":"Restarting application."}
```
#### To-Do
When usb0 is down for too long, the create3 still has those strange multicasting errors. Need to try with Multicast set to false from the create3. <- That should not work.

Maybe can try with peers listed again, but I hate this.
Can't assume usb0 will always be up, create3 could bomb out with multicast at any time it seems.

