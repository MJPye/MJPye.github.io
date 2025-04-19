---
title: Launching cameras, streams and controls on boot in screens
date: 2024-11-16
---
### Screen to view cameras and drive the robot around
Currently have 5 screens that need to be run to make the robot work. Now want these to start at boot in the correct order.
- rtsp-server
- rtsp-to-webrtc
- signalling-server
- webrtc-to-ros2
- teleop-twist-joy
<!--more-->
#### rtsp-server
```
cd /home/rpi/webcam_testing
python3 test-low-bw-rtsp-server.py
```

#### rtsp-to-webrtc
```
cd /home/rpi/webcam_testing/RTSPtoWeb
GO111MODULE=on go run *.go
```

#### signalling-server
```
cd /home/rpi/webcam_testing/signalling-server
node index.js
```

#### webrtc-to-ros2
```
cd /home/rpi/webrtc-to-ros2
node index.js
```

#### teleop-twist-joy
```
cd /home/rpi
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='matthew'
```

### Solution
For now I made 2 bash scripts, one starts and on stops everything:
```
/home/rpi/robot_launch.sh
and
/home/rpi/robot_kill.sh
```