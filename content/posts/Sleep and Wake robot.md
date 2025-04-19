---
title: Robot sleep and wake states
date: 2025-02-15
---
We need the following to run all the time:
- Signalling Server
- Vizanti
- WebRTC to Ros2

The following can wait for a wake up signal:
- rtsp-server
- rtsp-to-webrtc

The following can wait until cameras are up and user has asked for them:
- LiDAR
- SLAM
- Behaviour

<!--more-->
What we need is a state that is known to both server and robot. The state could be sleep/wake.
The reason we need this is so when the webpage refreshes, we can see the RTSP streams are still online and don't try to restart them again.

So in the `webrtc-to-ros2` package, once the data channel is setup, you can easily call a service. There could be 1 service to start the rtsp servers and one to kill.

Thinking we could start the rtsp stuff inside webrtc-to-ros2/index.js for now in the `socket.on('data-request')` part. So when new connection made, start those scripts. On Hangup, kill them. Can try that first.

### Note on Sleep/Wake - March 2025
The have the state persist on the robot and appear in UI, need to add some data coming in from robot to UI. The data channel is setup for comms both ways, but there are no features on the UI yet which can display any useful information, such as:
- Is the robot asleep/awake?
- Is SLAM running?
- Values like `htop`
- Temperature
#### Progress 2025-03-03
Made a branch called `mp-sleep-wake` which changed the `webrtc-to-ros2` node so that when an offer is created over the websocket, we launch the RTSP related screens. When there is a disconnect or `bye`, these screens are killed. Added new scripts for this.
Going to leave it with `robot-sleep.sh` screens running overnight and try logging in the morning. Will also check `htop` difference.
I think there is a better solution here for sure to make it more stateful, especially when thinking about launching SLAM.
HTOP shows all less than 10%, Mem 700M
Temp GPU: 50 degC, check with `vcgencmd measure_temp`
Temp CPU: 
```
cpu=$(</sys/class/thermal/thermal_zone0/temp)
echo "$((cpu/1000)) c"
```
Values looked the same in the morning so no issue running some parts all the time.