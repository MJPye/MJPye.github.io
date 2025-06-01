---
title: Reolink camera ROS2 interface
date: 2025-05-25
---
## ROS2 interface
There is a package which defines actions for a PTZ camera, so a generic interface which I should use. [ptz_action_server](https://github.com/clearpathrobotics/ptz_action_server/tree/ros2).
It doesn't include a driver for working with my E1 zoom camera though from what I see, so I should use this package: [reolinkapipy](https://github.com/ReolinkCameraAPI/reolinkapipy/tree/master)
##### ONVIF - not used
Tried this [ONVIF python package](https://github.com/FalkTannhaeuser/python-onvif-zeep/tree/zeep) and it works, but gives nothing over the Python on in terms of absolute position. This is because the PTZ doesn't seem to be capable. Remember you have to enable ONVIF in reolink app.
```
(venv) rpi@rpi-desktop:~/reolinkapi_testing$ onvif-cli -u 'admin' -a '<password>' --host '192.168.50.2' --port 8000 --wsdl python-onvif-zeep/wsdl/
```
The above starts the interactive session, from which we can run commands like below.
```
ONVIF >>> cmd ptz GetConfigurationOptions {'ConfigurationToken': '000'}
```
According to API though we can go to a preset value, so not sure how this works without absolute positioning.

### Preset values
If we wanted to do missions in the future with the PTZ camera, it would be necessary to save the orientation the camera is looking. Sadly we don't seem to be able to pull absolute data about camera position. This is strange though, as we can set "presets" with the API or App and go back to them. It will change Pan/Tilt/Zoom to get back to the original preset. 
At least if I want this feature, I can just map lots of camera poses and save them as presets, or whenever adding a POI save that as a preset to return to. Likely limited on max number of preset values though. According to docs is 64.
[API docs](file:///Users/matthewpye/Downloads/Camera%20HTTP%20API.pdf)
Need to test that presets save correctly when I move the camera with API and not the App.

### Creating the ROS2 package

```
cd /home/rpi/ros2_ws/src

ros2 pkg create --build-type ament_python --node-name reolink_cam_ros2_node reolink_cam_ros2

cd /home/rpi/ros2_ws
colcon build
```
Then to test it worked try running.
```
source /home/rpi/ros2_ws/install/setup.bash
ros2 run reolink_cam_ros2 reolink_cam_ros2_node
```
Which gives the Hello World output.

Continue the tutorial from [here](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/first_node_python.html).

For tab completion:
```
ros2 topic pub /joy sensor_msgs/msg/Joy "h
```
then hit tab.

To use left bumper to trigger looking around:
```
sensor_msgs.msg.Joy(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons=[0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
```

#### rclnodejs adding times to messages
Sometimes it is necessary to have timestamps on the messages to check we are not acting on old information. Here is how to add it, and the full example is in the`webrtc-to-ros2` node:
```
clock = node.getClock();
const now = clock.now();
const {seconds, nanoseconds} = now.secondsAndNanoseconds

const joyMessage = {
	header: {
		stamp: {
			sec: seconds,
			nanosec: nanoseconds
		},
```

Have added `Joy` message processing logic to `reolink_cam_ros2_node.py`, now just need to actually call the reolink API inside functions like `reolink_move_callback`.