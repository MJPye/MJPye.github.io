---
title: Gamepad and Keypad controls over WebRTC data channel
date: 2024-07-21
---
### teleop_twist_joy
There is a config file setup for this, where we invert the left and right stick values to make the robot move in the correct direction. In summary, the values from the browser were opposite from what I wanted with gamepad controls, so this is equivalent to a setting of "invert horizontal and vertical".
<!--more-->
```
sudo nano /opt/ros/humble/share/teleop_twist_joy/config/matthew.config.yaml
```
Then just set the `scale_linear` and `scale_angular` to negative values, and launch using your config file.
#### Gamepad controls
Using function called mapValues, have mapped the button presses from javascript position to ROS2. Have a function in a gamepad controller class which maps the JS values to ROS2.

Values from browser:
```
"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
Values we publish in ROS2:
```
ros2 topic pub /joy sensor_msgs/msg/Joy "{axes: [0.0, -0.1, 0.0, 0.0, 0.0, 0.0], buttons: [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
```

### Gamepad control policy - when do we send commands?
Here's a simple summary of how joy commands are sent:
**When commands are sent:**
1. Every time a button is pressed (any button value > 0)
2. One time when all buttons are released (sends all zeros)
3. When the gamepad disconnects (sends all zeros)
4. When the connection is closed (sends all zeros)

**When commands are NOT sent:**
1. When no buttons are pressed and the last state was also no buttons pressed
2. When only stick/axis values change (unless a button is also pressed)

This ensures responsive control when needed while minimising unnecessary network traffic. So we don't spam empty arrays.

### Button mapping
Mapping in the browser:
gamepad.buttons is an array from javascript.
```
0: A
1: B
2: X
3: Y
4: LB
5: RB
6: LT
7: RT
8: Select (left one)
9: Start (right one)
10: left stick click
11: right stick click
12: UP
13: DOWN
14: LEFT
15: RIGHT
```

The mapping for the ROS2 joy node can still be found here: https://index.ros.org/p/joy/
From ROS2 joy controller
```
0: A
1: B
2: X
3: Y
4: Select (left one)
6: Start (right one)
7: left stick click
8: right stick click
9: left bumper
10: right bumper
11: UP
12: DOWN
13: LEFT
14: RIGHT
```
#### Joysticks 
javascript
```

0: LEFTX (value of 1 is right, value of -1 is left)
1: LEFTY (value of 1 is down, value of -1 is up)
2: RIGHTX (value of 1 is right, value of -1 is left)
3: RIGHTY (value of 1 is down, value of -1 is up)
```

For the ROS2 node they **added trigger values in 4,5**, but the values are floats based on how much the trigger is pushed.
Added the **LT** and **RT** buttons in [this commit](https://github.com/MJPye/robot_with_webrtc/commit/4194405892c1f2e5703025d4820059d25ec8a07e).
#### Keypad Controls

From gamepad, we hold the right bumper:
```
{"axes":[0,0,0,0,0,0],"buttons":[0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0]}
```
Forward
```
{"axes":[0,-0.1,0,0,0,0],"buttons":[0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0]}
```
Backward
```
{"axes":[0,0.1,0,0,0,0],"buttons":[0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0]}
```
Left
```
{"axes":[0,0,-0.2,0,0,0],"buttons":[0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0]}
```
Right
```
{"axes":[0,0,0.2,0,0,0],"buttons":[0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0]}
```

Also need to flip/un-flip the right stick value for turning with the gampad.