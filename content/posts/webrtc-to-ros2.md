---
title: WebRTC message conversion to ROS2
date: 2025-08-05
---
### What does this node do?
Sometimes we want to send ROS2 actions or service calls over the WebRTC data channel from the UI and have them execute on the robot. This node handles that functionality using [rclnodejs](https://github.com/RobotWebTools/rclnodejs), the ROS 2 client library for Javascript.
There isn't much to learn on this page so far, we are just receiving messages on the WebRTC data channel and converting them to ROS 2 actions.
<!--more-->

Had a problem with an outdated version of node:
```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.4/install.sh | bash
source ~/.nvm/nvm.sh
nvm install node
nvm use node
node -v
npm -v
```

Made a new branch for splitting Data and Video, working branch is called `mp-split-server-client`.

```bash
scp -r -i ~/.ssh/create_laptop ~/Documents/WebRTC_learning/webrtc-web/signalling-server rpi@<rpi_ip>:/home/rpi/webcam_testing

scp -r -i ~/.ssh/create_laptop ~/Documents/WebRTC_learning/webrtc-web/webrtc-to-ros2 rpi@<rpi_ip>:/home/rpi
```

Made some progress on splitting video and data channel apart, tested and it still seems to run with video ok. Now create 2 peer connections, need to add the message topic to the signalling server and main.js then add the DataChannel code to the new server in webrtc-to-ros2.

startVideoButton now makes the video connection first. Need to create the createDataConnection function.

main.js has some code to create the data peer connection. Now need to add to signalling server index.js, and the webrtc-to-ros2 node to handle the offer and respond.

Managed to get video working first, then data after. Meaning there is likely some issue with the isStarted variable. Also get ICE problem if creating video first then data, because when we ask for video we send candidates before the data channel has received a remote description. So need to make sure candidates are not sent over the data channel socket topic until the offer has been sent.

Current state is Video then data working if steps in correct order are done:
- Start all screens.
- Load webpage first.
- Then do "node index.js" in webrtc-to-ros2.
- Start Video.
- Start Data.
Next step is to get it working in any order, and decide which should be default.
Ok so now works in any order, but want the webrtc-to-ros2 node to be able to handle disconnects. Fixing now.

Ok after latest changes we seem quite resilient:
- Can leave the webrtc-to-ros2 screen running and connect/disconnect with the browser as needed.
Next step is get the gamepad running. Timer has to run on the server side, because if the client disconnects we need the robot to stop moving. So on server, if you don't receive a movement message, you need to stop movement.

Got simple commands running using https://github.com/ros2/teleop_twist_joy. This is setup in a screen. When launched, if you publish a `Joy` message, it gets converted into a `cmd_vel`. You need to set button 2 to true, I just set the first 3 to true and it worked. Therefore, I think the solution is to send the JSON through the WebRTC, then publish to Joy from the webrtc-to-ros2 node. Some example commands in the Gamepad controls page.
Next Step:
- Send the button and stick arrays as a JSON over the data channel.
- Take the JSON and use it as the message to `/Joy`.

Got the gamepad working:
- Try adding a negative scale to reverse direction of linear.
- Make sure we are setting good safety limits.
- Do some CSS styling - Done some

TODO 06-09-24
- Safety limits on driving.
- OpenCV on the RTSP stream
- Start with 2D navigation, lets order an rplidar C1 from amazon. - Ordered.


### 2025-01-15 - Adding Docking and Undocking to the UI
Cleaned up the UI and in the process added Dock and Undock buttons. This required changes to:
- modified:   signalling-server/index.html
- modified:   signalling-server/js/main.js
- modified:   webrtc-to-ros2/index.js
Found in the following [commit](https://github.com/MJPye/robot_with_webrtc/commit/99ebe3891b50af39bb48606c643c1ce2754b72cb).
Uses action client in the `webrtc-to-ros2` node to call actions when requested over WebRTC.
So far untested, have backed up the files listed above into a folder called `tmp_backups` in case they need to be added back to the rpi (those ones were working today).