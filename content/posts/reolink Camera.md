---
title: reolink PTZ camera
date: 2025-03-30
---
Grabbed a reolink E1 zoom camera from the Amazon spring sale. Based on what works or doesn't work, and the performance of the stuff that works, I had 2 ideas:
<!--more-->
- Install the camera on the robot, and make use of the Pan-Tilt-Zoom (PTZ) features to get a more capable robot. The Camera creates an RTSP stream out of the box which I can pass into my existing RTSP->WebRTC pipeline.
- Install the robot in the Living Room and still retrieve the RTSP stream and pass to WebRTC. Would give an overview of the robot in the room. Main reasons to do this instead would be.
	- Can't get the responsiveness (low stream lag) needed for driving, the current webcam stream is very responsive.
	- 5V/2A power requirement is too much and drains current robot battery.

![[reolink_e1_zoom.png]]

Setup the reolink camera using app on smartphone, with Credentials set for the admin account on the device. Instructions [here](https://support.reolink.com/hc/en-us/articles/360007010473-How-to-Live-View-Reolink-Cameras-via-VLC-Media-Player/) for the RTSP stream and VLC. Could instantly get a stream on VLC on laptop like so:
```
rtsp://admin:<password>@<camera_ip>/Preview_01_main
```

Now we know the camera works and can generate the RTSP streams, I want to look into the [Python API client](https://github.com/ReolinkCameraAPI/reolinkapipy) which would mainly be required for the PTZ functionality.

To see the page where you can enable HTTP, RTSP etc, as shows [here](https://support.reolink.com/hc/en-us/articles/360003452893-How-to-Access-Reolink-Cameras-NVRs-Home-Hub-Locally-via-Web-Browsers/):
```
Load stream on Phone > Settings > E1Zoom > Network Information > Advanced
```

