---
title: reolink PTZ camera
date: 2025-03-30
---
Grabbed a reolink E1 zoom camera from the Amazon spring sale. Based on what works or doesn't work, and the performance of the stuff that works, I had 2 ideas:

- Install the camera on the robot, and make use of the Pan-Tilt-Zoom (PTZ) features to get a more capable robot. The Camera creates an RTSP stream out of the box which I can pass into my existing RTSP->WebRTC pipeline.
- Install the robot in the Living Room and still retrieve the RTSP stream and pass to WebRTC. Would give an overview of the robot in the room. Main reasons to do this instead would be.
	- Can't get the responsiveness (low stream lag) needed for driving, the current webcam stream is very responsive.
	- 5V/2A power requirement is too much and drains current robot battery.

<!--more-->
![Image Description](/images/reolink_e1_zoom.png)

Setup the reolink camera using app on smartphone, with Credentials set for the admin account on the device. Instructions [here](https://support.reolink.com/hc/en-us/articles/360007010473-How-to-Live-View-Reolink-Cameras-via-VLC-Media-Player/) for the RTSP stream and VLC. Could instantly get a stream on VLC on laptop like so:
```
rtsp://admin:<password>@<camera_ip>/Preview_01_main
```

Now we know the camera works and can generate the RTSP streams, I want to look into the [Python API client](https://github.com/ReolinkCameraAPI/reolinkapipy) which would mainly be required for the PTZ functionality.

To see the page where you can enable HTTP, RTSP etc, as shown [here](https://support.reolink.com/hc/en-us/articles/360003452893-How-to-Access-Reolink-Cameras-NVRs-Home-Hub-Locally-via-Web-Browsers/):
```
Load stream on Phone > Settings > E1Zoom > Network Information > Advanced
```
HTTP or HTTPS are needed when using the Python API.
### Ethernet instead of Wi-Fi
There is an Ethernet port on the camera, which is more suitable than Wi-Fi for use with the Robot. After powering on the camera and connecting the Ethernet cable to my MacBook via USB-C to Ethernet adapter, I did not see any devices showing in the table with `arp -a`.
Therefore I assumed the camera is waiting for an address to be given to it by a **DHCP server**.

To run a DHCP server on a MacBook, go to:
```
Settings > Sharing > Internet Sharing
```
Select the correct interface to share to (in my case the USB-C to Ethernet adapter) and turn Internet Sharing **ON**.

Now the `arp` table shows an address for the MacBook itself and the Reolink camera, which is second on this list.
```
? (192.168.2.1) at ce:8:fa:b:7b:64 on bridge100 ifscope permanent [bridge]
? (192.168.2.2) at ec:71:db:87:c7:f1 on bridge100 ifscope [bridge]
```
So now we have the camera connected over Ethernet, and this setup can be replicated on a Raspberry Pi or Jetson when the camera is mounted on the robot.

With the camera connected, as before but this time using the IP of the Ethernet interface, we can load the RTSP stream:
```
rtsp://admin:<password>@192.168.2.2:554
```

### Controlling the Pan-Tilt-Zoom (PTZ) feature with Python
The Webcam stream I already have is very responsive which is good for driving, but suffers a couple drawbacks:
- It is mounted with a fixed orientation facing the front of the robot.
- The Auto-Exposure is still not working great (likely because I didn't configure it well), meaning performance in low light conditions is poor.

I plan to keep the webcam stream as the responsive stream for driving, and have the Reolink E1 Zoom PTZ camera as more of an inspection camera, taking higher quality images from all angles. To do this, we need to be able to control camera movement, zoom and switch between day and night streams.

I tested the [Python API client](https://github.com/ReolinkCameraAPI/reolinkapipy) and the `stream_gui.py` example works great.
#### Installing reolinkapi
```
cd ~/reolink_api_testing
python3 -m venv venv && source venv/bin/activate
pip install 'reolinkapi[streaming]'
pip install PyQt6
```
Then for the example I tested clone the repo:
```
git clone git@github.com:ReolinkCameraAPI/reolinkapipy.git
cd reolinkapipy/examples
```
Inside the examples there is one called `stream_gui.py` which displays the RTSP streams but also lets you use the arrow keys and mouse scroll to move the camera and zoom.
First create a config file in the examples folder called `camera.cfg` with the following content:
```
[camera]
ip = <reolink_IP>
username = <username, default is admin>
password = <password>
```
The username and password will have been created when first turning the camera on and connecting with the smartphone application.

Now running `python3 stream_gui.py` will show the RTSP streams in a QT widget, and pressing arrow keys will move the camera. Scrolling the mouse wheel changes the zoom level.

#### Summary and next steps
Next steps are to:
- Order another camera power adapter from Amazon which I can modify to connect to the 5V DC converter board I will connect to the Robot's 12V output. Then we can mount the camera to the robot and see how well it is powered, and the impact on battery life.
- Add another panel to the UI for viewing the streams of this camera, as well as creating the WebRTC stream.
- Add UI controls for PTZ functionality.