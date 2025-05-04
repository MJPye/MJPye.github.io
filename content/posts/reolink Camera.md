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


### Testing Reolink camera powered by Robot battery
Seems I missed the part of the Create 3 docs where is says the 5V 3A power for the raspberry pi shares the same Battery Voltage x 2A power as the unregulated port. 

I attached a Step Down voltage converter in between the Robot's unregulated power output and the Reolink camera. The Voltage converter provides the needed 5V input. The Reolink manual and supplied power adapter suggest 2A, so far when moving the camera I see a current consumption of around 1A, so 5V 1A.

On the input side of the Voltage converter, I need to monitor the power consumption. At 16V input I see current consumption up to 0.8 A.

Power P = VI,
Input: 16 x 0.8 = 12.8 W
Output: 5 x 1 = 5 W
5 / 12.8 x 100 = 39% efficiency.

Fair to assume as battery voltage drops, input current will increase.

Going to get a USB-C splitter for this, and first see if the reolink camera and RPi with all components can just be run from the 5V 3A supply.

Camera max: 5V 1A so far.

### Testing with every component connected
With:
- Reolink camera running through voltage regulator.
- LiDAR running
- RPi with all screens running
- Webcam running
Everything worked and the output from the battery is like below.

```
---
header:
  stamp:
    sec: 1745875763
    nanosec: 719883127
  frame_id: ''
voltage: 14.409000396728516
temperature: 31.44999885559082
current: -0.9549999833106995
charge: 0.36000001430511475
capacity: 1.2510000467300415
design_capacity: 1.2510000467300415
percentage: 0.28999999165534973
power_supply_status: 0
power_supply_health: 0
power_supply_technology: 0
present: true
cell_voltage: []
cell_temperature: []
location: ''
serial_number: ''
```


0.25 A at 14.4V was the difference when I turned off the Reolink.

0.53A at 14.3V when the Rpi is on but nothing running.

Is it possible I was reading input current on the LCD display? Can try to long press the button, because 14.4V and 0.25A don't match what I saw. According to [msg](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/BatteryState.html) definition it should be in Amps.
- Plug the DC-DC converted back in with the camera and check if pressing the button changes the current value. - Do this next!

### Options:
- Reolink is in the room, connected to Wi-Fi, controlled ON/OFF by a smart plug.
- Reolink is attached to the robot using step down converter, can get a relay to power it ON/OFF and control with RPI GPIO. <- Relay consumes too much current from RPi.
	- Do we use the WebCam still or not? Need to see if Reolink can give a responsive stream.
Looks like we can get rid of the webcam so far.

Next hardware step to test:
- Plug the voltage regulator into 5V input power.
- Measure voltage across button.
- Do the same with robot battery voltage.
- Can we replicate that with RPi GPIOs?