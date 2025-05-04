---
title: Reolink camera streaming
date: 2025-05-03
---
### Setting up ETH0 interface between camera and RPi
With the Reolink camera connected to my MacBook or over Wi-Fi, I found the streams had some latency. Going to try changing settings to see if we can improve this.

First a connection profile for the `eth0` interface, which hands out the IP address `192.168.50.2`:
```
[connection]
id=eth0
type=ethernet
interface-name=eth0
autoconnect=true
autoconnect-retries=0

[ipv4]
method=shared
addresses=192.168.50.1/30
never-default=true

[ipv6]
method=ignore
```

### Converting the RTSP stream to WebRTC
To test with my UI, going to change the following file: `/home/rpi/webcam_testing/RTSPtoWeb/config.json` and include the link to the reolink RTSP stream instead of the WebCam one.

```
"streams": {
    "f29c576b-bbfb-449a-be5a-180fd0a7bedc": {
      "channels": {
        "0": {
          "url": "rtsp://admin:<password>@192.168.50.2:554"
        }
      },
      "name": "robot_feed_rpi"
    }
  }
```

Discovered the option in the settings on the app to 
change the resolution and frame rate of the camera. If we change the RTSP stream address, we can either take the `clear` stream or the `fluent` one. `clear` is the main stream, `fluent` is the sub stream.
Fluent
```
"url": "rtsp://admin:<password>@192.168.50.2:554/Preview_01_sub"
```
Clear
```
"url": "rtsp://admin:<password>@192.168.50.2:554/Preview_01_main"
```
Using both so far we get a stream that performs well with low latency, might be a good option to remove the old webcam altogether. 

```
http://192.168.50.2/cgi-bin/api.cgi?cmd=Snap&channel=0&rs=wuuPhkmUCeI9WG7C&user=admin&password=password
```

### Taking snapshots
We can also take snapshots from the command line as follows, they are of the higher resolution by default, but that is what we want:
```
curl -o snapshot.jpg "http://192.168.50.2/cgi-bin/api.cgi?cmd=Snap&channel=0&rs=wuuPhkmUCeI9WG7C&user=admin&password=password"
```