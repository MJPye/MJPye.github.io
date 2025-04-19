---
title: Streaming - from WebCam and Video for Linux to WebRTC
date: 2024-07-29
---
On this page I include the details on how to use `gst-launch` or `python` scripts to create `rtsp` streams. We then convert the `rtsp` streams to `WebRTC` using Go and pass them on to the client.
Note Kurento was not used as it cannot run directly on the Raspberry Pi. In the future, with some intermediate server, we may return to Kurento, as it will handle multiple clients far easier.
<!--more-->
## Webcam stream setup
Testing the webcam with `ffmpeg`.
```
sudo apt install ffmpeg
ffmpeg -f v4l2 -video_size 1280x720 -i /dev/video0 -frames 1 out.jpg
```

### May need to add rpi user to video group if not already there
Issue was camera permissions, fix with video group and udev rule.
```bash
# Check rpi user is in the video group, if not then add it.
groups
# If video group doesn't show then:
sudo usermod -a -G video rpi
sudo nano /etc/udev/rules.d/99-video.rules
# adding
KERNEL=="video[0-9]*", GROUP="video", MODE="0660"
#Then reboot
```
### V4L controls
So info in a [guide here on V4L](https://medium.com/@deepeshdeepakdd2/v4l-a-complete-practical-tutorial-c520f097b590).

```
sudo apt install v4l-utils
v4l2-ctl -d /dev/video0 --list-ctrls
v4l2-ctl -d /dev/video0 --list-formats-ext
```
Cannot find a plug and play solution for this, will have to figure it out. Or can ask Jonny.
RPI3 has the HDMI output, try setting up the stream first with a GUI, then work on getting the stream shared over HTTP to another device in local network.

On Lenovo laptop with webcam
```
gst-launch-1.0 v4l2src device=/dev/video2 ! image/jpeg ! jpegdec ! xvimagesink
```
Now get it to work properly over the network.

Launch on RPI - don't use
```
gst-launch-1.0 -vv -e v4l2src device=/dev/video0  ! \
"image/jpeg,width=640,height=480" ! queue ! jpegdec ! videoconvert ! \
x264enc ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=127.0.0.1 port=5000
```

View on RPI or Lenovo (after changing IP from fast stream to Lenovo laptop)
```
gst-launch-1.0 -v udpsrc port=5000 caps="application/x-rtp, media=video, encoding-name=H264, payload=96" ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink
```

Now we need a fast stream
```
gst-launch-1.0 -v v4l2src device=/dev/video0 ! \
video/x-raw,width=640,height=480,framerate=30/1 ! \
videoconvert ! queue ! x264enc tune=zerolatency bitrate=512 speed-preset=superfast ! \
rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.0.188 port=5000
```

Alternatively, run `python3 /home/rpi/webcam_testing/rtsp-server.py` and access on VLC with `rtsp://192.168.0.188:8554/test`, or view with:
```
gst-launch-1.0 -v rtspsrc location=rtsp://192.168.0.188:8554/test ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink
```

Now we have the RTSP stream, we can use Kurento's PlayerEndpoint to retrieve the content, then send it out with a WebRTC endpoint.
Alternatively check these: https://github.com/deepch/RTSPtoWebRTC and https://github.com/deepch/RTSPtoWeb, might help skip Kurento.

Kurento can't run on the Pi itself.
I think the issue with streaming is the codec, try to follow tips here to get this one working: https://www.willusher.io/general/2020/11/15/hw-accel-encoding-rpi4/

Best results so far were with:
`python3 test-low-bw-rtsp-server.py`
`GO111MODULE=on go run *.go`
Then open rtsp-to-webrtc index.html

New goal is to get an rtsp stream created using `h264_v4l2m2m`
### Something working
Ok so the following works to get an RTSP stream from the camera, run on the pi.
```
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GObject

Gst.init(None)

class MyFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        GstRtspServer.RTSPMediaFactory.__init__(self)

    def do_create_element(self, url):
        pipeline_str = (
          "v4l2src device=/dev/video0 ! "
          "video/x-raw,width=640,height=480,framerate=30/1 ! "
          "videoconvert ! queue max-size-buffers=1 ! video/x-raw,format=I420 ! x264enc speed-preset=ultrafast tune=zerolatency threads=1 key-int-max=15 ! "
          "video/x-h264,profile=constrained-baseline ! "
          "queue max-size-time=100000000 ! h264parse ! "
          "rtph264pay config-interval=1 name=pay0 pt=96"
	)
        return Gst.parse_launch(pipeline_str)

class GstServer:
    def __init__(self):
        self.server = GstRtspServer.RTSPServer()
        self.server.props.service = "8554"
        self.factory = MyFactory()
        self.factory.set_shared(True)
        self.mount_points = self.server.get_mount_points()
        self.mount_points.add_factory("/test", self.factory)
        self.server.attach(None)
        print("RTSP stream ready at rtsp://<RaspberryPi_IP>:8554/test")

if __name__ == '__main__':
    GObject.threads_init()
    GstServer()
    loop = GObject.MainLoop()
    loop.run()
```

Then also on the pi (or laptop for testing), run:
```
RTSPtoWeb$ GO111MODULE=on go run *.go
```
You should get a fairly responsive WebRTC stream that doesn't cut out much.
Got the answer from here: https://github.com/centricular/gstwebrtc-demos/blob/e4b86bc4f151e35222aff1bf7e46cec016e7b0ee/sendonly/webrtc-unidirectional-h264.c#L181C1-L183C66

![Image Description](/images/WebRTC_session_description.png)

Have setup screens to do the following:
- signalling-server: go to `~/webcam_testing/signalling-server` and run `node index.js`.
- rtsp-to-webrtc: go to `~/webcam_testing/RTSPtoWeb` and run `GO111MODULE=on go run *.go`.
- rtsp-server: go to `~/webcam_testing` and run `python3 test-low-bw-rtsp-server.py`.
With these 3 screens running, you can go to the local or VPN IP address of the RPi, currently at port 8040, then click start. This should open data channel (no way to respond yet) and start the stream :) Tested with VPN on and there is latency but it was fair.

Next steps:
- Think about the data channel again. How are we going to send data messages from the client, receive them on another peer, then convert to ROS messages for example, and vice versa for ThreeJS. Another NodeJS server that connects to the client DataChannel and uses some ROS2-NodeJS library to forward messages to ROS2 topics?
- https://github.com/RobotWebTools/rclnodejs
- https://github.com/node-webrtc/node-webrtc-examples/tree/master
- https://github.com/node-webrtc/node-webrtc
- So next step is to make the same client be able to send a string over the DataChannel, then a NodeJS server is able to receive these messages and convert to ROS2 topic data.

Could come back to Kurento in the future: https://doc-kurento.readthedocs.io/en/latest/user/installation.html
Installed in a docker container, preferably linux but can also try docker on mac.
The end goal would be this tutorial.

## RTSPtoWeb - Converting RTSP stream to WebRTC
`/home/rpi/webcam_testing/RTSPtoWeb/config.json` contains the RTSP link and the name of the stream (UUID) that will be created in WebRTC.
This value is then found in `signalling-server/index.js`. So when the Signalling server receives a `video-request` with just the `sdp`, it knows to do the `post` request to the URL including the UUID below. The response is the `sdp` from `RTSPtoWeb` which is returned to the client.

[RTSPtoWeb Repo](https://github.com/deepch/RTSPtoWeb).
All we had to edit was `config.json` with the following:
```
"streams": {
    "f29c576b-bbfb-449a-be5a-180fd0a7bedc": {
      "channels": {
        "0": {
          "url": "rstp://192.168.0.188:8554/test"
        }
      },
      "name": "robot_feed_rpi"
    }
  }
```


