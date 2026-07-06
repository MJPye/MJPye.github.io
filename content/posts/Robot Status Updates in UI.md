---
title: Bi-directional WebRTC data for Robot Status Updates
date: 2025-06-14
tags: ["MobileRobot"]
---
I want to get robot status updates in the UI. First step is 2 way communication. Made a test using this [commit](https://github.com/MJPye/robot_with_webrtc/commit/31851b41609b6cc376fd9242b1da31ae42fad30f).
### Side note - creating a wireframe
I took the Downloadable 3D STEP model from the [Create3 Docs](https://iroboteducation.github.io/create3_docs/hw/mechanical/).
With it loaded in Fusion 360, I was able to export an `obj` file.
The `obj` file can be loaded using ThreeJS as shown in this [example](https://threejs.org/examples/?q=obj#webgl_loader_obj) and it's [source code](https://github.com/mrdoob/three.js/blob/master/examples/webgl_loader_obj.html).
<!--more-->

With the `obj` file loaded, I created the wireframe by drawing the model first in black, then over the top with green lines. This prevents being able to see lines that should be hidden. 

All of the required files are stored at `/Users/matthewpye/Documents/create3_wireframe_js` and can be run with `python3 -m http.server`.

From then on it was just changing some positions, animation rotations and colours to end up with a nice widget that can go somewhere on the UI. Have not tested yet to see the impact on performance.

<video src="/images/wire_mesh_threejs.mov" autoplay muted loop playsinline style="max-width:100%; height:auto;"></video>

### Adding the model to an S3 bucket
Created a new bucket in AWS console and uploaded the 3D model of the full assembly.
In order to allow public downloads, go to the bucket's "Permissions" tab and turn off the "Block all public access" switch. Then add a Bucket Policy like the one from [here](https://docs.aws.amazon.com/AmazonS3/latest/userguide/WebsiteAccessPermissionsReqd.html#bucket-policy-static-site):
```
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Sid": "PublicReadGetObject",
            "Effect": "Allow",
            "Principal": "*",
            "Action": [
                "s3:GetObject"
            ],
            "Resource": [
                "arn:aws:s3:::Bucket-Name/*"
            ]
        }
    ]
}
```
With the switch flipped and this bucket policy added, the URL is reachable. This is not ideal though as there is nothing preventing repeated downloads, so I need to manage access to this.
`aws:Referer` is one option but can be spoofed. CloudFront seems to be best practice and could be good to learn.
### Loading in UI
Got the Robot model into the UI.
<video src="/images/UI_with_robot_viewer.mov" autoplay muted loop playsinline style="max-width:100%; height:auto;"></video>

Tricky parts were the `threejs` imports, needed to use an import map. Also the reason it didn't show at first is that the containers height and width were set to 0.
For now have blocked public access to the bucket, re-enable to see the wireframe.

Fix for now with imports:
```
<head>
	<title>Robot in the flat</title>
	<link rel="stylesheet" href="css/main.css" />
	<script type="importmap">
	{
		"imports": {
		"three": "https://cdn.jsdelivr.net/npm/three@0.155.0/build/three.module.js",
		"three/addons/": "https://cdn.jsdelivr.net/npm/three@0.155.0/examples/jsm/"
		}
	}
	</script>
</head>
```

Commit [here](https://github.com/MJPye/robot_with_webrtc/commit/015a10de732202d7bd1622c3e5951b52f083bf7a) where I first added the robot viewer to the UI using the `WireframeOBJViewer.js` class and second commit [here](https://github.com/MJPye/robot_with_webrtc/commit/74a51be625c545946e40ee63233580fa3eaf5566) for visual improvements.

### S3 policy update for access by the UI
Created a backup of the AMI called `mp_vpn_server_backup` in case the server shuts down for some reason.

Updated the Bucket Policy from above to limit requests to only certain urls. This is not foolproof as it is easy to do something like 
```
curl -H "Referer: https://<some_address>" https://<bucket_url>
```
But this data isn't anything special. Could use CloudFront if it needed to be more secure.
```
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Sid": "PublicReadGetObjectWithRefererCheck",
            "Effect": "Allow",
            "Principal": "*",
            "Action": "s3:GetObject",
            "Resource": "arn:aws:s3:::<bucket_name>/*",
            "Condition": {
                "StringLike": {
                    "aws:Referer": "https://<referrer_address>/*"
                }
            }
        }
    ]
}
```

### Issue with rclnodejs subscribing to create3 topics
When I tried to subscribe to some create3 topics from `rclnodejs`, the data was not being received. Could see the following:
- `rclnodejs` debug logs showed we were subscribed to the topic.
- Data published from `cli` appears in `rclnodejs`.
- Data showing up in `cli` like normal.
Issue became clear when I did:
```
ros2 run topic_tools relay /battery_state /battery_state_relayed

[WARN] [1750973330.540706563] [relay]: New subscription discovered on topic '/battery_state_relayed', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```
So using a relay is a good tool to get the errors that you can't see from the robot. It was all to do with the QoS policy. Turns out, in `rclnodejs` whenever I tried to create a QoS object, it just wasn't being used. This difference below shows it:
```
rpi@rpi-desktop:~/systemd-services/testing-sleep-wake$ ros2 topic info /imu --verbose
Type: sensor_msgs/msg/Imu

Publisher count: 1

Node name: robot_state
Node namespace: /
Topic type: sensor_msgs/msg/Imu
Endpoint type: PUBLISHER
GID: 01.10.63.38.98.98.b0.9b.de.98.16.32.00.01.1f.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): KEEP_LAST (1)
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 1

Node name: webrtc_to_ros2_node
Node namespace: /
Topic type: sensor_msgs/msg/Imu
Endpoint type: SUBSCRIPTION
GID: 01.10.7b.54.4e.73.a4.5e.7a.2d.9f.1f.00.00.14.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (10)
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite
```

And **the solution**, if you need a specific QoS, don't create the object yourself but use one of the predefined ones:
```
{ qos: rclnodejs.QoS.profileSensorData }
```

As a result of this, now the `battery_state` is read from the `create3`, `rclnodejs` subscribes to it and publishes it on the WebRTC data channel. Voltage is now printed in the console of the browser. This means we are ready to display robot feedback in the UI more.

#### Splitting the wireframe into colours
The Wireframe array has length 35, with components corresponding to the following indexes:
- wireframe[33:34] = reolink sphere and cylinder base
- wireframe[28:32] = reolink holder and blocks
- wireframe[12:27] = LiDAR and LiDAR holder
- wireframe[00:11] = Create3 robot

Robot statuses now appear in the UI, as of this [commit to main](https://github.com/MJPye/robot_with_webrtc/commit/aa07dcadd8a30ba2db811e2aea57f267f468c2eb).

### Feedback on the buttons
Have added feedback to the `Start Data` and `Start Video` buttons. 
The first will glow green when the application loads and a data channel is not already open. When clicked, the button will stop glowing.

The `Start Video` button glows when the data channel is open. I notice that clicking this immediately does nothing, seems like we need to wait for all the SLAM parts to start then we can `Start Video`. A better solution for the glowing button here might be to wait for the LiDAR status to be online.

We also added text which says:
- `Robot Visualisation Loading...` when the robot wireframe is loading.
- `Camera Offline` when the camera stream is not available.

All of these changes are in [this commit](https://github.com/MJPye/robot_with_webrtc/commit/7a1131fca507129b2b261b6caa0a5d2f32db1c6f).