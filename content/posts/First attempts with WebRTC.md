---
title: First attempts with WebRTC, data channels working
date: 2024-06-22
---
#### Basics
WebRTC uses RTCPeerConnection to communicate streaming data between browsers, but also needs a mechanism to coordinate communication and to send control messages, a process known as signalling.
The WebRTC APIs use STUN servers to get the IP address of your computer, and TURN servers to function as relay servers in case peer-to-peer communication fails.
To learn how this works, following the [codelab here](https://codelabs.developers.google.com/codelabs/webrtc-web#0), which is where references to `step-05` come from.

<!--more-->
Installed Node Package manager on macbook
```
# installs nvm (Node Version Manager)
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | 

# then open a new terminal

# download and install Node.js (you may need to restart the terminal)
nvm install 20

# verifies the right Node.js version is in the environment
node -v # should print `v20.15.0`

# verifies the right NPM version is in the environment
npm -v # should print `10.7.0`
```

Next step will be to edit step-05 folder so that instead of a video stream, we are able to send data between rpi and laptop using the data channel. Laptop acts as the signalling server when `node index.js` is run.
Once you have some data communication, move onto images, then videos/streams.

chrome://webrtc-internals/

Have the data channel working between chrome/firefox or between 2 computers connected to the same network. Only need to run `node index.js` from the host and then access with IP:8010. Turn off browser caching with Developer tools > Network.
When home internet is back, check if this data channel works between laptop and RPI over VPN. Working version is in step-05 folder.

Can now open/close connection successfully, but not sure about the cleanup function. Next figure out how to get addStream working.

Now get no data channel comms and error `WebRTC: ICE failed, add a STUN server and see about:webrtc for more details`. - Was issue running on local firefox and chrome.

Not much progress made on the streams here. It is vital you learn how to check if the stream is actually being created first, learn how to read the WebRTC logs. https://getstream.io/blog/debugging-webrtc-calls/

```
pc.getLocalStreams()
1. [MediaStream]

1. 0: MediaStream {id: '71544b4a-3314-490e-971e-006090499d40', active: true, onaddtrack: null, onremovetrack: null, onactive: null, …}
2. length: 1
3. [[Prototype]]: Array(0)
```

Problem was that I was doing the call before the stream was added to the tracks. Now issue is that "Add Remote Stream" needs to be called before "Add Local Stream". 
Todo: Move adding of remote stream somewhere so that as soon as a local stream is available, it appears. Or do something nicer, maybe checkboxes?

Snags:
- Restarting the WebRTC connection fails sometimes after disconnect, refreshing sometimes works but not frequently.
Problem where can't start stream from remote is because http is insecure. On the remote browser do:
- chrome://flags/#unsafely-treat-insecure-origin-as-secure
- Enable it
- Add `http://192.168.0.21:8010`
- Relaunch chrome. It should work now.

	In `createPeerConnection` have added `addRemoteStreamChannel();` to test if that works on launch.


#### Fixing webcam issue on rpi
Webcam not detected by cheese or chromium, unless run like so:
```bash
xhost +si:localuser:root
sudo cheese
sudo chromium
# To remove this xhost change
xhost -si:localuser:root
```
#### Permanently FIXED
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

#### Testing 2024-07-20 - WebRTC in client and RPi browsers over VPN
Tested the WebRTC over the VPN, everything works quite well. The slow point now is having to login to the desktop of the robot, the stream looked ok.
Next steps:
- Find a way to launch what is needed on the robot side, without needing chromium open. Requires creation of a separate server/client files.
- Add joystick controls to the client.