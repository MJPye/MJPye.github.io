---
title: Fixing SSL with Vizanti, mostly rambling while trying to fix but ends in a solution
date: 2025-02-08
---
We need to get SSL working again with Vizanti, which will allow the full page to work with HTTPS.
<!--more-->
### Some testing to try and fix - Solution in next section this is just rambling.
First added new launch and kill scripts `pure-vizanti-slam-launch.sh`. These launch SLAM and Vizanti only, no need to video feeds. These launch files are working for SLAM, driving the robot around and checking the vizanti interface on port 5000.

Somebody asking for Vizanti over HTTPS in this [issue](https://github.com/MoffKalast/vizanti/issues/82).

Testing with RWS driving around gives average around 10% on each CPU core.
Goes up to around 20% for me with the rosbridge one, for simplicity lets try this one instead to get SSL back. in the 40% range with nav2, peaking at 60% with an active goal.

| CPU average | Server and SLAM | + NAV2 | + Active goal |
| ----------- | --------------- | ------ | ------------- |
| Rosbridge   | 20%             | 45%    | 60%           |
| RWS         | 10%             | 40%    | 50%           |

So according to ChatGPT we can do it just by editing:
- Nginx config to `nginx_full_https`
- `<iframe src="https://<url>/public/" width="800" height="800" frameborder="0"></iframe>` in `index.html` of signalling server.
- `url: (window.location.protocol === "https:" ? "wss://" : "ws://") + this.url + "/ws/"` in `rosbridge.js` of Vizanti.
- Assuming we also need to edit `rosbridge_script.js` with `wss` instead of `ws`.
Made the vizanti changes on local copy for reference.

Edited these files to change Vizanti:
- `./install/vizanti_server/share/vizanti_server/public/js/modules/rosbridge.js`
- `./install/vizanti_server/share/vizanti_server/public/templates/rosbridge/rosbridge_script.js`

So with the 8443 config and auth turned off, the following works: [websocat](https://github.com/vi/websocat/releases)
```
openvpnas@<ec2-ip>:~$ sudo ./websocat_max.x86_64-unknown-linux-musl wss://<url>:8443/ws/
```
Which suggests the conversion from `ws` to `wss` is ok right?
Defo something to check here, see if you can somehow send messages.
So try changing flask settings so we load https instead of http or something.
The websocket error with socket.io is something different.

add 5000, 5001 and 8443 back to security.
For sure worth setting back up https and trying to connect to rosbridge, would be good to see some data when you reach out using websocat command above.
Remember you have to remove auth from nginx for testing with websocat.

First problem found, you need 8443 here:
```
<iframe src="https://<url>/public/" width="800" height="800" frameborder="0"></iframe>
```
Basically the issues may all be because you use 8443 not 443 which is default. Maybe can change the OpenVPN server to use another port instead.

## Changes required for HTTPS - Actual working solution on 8443, switched to default 443 later.
`rosbridge.js`
```
this.ros = new ROSLIB.Ros({
    url: 'wss://' + this.url + ':8443' + '/ws/'
});
```
`rosbridge_script.js` - Not sure if this is needed
```
url.innerText = "Bridge URL: wss://"+rosbridge.url + ":"+rosbridge.port;
```
`index.html`
```
<iframe src="https://<url>:8443/public/" width="800" height="800" frameborder="0"></iframe>
```
Add the nginx config called `nginx_working_https`.
`nginx` and `index.js` require the same port number.
`main.js` requires no turn server until fixed, currently breaks cross origin rules.

You only require port 8443 btw for the security on AWS, not 5000 and 5001.
Commit to Fix [vizanti for HTTPS and wss](https://github.com/MJPye/vizanti/commit/06528c3e945ce0d00a174cfe33f0e68b7781338c).
Commit to Fix [signalling server and nginx for https and wss](https://github.com/MJPye/robot_with_webrtc/commit/318f8213540e8b3b75f1762c2f17578ffcd39ae7).