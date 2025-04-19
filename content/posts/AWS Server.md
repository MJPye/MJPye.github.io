---
title: AWS Server setup
date: 2024-09-22
---
#### OpenVPN Access Server
I have created a launch template from my running AWS OpenVPN Access Server.
The existing server is based on the following ami: https://eu-central-1.console.aws.amazon.com/ec2/home?region=eu-central-1#ImageDetails:imageId=ami-039470c0765f439c4

We run this on a t2.micro instance. The OpenVPN server setup was done using the GUI provided when logging into the server. I must admit, I have forgotten how I did this, so will try to access it again and do a dump of the settings.
<!--more-->

Some backups:
I have created a folder on the mac called AWS_Setup which contains:
- A backup of the OpenVPN Access server.
	- Made with `sudo tar -czvf openvpn-as-backup.tar.gz /usr/local/openvpn_as/`
- The connection profiles.
	- Laptop: `profile-36xxxxx26.ovpn`
	- RPi: `profile-50xxxxx35.ovpn`
I am not certain these backups will work, but it is enough for me now to try setting up the reverse proxy.

#### Nginx Reverse Proxy Setup
I first stopped the OpenVPN server to allow certbot to use port 443, which was previously used by OpenVPN:
```
sudo service openvpnas stop
```

```
sudo apt update
sudo apt install nginx
sudo certbot --nginx -d <url>
```

Response when getting certificate
Failed the first time but worked when I opened port 80.
```
Requesting a certificate for <url>

Successfully received certificate.
Certificate is saved at: /etc/letsencrypt/live/<url>/fullchain.pem
Key is saved at:         /etc/letsencrypt/live/<url>/privkey.pem
This certificate expires on 2025-01-13.
These files will be updated when the certificate renews.
Certbot has set up a scheduled task to automatically renew this certificate in the background.

Deploying certificate
Successfully deployed certificate for <url> to /etc/nginx/sites-enabled/default
Congratulations! You have successfully enabled HTTPS on https://<url>
```

So now we have certificates, lets create a username and password.
```
sudo apt-get install apache2-utils
sudo htpasswd -c /etc/nginx/.htpasswd <username>
```
Which creates user `<username>` with usual password.
If you want to create more users in the future, do so like this:
```
sudo htpasswd /etc/nginx/.htpasswd another_username
```

I then added the following to replace the existing default nginx config, tested with `sudo nginx -t`, and `sudo systemctl restart nginx`:
```
server {
    listen 8443 ssl;
    server_name <url>;

    ssl_certificate /etc/letsencrypt/live/<url>/fullchain.pem;
    ssl_certificate_key /etc/letsencrypt/live/<url>/privkey.pem;

    # Enable Basic Authentication
    auth_basic "Restricted Content";
    auth_basic_user_file /etc/nginx/.htpasswd;

    location / {
        proxy_pass http://<rpi-vpn-ip>:8040;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }
}
```
Now restart the OpenVPN services:
```
sudo service openvpnas start
```

### Hostname
Created the following hostname using No-IP:
```
<url>:8443
```
With the HTTPS setup from earlier, we should access on port 8443, which should prompt for user and password.

For this to work, had to add a rule to security group again:
- Custom TCP
- 8443
- 0.0.0.0/0
Am going to remove this sometimes for security. Currently removed.

### Next Steps
- Add Fail2ban to check for brute force connection attempts.
- Can we add some sort of slider for camera exposure? When the lights are off we just see a blank screen.
- Have setup a 15 dollar monthly budget for AWS, watch this in the next 2 months to see how much the server costs after free tier expires.

Changed nginx conf to try and see Vizanti, change back to one above if broken.
```
server {
    listen 8443 ssl;
    server_name <url>;

    ssl_certificate /etc/letsencrypt/live/<url>/fullchain.pem;
    ssl_certificate_key /etc/letsencrypt/live/<url>/privkey.pem;

    # Enable Basic Authentication
    auth_basic "Restricted Content";
    auth_basic_user_file /etc/nginx/.htpasswd;

    location / {
        proxy_pass http://<rpi-vpn-ip>:8040;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }

    location /iframe-content/ {
        proxy_pass http://<rpi-vpn-ip>:5000;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }
}

```

Changed the nginx and the signalling server html. Works fine with the iframe so far locally and with VPN. Not working over the webname.

### Solution with HTTP not HTTPS
I can see the dashboard when using HTTP using this nginx config and visiting port 80:
```
server {
    listen 80;
    server_name <url>;

    # HTTP traffic (main viewer)
    location / {
        proxy_pass http://<rpi-vpn-ip>:5000;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }

    location /ws/ {
        proxy_pass http://<rpi-vpn-ip>:5001;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection 'upgrade';
        proxy_set_header Host $host;
        proxy_cache_bypass $http_upgrade;
    }
}
```
Might try working on not secure for now, until I can find a way for the websocket to be secure that rosbridge generates. https://answers.ros.org/question/378244/rosbridge-server-websocket-secure/

Also had to edit line 24 here: This will still be needed
```
/home/rpi/ros2_ws/install/vizanti_server/share/vizanti_server/public/js/modules/rosbridge.js
```

To look like this: 
```
		this.ros = new ROSLIB.Ros({
			url: 'ws://' + this.url + '/ws/'
		});
```

Now there is a problem serving the files from flask, added to `server.py` on line 98 the following to try and serve at `public` folder. Might not be needed when using the ros param below.
```
@app.route('/public/<path:filename>')
```

Ok fixed the path for flask, now can view with:
```
http://<url>/public/
```
Run like this to get there:
```
ros2 launch vizanti_server vizanti_rws.launch.py base_url:=/public
```
Changed this line to use http instead of https:
```
<iframe src="http://<url>/public/" width="1200" height="1000" frameborder="0"></iframe>
```

Also cleanup here:
```
chrome://flags/#unsafely-treat-insecure-origin-as-secure
```

Added ports 8443, 5000, 5001 and it worked on AWS. Check if all still needed.

### Getting SSL back
[ROS1 guy asks for SSL](https://github.com/RobotWebTools/rosbridge_suite/issues/232)
[ROS1 Pull request where SSL is added as param](https://github.com/RobotWebTools/rosbridge_suite/pull/26)
[ROS2 where you add ros parameters for ssl](https://github.com/RobotWebTools/rosbridge_suite/blob/460d202f2b4a054d4e7c77dd7dcf05c0d59648e9/rosbridge_server/launch/rosbridge_websocket_launch.xml#L5)
[Example from Ubiquity Robotics, ROS1](https://github.com/UbiquityRobotics/speech_commands)

You could edit [here](https://github.com/MoffKalast/vizanti/blob/8a2f122375366bb31bfe8a863672b48121328d54/vizanti_server/launch/vizanti_server.launch.py#L20) to use the ssl options in rosbridge params, but to do so you would have to switch to original rosbridge, not the rws one.

To use the RWS one, would have to edit it to allow tls [here](https://github.com/v-kiniv/rws/blob/fa93e856892ee88feb595c1b913d852175a7f27b/src/server_node.cpp#L17)

To check if there would be much difference, makes sense to run htop with the server running normal rosbridge and rws. Then can decide if the performance boost is worth effort getting rws changed (no idea how much work that would be). the non-rws version did not work out of the box, maybe need to change the RWM_IMPLEMENTATION.
Tried changing the RWM_IMPLEMENTATION and changing the address but still get:
`[rosbridge_websocket-1] WARNING:tornado.access:404 GET /ws/ (172.27.232.1) 12.79ms`

I think the next step here to better debug and also test the rosbridge version is to download a fresh Vizanti, and run it without the iframe stuff. Just try connecting with Thincast remote client.
I would first try fixing this issue with SLAM not working in Rviz, maybe a delay after the lidar turns on before SLAM starts <- this didn't work. Look here first [Similar issue NAV2](https://answers.ros.org/question/393581/for-nav2-lidar-timestamp-on-the-message-is-earlier-than-all-the-data-in-the-transform-cache/) 
Also 2 latest frames pdf are with lidar running, 1 before slam starts and 1 after. Try inspecting the timestamps of /scan and /scan_throttled.

### 2 factor authentication 2FA with Nginx
Article on [this blog](https://seantodd.co.uk/blog/putting-2fa-on-everything/) for 2FA with Nginx and Google Authenticator.
Links to [this repo](https://github.com/Arno0x/TwoFactorAuth).




