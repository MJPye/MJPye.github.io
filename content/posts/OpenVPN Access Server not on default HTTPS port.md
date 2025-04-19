---
title: Moving OpenVPN access server away from default port 443
date: 2025-02-16
---
### Moving OpenVPN to port 4443 and admin panel to 943
So currently when a user wants to use the robot dashboard, they need to add port 8443. This is because the OpenVPN access server by default runs on port 443 which is default HTTPS port. Going to try and change the default VPN port so that my domain by default points to the robot dashboard.
<!--more-->

Changed Default TCP Port number:
- Was 443, now 4443.
To access the Admin panel, you now must go to port 943 directly:
```
https://<url>:943/admin/
```

#### Making Robot stuff appear on 443 (default HTTPS)
- Change port number in `nginx_working_https`.
- Remove port 8443 from `signalling-server/index.html`.
- `vizanti_server/public/js/modules/rosbridge.js` remove reference to 8443 here.
Changed in commit [here](https://github.com/MJPye/robot_with_webrtc/commit/881642830aa0eee5ebd9a1dd9a1752711aab3282) and [here](https://github.com/MJPye/vizanti/commit/93583a9657dd25559ec9701e81fdcb7c9b80dc05)
