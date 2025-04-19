---
title: OpenVPN setup on Raspberry Pi
date: 2024-05-10
---
I first tried using [Husarnet](https://husarnet.com/) and opening a port on my router, but I found it was not stable (the router). Switched to using OpenVPN instead and had more success. Has been a while since I tried Husarnet though, likely worth revisiting.
### OpenVPN server
https://openvpn.net/faq/connection-initiated-with-x-x-x-x-but-i-cannot-ping-the-server-through-the-vpn/ - Then restarted the OpenVPN service on the server.
```
Start command in screen on rpi:
sudo openvpn --config profile-<profile-number>.ovpn

Connect laptop to VPN client.

ssh -i /Users/matthewpye/.ssh/create_laptop -o "ProxyCommand ssh -i /Users/matthewpye/Documents/vpn_server/vpn_server.pem -W %h:%p openvpnas@<ec2-ip>.eu-central-1.compute.amazonaws.com" rpi@<rpi-vpn-ip>

https://<ec2-ip>:943/?src=connect

```
### No ping response from server
Can ssh into the rpi from laptop just fine, and also ping between them when they are connected to OpenVPN server. Cannot ping from a client to the VPN server gateway address with `ping 172.27.232.1`, which would have been useful for checking VPN is active on RPi.
### OpenVPN client service on RPi
Added the file to `/etc/openvpn/client/vpn-client.conf` and the credentials to login in a file called `login.conf` at the same location. `vpn-client.conf` is also stored locally in `Documents/vpn_server`.
```
<user>
<password>
```
Now on boot, a service called `openvpn-client@vpn-client.service` should start and connect us to the VPN. We can check the status of this service to decide if we need to reboot.
### Creating static IPs for devices in OpenVPN admin panel
Let's set up static IPs for each device, seems to be done by creating multiple user profiles and assigning each a static IP.
- Created the second openvpn_laptop profile and use that on the laptop.
- Using the openvpn profile on the rpi.
- Maximum 2 connections on OpenVPN free plan.
#### What are we trying to develop in the near future?
- Able to login to a UI from anywhere without VPN.
	- Server needs to be hosted in AWS, as the Vodafone router will not enable me to open ports.
- From this UI, able to view robot camera feeds.
	- WebRTC stream camera data, through VPN.
- Able to view robot location data.
	- WebRTC data channel. (Avoiding the RosBridge).
- Able to trigger robot missions and drive the robot with a gamepad.
	- 2-way data channel.
#### What are the first steps for this?
MVP for a demo would be:
- Robot with Camera connected to raspberry pi.
- Camera data visible on client laptop.
- Client laptop able to send gamepad or keystroke commands to move robot.
#### Plan to achieve first steps
- Camera data streaming to WebRTC, can view the stream from laptop when connected to VPN.
- Lets get a RPi and camera connected to the robot, view the stream, then trigger ROS2 commands from the RPi when ssh'd into it.
#### Shopping list
- RPi 4 8GB [link](https://www.amazon.de/-/en/Raspberry-Model-ARM-8GB-Linux/dp/B09TTKT94J/ref=pd_ci_mcx_pspc_dp_d_2_t_3?pd_rd_w=b5bXX&content-id=amzn1.sym.7838253a-f0cb-4765-8290-3a5e015d7ac1&pf_rd_p=7838253a-f0cb-4765-8290-3a5e015d7ac1&pf_rd_r=ZFJC4EMX9G3VVEAWBEFS&pd_rd_wg=2Yuz4&pd_rd_r=fd188a76-7e23-43b4-b3a1-8d5cfa4ea8ae&pd_rd_i=B09TTKT94J&th=1)
- RPi 4 Power supply [link](https://www.amazon.de/-/en/Official-Standard-Raspberry-Supply-iUniker/dp/B09L49DKCS/ref=sr_1_2?crid=2TTNUVPXOJ2GP&dib=eyJ2IjoiMSJ9.OHjcW2qd867_PHA1VsiqPbum75X-DftVVmXChyO-KMuG6OSpW0VmxjgN8qwBrCZ19KD6l52eT4C9Ko1M4tzlA4U8yONcgjcEy0OzHHGG3eLojXN9K82iusHjbg2A8ywKIomyKn1qf2T3gnGsFZi-xgKAebmyQ_8yTvWSloglfVKUhAeGkuSWmf9mM_EwyGzYhe8edmWhcZ8RVjSo1Cj7zRwopEZPwdYpgxZrd5QkuXoooTWiQYW4-rrchSw1GzrEDbzYgcUgT5NMsIMz_UFCOwKgGt7BGzv7qox3vtRZVeU.p6bvDd2pgtGIDT_TBhV3_1VzfVaRPCt7sxmGx_NMD8g&dib_tag=se&keywords=rpi4+power+supply&qid=1717345064&s=computers&sprefix=rpi4+power+supply%2Ccomputers%2C88&sr=1-2)
- RPi 4 case [link](https://www.amazon.de/-/en/Geekworm-Raspberry-Aluminium-Aluminum-Dissaption/dp/B07ZVJDRF3/ref=sr_1_3?crid=3HDPPVXYJTFUM&dib=eyJ2IjoiMSJ9.7E1Q6COIkZRTNUb2IfTltshwOR6Lk1k_pC1h3vLIRuM4W7uDr0lYVs9EVtaA8-eY_9A5_dD1ew81M8hyzADxv6Lneneo5oHVBVkRNtb27ss5ryTWHezBgViJRuQk0ONwNa7SH1zRzXbQfP3EQ7MnPnWmZpftZ48DAYlbxAGotHCViu_L5lzSpHhrr72eU6P86gRp8p6cDmcZgdF3E8tYOjy8ifoioYWAfDuh_7Ql0Y0.vkmmh9UnzocdD1cCiyNi1O1-J2EFC8eJi9nKOaxYiQ4&dib_tag=se&keywords=rpi4+case&qid=1717345286&sprefix=rpi4+%2Caps%2C88&sr=8-3)
- USB-C to USB-C cable [link](https://www.amazon.de/-/en/Delivery-Charging-Compatible-MacBook-MateBook-natural/dp/B07PLSDPYN/ref=sr_1_3?crid=27OV6ROFSFKUK&dib=eyJ2IjoiMSJ9.bC-Dde4ex091BieGGRixmy65JZdzJXCFkbqC05Bt7MPIWtK2z8XiTlHz-eUwuWwxWeVumyvPbMDwZGoMyaFuOzOsnPKO7ErpN_E1j342RiHGusUHYXei8Sys7euaoMjEwov6H2dnqdXzkSvHKLHMXK3F6tbhr7XKAibED8nMeWI0I0uYEOqdjR4Vv2He3b4wCn_6Vrzsq6OeomiQEEpauOCwHE9ReuhUa8cRLi1n4ZIsDIscePLFgt0sm4NdNteRtPN37ZT0REAa3FLO42M6PSjlNSymNMmMoxpQAE20rtc.95pgoA2EGtX6AjXjhdlNN3jYmkv4fFdW5ZvkLioXprk&dib_tag=se&keywords=usb%2Bc%2Bto%2Busb%2Bc&qid=1717345123&s=computers&sprefix=usb%2Bc%2Bto%2B%2Ccomputers%2C92&sr=1-3&th=1)
- Ordered these parts.