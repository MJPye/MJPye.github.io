---
title: Setting up a new RPi4, manual steps which I should automate
date: 2024-06-09
---
### Manual steps
Below are some manual steps to setup a new Raspberry Pi 4 for use with the create3 robot and created OpenVPN and AWS servers, I used these steps when updating from RPi3 to RPi4.
Plan next is to test these steps again and convert to a bash script or [Ansible](https://docs.ansible.com/) playbook.
<!--more-->
```bash
sudo apt update
sudo apt upgrade

#Enable passwordless sudo
echo "$USER ALL=(ALL) NOPASSWD:ALL" | sudo tee -a /etc/sudoers

#Enable SSH on RPi 4
sudo apt install openssh-server -y

#Copy ssh public key to RPi at /home/rpi/.ssh/authorized_keys
sudo nano /etc/sshd_config
sudo nano /etc/ssh_config
PasswordAuthentication no
sudo service ssh restart

#OpenVPN client setup
#Copy across the ovpn client file to /etc/openvpn/client/vpn-client.conf, then add the following line:
auth-user-pass login.conf
#Then add credentials to a file called login.conf
cd /etc/openvpn/client && sudo chmod 400 login.conf vpn-client.conf
sudo systemctl enable openvpn-client@vpn-client.service
sudo systemctl start openvpn-client@vpn-client.service

sudo apt install net-tools
sudo apt install traceroute

#Can now login from laptop connected to VPN with:
ssh -i /Users/matthewpye/.ssh/create_laptop -o "ProxyCommand ssh -i /Users/matthewpye/Documents/vpn_server/vpn_server.pem -W %h:%p openvpnas@<ec2_ip>.eu-central-1.compute.amazonaws.com" rpi@<rpi_ip>

sudo apt install xrdp
```

### Turn off auto-login
Turn off auto login, as cannot be logged in locally and remotely at the same time with xrdp
```bash
rpi@rpi-desktop:/etc$ cat /etc/gdm3/custom.conf
# GDM configuration storage
#
# See /usr/share/gdm/gdm.schemas for a list of available options.

[daemon]
AutomaticLoginEnable=False
AutomaticLogin=rpi

# Uncomment the line below to force the login screen to use Xorg
#WaylandEnable=false

# Enabling automatic login

# Enabling timed login
#  TimedLoginEnable = true
#  TimedLogin = user1
#  TimedLoginDelay = 10

[security]

[xdmcp]

[chooser]

[debug]
# Uncomment the line below to turn on debugging
# More verbose logs
# Additionally lets the X server dump core if it crashes
#Enable=true
```

Use Thincast client with <rpi_ip> and rpi user.

Currently ssh, openvpn and xrdp server all start on boot. This allows ssh over VPN and remote desktop over VPN. Can SSH and Remote Desktop at the same time. Need to check the network usage on the VPN server when doing this, may start to cost something.
Keep track of data usage on the VPN server [here](https://us-east-1.console.aws.amazon.com/billing/home?region=us-east-1#/freetier).
#### Setting up streaming
```bash
#First check if you can get an image from the webcam by installing ffmpeg
sudo apt install ffmpeg
#Then some tools to help with camera
sudo apt install v4l-utils

#Working with the camera in darkness, taking single images
sudo v4l2-ctl -d /dev/video0 --set-ctrl auto_exposure=1 # <--(manual mode)
sudo v4l2-ctl -d /dev/video0 --set-ctrl exposure_time_absolute=10000
sudo v4l2-ctl -d /dev/video0 --set-ctrl brightness=20
sudo ffmpeg -f v4l2 -video_size 1280x720 -i /dev/video0 -frames 1 out.jpg

# Creating the stream
sudo gst-launch-1.0 v4l2src device=/dev/video0 ! 'image/jpeg,width=640,height=480,framerate=30/1' ! jpegparse ! multipartmux boundary=spionisto ! tcpserversink host=0.0.0.0 port=8080

# also works
sudo gst-launch-1.0 v4l2src device=/dev/video0 ! 'image/jpeg,width=640,height=480,framerate=30/1' ! jpegparse ! tcpserversink host=0.0.0.0 port=8080

#This creates a stream viewable in VLC media player at:
tcp://172.27.1.3:8080

#rtsp stream
cd /home/rpi/webcam_testing/rtsp-server.py
sudo ./rtsp-server.py
#Then access with vlc
rtsp://192.168.0.188:8554/test
```

These were the first attempts at image capture and streaming from the webcam, will explain the full setup in a later post.

### Getting USB to work as Ethernet, creating usb0 interface
Files to edit are located in `/boot/firmware/config.txt` and `/boot/firmware/cmdline.txt`.
Add the following to `/boot/firmware/config.txt`:
```bash
dtoverlay=dwc2,dr_mode=peripheral
```
Add the following after `rootwait` in `/boot/firmware/cmdline.txt`:
```bash
modules-load=dwc2,g_ether
```

Can now see the modules loaded correctly with `lsmod | grep -E 'dwc2|g_ether'`.
Next steps:
- Connect the rpi4 to the robot and check if `usb0` interface is created. -done
- Attach webcam to roof of robot and try to call with WebRTC again. -done
- Make a backup of current SD card in rpi4. -underway
- Install ROS2 on the rpi4, try to drive while calling through webRTC. -ready to test

#### usb0 connection
`usb0` connection profile for `NetworkManager`
```bash
sudo cat /etc/NetworkManager/system-connections/usb0.nmconnection 

[connection]
id=usb0
type=ethernet
interface-name=usb0
autoconnect=false

[ipv4]
method=manual
addresses=192.168.186.3/24
never-default=true

[ipv6]
method=auto
never-default=true
```

#### Installing ros2-humble

```
sudo apt update && sudo apt install -y curl gnupg2 lsb-release build-essential git cmake

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg 
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null 
sudo apt update

sudo apt install -y ros-humble-desktop 
sudo apt install -y ros-humble-irobot-create-msgs 
sudo apt install -y build-essential python3-colcon-common-extensions python3-rosdep ros-humble-rmw-cyclonedds-cpp

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```
After following these steps, the robots topics were automatically listed with `ros2 topic list`.

#### How I created a backup
Backing up and restoring rpi SD card on mac: https://pimylifeup.com/backup-raspberry-pi/#backup-up-your-raspberry-pi-sd-card-on-os-x
Takes around 45 mins to create the backup, and 223 minutes to restore one.
Left the latest backup (no ROS) in the white SD card adapter.