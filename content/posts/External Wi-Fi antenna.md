---
title: Adding External Wi-Fi antenna to robot - or how to drive in the kitchen
date: 2025-01-15
---
Github with installation instructions here [here](https://github.com/morrownr/8821cu-20210916), for this [product](https://thepihut.com/products/raspberry-pi-dual-band-5ghz-2-4ghz-usb-wifi-adapter-with-antenna).

Install speedtest [here](https://www.speedtest.net/apps/cli) and run with `speedtest`.

<!--more-->

Result with **no antenna** from living room:
```
      Server: LWLcom GmbH - Berlin (id: 53495)
         ISP: Vodafone Germany
Idle Latency:    10.67 ms   (jitter: 0.65ms, low: 10.56ms, high: 12.68ms)
    Download:    53.86 Mbps (data used: 42.4 MB)
                153.09 ms   (jitter: 48.94ms, low: 14.20ms, high: 237.29ms)
      Upload:    10.74 Mbps (data used: 18.2 MB)
                 18.37 ms   (jitter: 8.93ms, low: 7.88ms, high: 289.17ms)
 Packet Loss:     0.0%
  Result URL: https://www.speedtest.net/result/c/dcd7f435-ed2c-4968-be81-fe1d77f63eb6
```

Result with **no antenna** from kitchen:
```
      Server: LWLcom GmbH - Berlin (id: 53495)
         ISP: Vodafone Germany
Idle Latency:    12.67 ms   (jitter: 1.84ms, low: 11.57ms, high: 15.24ms)
    Download:    16.51 Mbps (data used: 14.9 MB)
                427.29 ms   (jitter: 78.64ms, low: 44.57ms, high: 884.51ms)
      Upload:    10.57 Mbps (data used: 17.0 MB)
                 34.43 ms   (jitter: 26.02ms, low: 8.20ms, high: 1245.16ms)
 Packet Loss:     0.0%
  Result URL: https://www.speedtest.net/result/c/57070a27-3f80-4959-9e25-959c78413e59
```

Result **with antenna** from kitchen, results look improved:
```
      Server: LWLcom GmbH - Berlin (id: 53495)
         ISP: Vodafone Germany
Idle Latency:    12.16 ms   (jitter: 1.89ms, low: 10.72ms, high: 13.30ms)
    Download:    53.93 Mbps (data used: 40.4 MB)
                150.81 ms   (jitter: 48.53ms, low: 21.38ms, high: 245.11ms)
      Upload:    10.87 Mbps (data used: 6.6 MB)
                 21.67 ms   (jitter: 13.32ms, low: 9.03ms, high: 180.27ms)
 Packet Loss:     0.0%
  Result URL: https://www.speedtest.net/result/c/baba26b4-54b7-4bac-b539-e90bf78018df
```
#### Installing
```
sudo apt install -y build-essential dkms git iw
cd ~
mkdir antenna_driver
cd antenna_driver
git clone https://github.com/morrownr/8821cu-20210916.git
cd 8821cu-20210916
sudo ./install-driver.sh
```

Will show:
```
Running module version sanity check.
 - Original module
   - No original module exists within this kernel
 - Installation
   - Installing to /lib/modules/5.15.0-1070-raspi/updates/dkms/
```
### Editing options
I didn't change anything, it worked out the box, may be possible to improve performance if needed.
```
nano /etc/modprobe.d/8821cu.conf
```

#### After install
After install and rebooting, the device will show and can be used in a network connection profile.
```
rpi@rpi-desktop:~$ nmcli d
DEVICE                   TYPE      STATE                   CONNECTION
wlx1cbfce2684be          wifi      connected               wlx1cbfce2684be-Vodafone-1A49
```

Created the following connection profile `wlx1cbfce2684be-Vodafone-1A49.nmconnection`:
```
[connection]
id=wlx1cbfce2684be-Vodafone-1A49
uuid=96243c6e-92c6-4bb3-a9f3-ff28b5aa8397
type=wifi
interface-name=wlx1cbfce2684be
autoconnect-priority=10

[wifi]
mode=infrastructure
ssid=Vodafone-1A49

[wifi-security]
auth-alg=open
key-mgmt=wpa-psk
psk=<password>

[ipv4]
route-metric=100
method=auto
addresses=192.168.0.197/24

[ipv6]
addr-gen-mode=stable-privacy
method=auto

[proxy]
```

### Handling fallbacks and routes with NetworkManager
We can keep both connections running on `wlan0` and `wlx1cbfce2684be`, they just can't have the same IP. I want the behaviour to be use `wlx1cbfce2684be` when it is available, and `wlan0` when not. Achieved this with 2 parameters in the connection profiles:
- `ipv4.route-metric=100` - We want a lower value for the preferred one.
- `connection.autoconnect-priority=10` - We want a higher value for the preferred one.
Tested this and when the antenna is disconnected, that connection goes down, and `wlan0` takes over.
Antenna connection is currently set to `192.168.0.197`.
The VPN on `tun0` goes through the default route, so as long as one connection is active the VPN stays online.

To-Do:
- Test with a USB3 female to male cable when it is delivered.