---
title: Adding Fail2Ban to prevent brute force attempts
date: 2025-01-30
---
Following the [guide here](https://www.digitalocean.com/community/tutorials/how-to-protect-an-nginx-server-with-fail2ban-on-ubuntu-20-04).

Tip: In Nano editor, you can do `CTRL + w` to open a search. Type what you want and hit enter, profit.
#### Configuring fail2ban
Could see in my nginx logs lots of attempts by bots to access the server. Fortunately with the password protection, these all return 401 errors. But lets add Fail2Ban to prevent brute force attacks.
```
sudo apt update
sudo apt install fail2ban
```
<!--more-->
Installs a service but doesn't enable or run it as you don't want to accidentally get locked out.
First make a copy of the `jail.conf` file in the same location for editing, called `jail.local`.
```
cd /etc/fail2ban
sudo cp jail.conf jail.local
```
We can setup email notifications for bans, but will leave that for now. Defaults here look good, lets just enable `nginx-http-auth` as by default only ssh is enabled.
Edit the section to look like the following `sudo nano jail.local`:
```
[nginx-http-auth]
enabled = true
port    = http,https
logpath = %(nginx_error_log)s
```
Also saw the following filter and enabled it:
```
nginx-botsearch
```

#### Enabling and Starting fail2ban
```
sudo systemctl enable fail2ban
sudo systemctl start fail2ban
```
And to check the status:
```
sudo systemctl status fail2ban
```

#### Getting info about enabled jails
List jails:
```
sudo fail2ban-client status
```
And to see status of a specific jail:
```
sudo fail2ban-client status nginx-http-auth
```
You can also see the effect of the ban on the IPTABLES:
```
sudo iptables -S | grep f2b
```
For example when I purposely got my password wrong on my phone:
```
-A f2b-nginx-http-auth -s 109.40.240.245/32 -j REJECT --reject-with icmp-port-unreachable
```
#### Unbanning
And then to unban, just do this:
```
sudo fail2ban-client set nginx-http-auth unbanip <your_IP>
```

