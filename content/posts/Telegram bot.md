---
title: Telegram bot for login and ban notifications
date: 2025-02-26
---
Let's create a Telegram bot that sends a notification to my phone when there are multiple failed attempts to login, or when there is a successful login. Was more practical for me than email based notifications and cronjobs.

You can follow instructions [here](https://core.telegram.org/bots/tutorial) to create the Telegram bot with BotFather. BotFather will return the BotID.
<!--more-->
How to get the chat id of a user talking to the bot
```
https://api.telegram.org/<bot+bot_id>/getUpdates
```

Send a message
```
https://api.telegram.org/<bot+bot_id>/sendMessage?chat_id=<chat_id>&text=test345
```

### Telegram bot and Fail2Ban
Let's create a Telegram bot that sends a notification to my phone when there are multiple failed attempts to login.
Added `/etc/fail2ban/action.d/telegram.conf`
```
[Definition]

actionban = curl -s -X POST "https://api.telegram.org/bot<bot_id>/sendMessage" -d "chat_id=<chat_id>&text=Banned IP: <ip>"

actionunban = curl -s -X POST "https://api.telegram.org/bot<bot_id>/sendMessage" -d "chat_id=<chat_id>&text=Unbanned IP: <ip>"
```

#### We still need to check for successful logins and send a message.
Some `nginx_access_success.sh` script which is run by a cronjob every minute, the script should only try to read messages from the last minute.

Created this script which is run by cronjob at `/home/openvpnas/nginx_access_success.sh`, is saved in repo.

The crontab entry for root user is edited with `sudo crontab -e`:
```
* * * * * /home/openvpnas/nginx_access_success.sh
```

### Current state
So when an IP is banned or un-banned, we get the telegram notification due to edited `jail.local` and `/etc/fail2ban/action.d/telegram.conf`.

When there is a successful login detected by parsing HTTP returning 200 in `/var/log/nginx/access.log`, we also get a notification from the script triggered every minute by the cronjob.