---
title: Making some visual changes to Vizanti
date: 2024-11-18
---
The location to copy edited files for the webpage is here.
```
/home/rpi/ros2_ws/install/vizanti_server/share/vizanti_server/public
```
and the command to copy is:
```
scp -r -i ~/.ssh/create_laptop ~/Documents/vizanti/vizanti_server/public rpi@<rpi_ip>:/home/rpi/ros2_ws/install/vizanti_server/share/vizanti_server
```
run with
```
ros2 launch vizanti_server vizanti_rws.launch.py base_url:=/public
```
<!--more-->

The following doesn't seem to be needed anymore.
```
+@app.route('/public/<path:filename>')
 def serve_static(path):
        return send_from_directory(app.static_folder, path)
```

If you want to remove the D-Pad buttons and text boxes, you need to edit the JS also. I tried to remove them and it said something about a non-existing item and emit. Basically just try to comment out everything about the D-Pad and text receive and entry boxes.

### Colour changes
Grid background colour: `3e556a`to this is the grid lines `179617`
Grid background colour: `3e556a` to this is the grid lines `179617`
background-color: colour of the grid background: `273444` to normal background `191A19`
Thin bar between grid and icons: `444444` to `9715A3` is `icon_bar_handle::before`, also do `7px` height down to `3px`.

### Adding default layout config Vizanti
##### 2025-02-16
Added a new file called `vizanti_mp_robot_config.json` to `/home/rpi/systemd-services`.
Also edited `robot_launch.sh` to pass this file in as a parameter when starting Vizanti.
Committed [here](https://github.com/MJPye/robot_with_webrtc/commit/f1d5b05aa694fb18d8573f162e190043c0297a0a).