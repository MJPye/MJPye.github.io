---
title: Waypoint missions with NAV2
date: 2025-08-09
tags: ["MobileRobot"]
---
We have the robot UI working and can give single Pose Goals, now lets try to link different goals and actions together to make an autonomous mission.
<!--more-->

Look into `nav2_waypoint_follower`, which should work better with dumb robot, smart dispatcher. The docs [here](https://docs.nav2.org/concepts/index.html#waypoint-following) suggest to use `nav2_behavior_tree` instead for more complex behaviour and to do stuff like check robot battery in between waypoints.

Waypoint Follower configuration guide is [here](https://docs.nav2.org/configuration/packages/configuring-waypoint-follower.html#waypoint-follower).

Not to be confused is the Route Server. This is a plugin for navigating routes through a pre-defined Navigation graph, rather than through free space. Think of a robot that should only travel on certain highways in a factory.

The more complex is [Behaviour-Tree Navigation](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html#behavior-tree-navigator) which can now be visualised with a tool called GROOT. For more docs see [here](https://docs.nav2.org/behavior_trees/index.html#nav2-behavior-trees).

Oh well, can use Groot as an Editor but it no longer features live monitoring: [Github Issue](https://github.com/ros-navigation/navigation2/issues/3770). Even in newer versions of ROS2, live visualisation is an expensive Pro feature.

When I tried to run nav2 with a config file it crashed and won't generate Costmaps: `ros2 launch nav2_bringup navigation_launch.py params_file:=/home/rpi/robot_configs/bt_groot.yaml`

```
bt_navigator:
  ros__parameters:
    enable_groot_monitoring: true
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
```

### Anatomy of a Waypoint Mission
So I'm thinking first we need to create a map of the apartment, then load that in. I guess if we assume we always start missions on the dock, then SLAM would also work, but for now a static map seems like a good idea. Still have costmaps for obstacle avoidance.

But what is launched by the different launch files in NAV2 bringup?

| Launch File           | Map Server | Localization (AMCL) | SLAM | Nav2 Core | RViz     | Use Case                          |
|----------------------|------------|--------------------|------|-----------|----------|----------------------------------|
| `bringup_launch.py`   | ✅         | ✅                 | ❌   | ✅        | Optional | Full navigation with pre-made map|
| `navigation_launch.py`| ❌         | ❌                 | ❌   | ✅        | Optional | Nav stack only, external localization |
| `localization_launch.py` | ✅      | ✅                 | ❌   | ❌        | Optional | Localization only                |
| `slam_launch.py`      | ❌         | ❌                 | ✅   | ✅        | Optional | Mapping + navigation             |
| `rviz_launch.py`      | ❌         | ❌                 | ❌   | ❌        | ✅       | Visualization only              |
So plan is as follows:
- Map the environment and save the map using `slam toolbox`.
- Run `bringup_launch.py` version of NAV2, and don't run `slam toolbox`.
- Set an initial Pose using Rviz on the docking station location then monitor the value of `/initialpose`.
- `/initialpose` is then passed every time the robot wakes up.
- The `/map` frame will be consistent each time so we can add waypoints.
- Maybe can add a button to the UI called `start mapping mode`, which has a separate launch file.