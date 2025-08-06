---
title: Waypoint missions with NAV2
date: 2025-08-06
---
We have the robot UI working and can give single Pose Goals, now lets try to link different goals and actions together to make an autonomous mission.

Look into `nav2_waypoint_follower`, which should work better with dumb robot, smart dispatcher. The docs [here](https://docs.nav2.org/concepts/index.html#waypoint-following) suggest to use `nav2_behavior_tree` instead for more complex behaviour and to do stuff like check robot battery in between waypoints.

Waypoint Follower configuration guide is [here](https://docs.nav2.org/configuration/packages/configuring-waypoint-follower.html#waypoint-follower).

Not to be confused is the Route Server. This is a plugin for navigating routes through a pre-defined Navigation graph, rather than through free space. Think of a robot that should only travel on certain highways in a factory.

The more complex is [Behaviour-Tree Navigation](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html#behavior-tree-navigator) which can now be visualised with a tool called GROOT. For more docs see [here](https://docs.nav2.org/behavior_trees/index.html#nav2-behavior-trees).