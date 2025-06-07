---
title: 3D Printing Robot Accessories
date: 2025-05-04
---
In 2024 I bought a [Bambu Lab A1 mini](https://eu.store.bambulab.com/de/products/a1-mini-de?variant=53687077372252&srsltid=AfmBOopoGYvPMDh-c0Fb66Z2q_L9TDGdfamtqXGMOfNUWpJAezwZpcXF) for €200. It comes fully assembled and I have found it to be an amazing tool to have around, far better than the 3D printers that were available when I was at university in 2016.

The printer was bought for other purposes, but it has been useful for printing accessories and mounts for the Create 3 robot. There is a library of mounts in the [Create 3 Docs](https://iroboteducation.github.io/create3_docs/hw/print_sensor_mounts/), but not for the devices I am using.

The faceplate and internal cargo bay of the Create 3 robot feature a **12mm grid of "3mm diameter"** mounting holes, though I believe they are more like 3.2mm.

![Image Description](/images/create3_payload_surfaces.svg)

### LiDAR holder
The first part to print for the robot was a mount for an [RPLidar C1](https://www.slamtec.com/en/C1). This was just a holder with a wall missing on one side for cables.

![Image Description](/images/LiDAR_holder.png)


### Reolink camera holder
The [Reolink E1 zoom](https://reolink.com/de/product/e1-zoom/?srsltid=AfmBOookvPFs8GD6HHeOGWlzj1j_C8yrEwrdLXJuJ7Y9oRyJAvPvJMB7) has a mounting plate included in the packaging which screws onto the bottom. The mounting plate has 2 x 4mm holes 18mm apart, which are in the centre of the holder in a 0.5mm recess.
<video src="/images/reolink_holder_slicer.mov" autoplay muted loop playsinline style="max-width:100%; height:auto;"></video>
To add space underneath the Reolink mount, I printed some plastic shims also, around 15mm. This allows the cable for the RPLidar to run underneath them. 

### Assembled Robot
With the antenna attached to the top of the robot and some flexible Ethernet cables for wiring, the robot assembly is for now complete 🦾.
After months with a webcam on a stand and cables filling the cargo bay, the robot is now looking clean and I can go back to working on software. 
Next up, Reolink camera controls in the UI.

![Image Description](/images/assembled_robot.jpg)

P.S The camera is blocking around 20% of the LiDAR's field of view, will find a solution for this later.