---
title: Bi-directional WebRTC data for Robot Status Updates
date: 2025-06-14
---
I want to get robot status updates in the UI. First step is 2 way communication. Made a test using this [commit](https://github.com/MJPye/robot_with_webrtc/commit/31851b41609b6cc376fd9242b1da31ae42fad30f).
### Side note - adding a wireframe
I took the Downloadable 3D STEP model from the [Create3 Docs](https://iroboteducation.github.io/create3_docs/hw/mechanical/).
With it loaded in Fusion 360, I was able to export an `obj` file.
The `obj` file can be loaded using ThreeJS as shown in this [example](https://threejs.org/examples/?q=obj#webgl_loader_obj) and it's [source code](https://github.com/mrdoob/three.js/blob/master/examples/webgl_loader_obj.html).

With the `obj` file loaded, I created the wireframe by drawing the model first in black, then over the top with green lines. This prevents being able to see lines that should be hidden. 

All of the required files are stored at `/Users/matthewpye/Documents/create3_wireframe_js` and can be run with `python3 -m http.server`.

From then on it was just changing some positions, animation rotations and colours to end up with a nice widget that can go somewhere on the UI. Have not tested yet to see the impact on performance.

<video src="/images/wire_mesh_threejs.mov" autoplay muted loop playsinline style="max-width:100%; height:auto;"></video>