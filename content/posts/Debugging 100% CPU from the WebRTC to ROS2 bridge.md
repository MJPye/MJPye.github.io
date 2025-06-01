---
title: Debugging 100% CPU usage from the WebRTC to ROS2 bridge
date: 2025-05-22
---
### Issue with CPU usage spike
##### What is the issue?
When I am running the server with "node index.js" and access it, I click "start data" button and everything works. htop on the computer the server runs on shows 10% use on each CPU core, so all good. I click "start video", the stream looks fine and CPU cores are also fine. After 10 seconds or so, one of the CPUs maxes out at 100% and stays there permanently.

**The issue actually comes from the webrtc-to-ros2 node**
and it happens without even starting the rtsp screens.
Happens after pressing "start data" at some point. Break down `index.js` in `webrtc-to-ros2` to figure this out.
Have a go at debugging yourself, here is an example: https://stackoverflow.com/questions/36502849/node-js-high-cpu-usage-how-to-debug-it

Run with the profiler:
```
node --title="webrtc-to-ros2" --prof index.js
```
I let it get to 100% usage then after some time killed the screen. Left with a file called:
```
isolate-0x326e0000-228025-v8.log
```

Process the file like so:
```
node --prof-process isolate-0xnnnnnnnnnnnn-v8.log > processed.txt
```

my version is at:
```
/Users/matthewpye/Documents/processed_log_19_May.txt
```


### perf and strace
perf report
```
Samples: 44K of event 'cycles', Event count (approx.): 19480244680
  Children      Self  Command          Shared Object             Symbol
+   98.31%     0.00%  PeerConnectionF  libc.so.6                 [.] thread_start
+   98.31%     0.00%  PeerConnectionF  libc.so.6                 [.] start_thread
+   98.31%     0.00%  PeerConnectionF  wrtc.node                 [.] rtc::Thread::PreRun
+   98.31%     0.00%  PeerConnectionF  wrtc.node                 [.] rtc::Thread::Run
+   98.25%     0.00%  PeerConnectionF  wrtc.node                 [.] rtc::Thread::Get
+   97.69%     0.78%  PeerConnectionF  wrtc.node                 [.] rtc::PhysicalSocketServer::WaitEpoll
+   83.18%     0.00%  PeerConnectionF  [kernel.kallsyms]         [k] el0t_64_sync
+   83.14%     0.00%  PeerConnectionF  [kernel.kallsyms]         [k] do_el0_svc
+   47.70%     6.55%  PeerConnectionF  libc.so.6                 [.] __getsockopt
+   47.10%     5.03%  PeerConnectionF  libc.so.6                 [.] epoll_pwait
```
Captured perf report and checked it. Reason seems to be repeated calls to `epoll_pwait` which can be seen using strace.
Interesting is that the existing process is not the one that does all these calls. When the problem happens, a second process starts with a different PID. That is the one that does all the system calls which causes the 100% usage.
`sudo strace -p 238118`
will show:
```
epoll_pwait(49, [{events=EPOLLHUP, data={u32=2417413920, u64=281460214255392}}], 128, 232, NULL, 8) = 1
getsockopt(92, SOL_SOCKET, SO_ERROR, [0], [4]) = 0
epoll_pwait(49, [{events=EPOLLHUP, data={u32=2417413920, u64=281460214255392}}], 128, 231, NULL, 8) = 1
getsockopt(92, SOL_SOCKET, SO_ERROR, [0], [4]) = 0
epoll_pwait(49, [{events=EPOLLHUP, data={u32=2417413920, u64=281460214255392}}], 128, 231, NULL, 8) = 1
getsockopt(92, SOL_SOCKET, SO_ERROR, [0], [4]) = 0
epoll_pwait(49, [{events=EPOLLHUP, data={u32=2417413920, u64=281460214255392}}], 128, 231, NULL, 8) = 1
getsockopt(92, SOL_SOCKET, SO_ERROR, [0], [4]) = 0
epoll_pwait(49, [{events=EPOLLHUP, data={u32=2417413920, u64=281460214255392}}], 128, 230, NULL, 8) = 1
```

```
UV_USE_IO_URING=0
```
did not work

Tricky issue, I think best bet is to selectively remove everything until we know exactly which function call gives the problem.

Need to fix package.json for webrtc-to-ros2 also.

Tested with signalling server code from before refactor and problem is still there. Lets now test with old `webrtc-to-ros2` code.

## Solution
The solution is just to use this package instead of `wrtc`: https://github.com/WonderInventions/node-webrtc