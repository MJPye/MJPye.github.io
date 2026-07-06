---
title: Training Action Chunking Transformer (ACT) on SO-101
date: 2026-07-06
tags: ["RobotArms"]
---
I have been working on a mobile robot platform for some time now, but I thought it would be nice to add some manipulation capabilities. For now I would like to keep the mobile robot and the robot arms separate, so I am testing a dual SO-101 setup on it's own workstation. Hopefully these will interact with the mobile robot in the near future.

Following guidance from [Hugging Face ACT](https://huggingface.co/docs/lerobot/en/act) and [Hugging Face IL](https://huggingface.co/docs/lerobot/en/il_robots), I started with the ACT policy because it is:
- **Fast Training**: Trains in a few hours on a single GPU
- **Lightweight**: Only ~80M parameters, making it efficient and easy to work with
- **Data Efficient**: Often achieves high success rates with just 50 demonstrations

## SO-101 Setup
I setup a pair of SO-101 arms, 1 follower and 1 leader. These are directly connected via USB to a Linux workstation with an NVIDIA RTX 5060 for training.
The follower arm has a wrist mounted camera and an overhead webcam, similar to the setup described on the [Robot Studio Github](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Optional/Overhead_Cam_Mount_32x32_UVC_Module/README.md).

#### Camera Setup
To prevent camera devices changing number, for example `/dev/video0` to `/dev/video1`, I pass the path to the teleoperate and record commands like so:
```
--robot.cameras="{ wrist: {type: opencv, index_or_path: "/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0", width: 640, height: 480, fps: 30}, overhead: {type: opencv, index_or_path: "/dev/v4l/by-id/usb-Generic_NexiGo_N660P_FHD_Webcam_200901010001-video-index0", width: 640, height: 480, fps: 30}}"
```
We can also fix parameters of the cameras with `v4l2`:
```
v4l2-ctl -d /dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0 -c brightness=20
v4l2-ctl -d /dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0 -c auto_exposure=1
```

#### LeRobot install
I installed LeRobot following the [Hugging Face Guide](https://huggingface.co/docs/lerobot/en/installation), which uses miniforge and conda.
Initialisation of the conda environment is handled by a shell script and alias called `mattaconda`.

#### Motor setup and Calibration
I then setup the motor IDs for both the follower and leader:
```
lerobot-setup-motors \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0
```
Then ran calibration:
```
lerobot-calibrate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=matt_follower_arm
```
From then onwards it was possible to teleoperate the follower arm using the leader arm:
```
lerobot-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=matt_follower_arm \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0\
    --teleop.id=matt_leader_arm
```
Also with cameras displaying in rerun:
```
lerobot-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=matt_follower_arm \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=matt_leader_arm \
    --robot.cameras="{ wrist: {type: opencv, index_or_path: "/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0", width: 640, height: 480, fps: 30}, overhead: {type: opencv, index_or_path: "/dev/v4l/by-id/usb-Generic_NexiGo_N660P_FHD_Webcam_200901010001-video-index0", width: 640, height: 480, fps: 30}}" \
    --display_data=true
```
Now that the arm can be tele-operated with cameras, it was time to start collecting training data.

### Task Definition
To begin, the goal would be completion of a simple pick and place task. The arm should pick up a solid blue cube and place it in a container. The container position would be fixed, with the cube's x and y coordinate changing; orientation of the cube would remain constant.
I split the workspace in 9 sections with clear tape, mainly to help me label the training data.

Rules I followed for data collection:
- Start from the same home position
- If you fumble or take multiple tries, re-record that episode.

More data can be added or merged into 1 dataset in the future like so:
```
lerobot-edit-dataset \
    --new_repo_id ${HF_USER}/record_test_merged \
    --operation.type merge \
    --operation.repo_ids "['${HF_USER}/record-test', '${HF_USER}/record-test2']"
```

**Note:** Any time you see `${HF_USER}` this is just an environment variable set by the following, after doing `hf auth login --token ${HUGGINGFACE_TOKEN}`:
```
export HF_USER=$(hf auth whoami | awk '/user:/ {print $2}')
echo $HF_USER
```

### First 60 Episodes
The first 60 episodes were collected with the cube in any of 6 grid sections in the workspace. The cube's position changed varied but the orientation was fixed.

```
lerobot-record \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=matt_follower_arm \
    --robot.cameras="{ wrist: {type: opencv, index_or_path: '/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0', width: 640, height: 480, fps: 30}, overhead: {type: opencv, index_or_path: '/dev/v4l/by-id/usb-Generic_NexiGo_N660P_FHD_Webcam_200901010001-video-index0', width: 640, height: 480, fps: 30}}" \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=matt_leader_arm \
    --display_data=true \
    --dataset.repo_id=${HF_USER}/blue_inline_1 \
    --dataset.num_episodes=10 \
    --dataset.single_task="pick and place blue block" \
    --dataset.streaming_encoding=true \
    --dataset.vcodec=auto \
    --dataset.push_to_hub=False
```

An example of the camera feeds from 1 episode is shown below:
<video src="/images/SO101_first_episodes.mov" autoplay muted loop playsinline style="max-width:100%; height:auto;"></video>

These datasets were then merged into one for training:
```
lerobot-edit-dataset \
    --new_repo_id ${HF_USER}/blue_inline_merged \
    --operation.type merge \
    --operation.repo_ids "['${HF_USER}/blue_inline_1', '${HF_USER}/blue_inline_2', '${HF_USER}/blue_inline_3', '${HF_USER}/blue_inline_4', '${HF_USER}/blue_inline_5', '${HF_USER}/blue_inline_6']"
```

**Note:** To view loss graphs, get a Weights and Biases key and run `wandb login`.

I then trained my first policy with:
```
lerobot-train \
  --dataset.repo_id=${HF_USER}/blue_inline_merged \
  --policy.type=act \
  --output_dir=/home/matthew/lerobot_policies/outputs/train/blue_inline_act_so101_test \
  --job_name=blue_inline_act_so101_test \
  --policy.device=cuda \
  --wandb.enable=true \
  --policy.repo_id=${HF_USER}/blue_inline_act
```

#### Results after 60 Episodes
With the model trained, we can run inference with it as input:
```
lerobot-record  \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.cameras="{ wrist: {type: opencv, index_or_path: '/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0', width: 640, height: 480, fps: 30}, overhead: {type: opencv, index_or_path: '/dev/v4l/by-id/usb-Generic_NexiGo_N660P_FHD_Webcam_200901010001-video-index0', width: 640, height: 480, fps: 30}}" \
  --robot.id=matt_follower_arm \
  --display_data=true \
  --dataset.repo_id=${HF_USER}/eval_blue_inline_act_so101 \
  --dataset.num_episodes=10 \
  --dataset.single_task="pick and place blue block" \
  --dataset.streaming_encoding=true \
  --dataset.vcodec=auto \
  --dataset.push_to_hub=False \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=matt_leader_arm \
  --policy.path=${policy_location}/blue_inline_act_so101_test/checkpoints/last/pretrained_model
```

With 60 episodes trained, the robot could successfully pick and place the cube when it was in grid locations 1-3, however 4-6 were problematic. The robot would often over-extend and miss the cube in grid sections 4-6, which were closer to the robot base. Any variance in the cube orientation in any grid location also caused failures in grasping. Out of distribution cube locations resulted in 100% grasping failure.

**Add an example here showing failures modes with this policy**.

### Adding 30 more episodes, including rotations and changing how we did teleop to be smoother and loop at the cube more

### Training with image transforms to counter lighting changes.

### Upping episode count to 160 or so

### Adding 40 episodes with the lamp turned off to counter shadows

### Add 50 more episodes where the container is in a different location each time, 25 lamp on and 25 lamp off