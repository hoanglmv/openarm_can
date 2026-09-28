# OpenArm RGB-D ACT Dataset Contract

This is the recording contract for the bimanual 16-DOF OpenArm and one chest
RGB-D camera.

## Episode schema

- One HDF5 file per episode.
- Nominal sampling frequency: 50 Hz.
- Normal episode duration: 15-25 seconds (`T=750..1250`).
- Joint order: left J1-J7, left gripper motor, right J1-J7, right gripper motor.
- All 16 position/action axes use the native motor angle in radians.

```text
episode_XX.hdf5
├── attributes
│   ├── sim              bool
│   ├── frequency_hz     50
│   ├── robot_type       "OpenArm_Bimanual_16DOF"
│   ├── num_joints       16
│   ├── depth_scale      0.001
│   └── depth_range_m    [0.2, 1.2]
├── observations
│   ├── images
│   │   ├── chest_rgb    uint8   [T, 480, 640, 3]
│   │   └── chest_depth  uint16  [T, 480, 640]
│   ├── qpos             float32 [T, 16]
│   ├── qvel             float32 [T, 16]
│   └── effort           float32 [T, 16]
├── action               float32 [T, 16]
└── timestamp_ns         int64   [T]
```

`effort` is recorded when CAN telemetry supplies motor torque. Depth is aligned
to color before recording. Zero remains the invalid-depth sentinel; other depth
values are clipped to 200-1200 mm. The recorder has no command-topic input, so
`action[t]` contains the observed absolute joint position `qpos[t]`.

## Runtime topics

```text
/camera/act/rgb             sensor_msgs/Image, rgb8, 424x240 @ 25 Hz
/camera/act/depth           sensor_msgs/Image, 16UC1, 424x240 @ 25 Hz
/openarm/joint_states       sensor_msgs/JointState (position, velocity, effort) @ 100 Hz
```

The RealSense wrapper publishes synchronized color and aligned depth. The RGB-D
preprocessor samples the source at 25 Hz. The backend publishes robot telemetry
at 100 Hz. The recorder only subscribes to the RGB, depth, and joint-state ROS 2
topics and samples their latest complete values at 50 Hz. Consequently each 25 Hz
RGB-D frame is normally repeated across two adjacent dataset timesteps, while the
joint state is downsampled from 100 Hz to 50 Hz. Since there is deliberately no
command subscription, `action[t]` stores the observed `qpos[t]`. The recorder
resizes the 424x240 RGB-D topic images to the dataset's 640x480 shape.

The default RealSense source profile is `424x240x30`; the preprocessor emits
bandwidth-limited `424x240` RGB-D at 25 Hz. Override the source profile for a camera
with different supported profiles:

```bash
export OPENARM_CAMERA_COLOR_PROFILE=640x480x30
export OPENARM_CAMERA_DEPTH_PROFILE=640x480x30
```

The 50 Hz dataset does not require a new camera exposure at every timestep.

## Recording

Start the dashboard/backend:

```bash
cd ~/openarm/openarm_can
source .venv-dashboard/bin/activate
python3 sim/server.py
```

In a second terminal, record one real-robot episode. Recording stops
automatically after 25 seconds, or earlier with `Ctrl+C`:

```bash
cd ~/openarm/openarm_can
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 sim/data_recorder.py --episode episode_0
```

For simulation, add `--sim`. A valid finalized episode is written to
`data_set/episode_0.hdf5`. Until finalization, it remains
`episode_0.partial.hdf5`.

Validate the result:

```bash
/usr/bin/python3 sim/validate_dataset.py data_set/episode_0.hdf5
```

Use `--allow-short` only for smoke-test recordings shorter than 15 seconds.

## ACT DataLoader contract

The DataLoader converts RGB to ImageNet-normalized float32, converts depth from
millimetres to metres, clips it to 0.2-1.2 m, normalizes depth to `[0,1]`, and
concatenates RGB plus depth into four channels.

```text
image    float32 [Batch, 1, 4, 480, 640]
qpos     float32 [Batch, 16]
actions  float32 [Batch, 50, 16]
is_pad   bool    [Batch, 50]
```

Action chunks are selected as `action[t:t+50]`; the recorder does not shift the
stored action array by one index.
