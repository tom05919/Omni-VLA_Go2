# OmniVLA-edge on the Unitree Go2 (Real Robot)

> **Full stop-judge + OmniVLA stack (edge / full / remote):** see
> [`../../README.md`](../../README.md) — everyday entry is
> `cd goal_stop_judge && python go2_nav.py`.
>
> This document is the **edge-oriented** driver / topic / launch deep dive.
> Canonical SDK path: `real_robot_SDKs/unofficial_sdk_unitree_go_2/src` then
> `source install/setup.bash`. `ROBOT_IP` is machine-specific (examples below
> may differ from [`RUNNING_GO2_SDK.md`](../../real_robot_SDKs/unofficial_sdk_unitree_go_2/RUNNING_GO2_SDK.md)).

This guide walks through **every command** needed to run OmniVLA-edge on a physical Unitree Go2 using the unofficial Go2 ROS 2 SDK. It covers one-time setup, the launch sequence for each session, verification steps, configuration, and troubleshooting.

---

## Table of contents

1. [System overview](#1-system-overview)
2. [Prerequisites](#2-prerequisites)
3. [One-time setup](#3-one-time-setup)
4. [Every session: command sequence](#4-every-session-command-sequence)
5. [Launch flags explained](#5-launch-flags-explained)
6. [Configure goals and behavior](#6-configure-goals-and-behavior)
7. [Safety and e-stop](#7-safety-and-e-stop)
8. [Optional: test with keyboard teleop first](#8-optional-test-with-keyboard-teleop-first)
9. [Troubleshooting](#9-troubleshooting)
10. [Known limitations of the current code](#10-known-limitations-of-the-current-code)

---

## 1. System overview

OmniVLA-edge runs as a ROS 2 node that:

- **Subscribes** to the Go2 front camera: `/camera/image_raw`
- **Publishes** velocity commands to `/cmd_vel`

The Go2 SDK launch file starts `twist_mux`, which forwards `/cmd_vel` to `/cmd_vel_out`. The driver node listens on `/cmd_vel_out` and sends commands to the robot over WebRTC.

```
┌─────────────────────────────────────────────────────────────────────┐
│  Terminal 2: python inference/run_omnivla_edge.py                   │
│                                                                     │
│  IsaacSimPublisher                                                  │
│    subscribe  /camera/image_raw  ◄──  go2_driver_node (WebRTC)      │
│    publish    /cmd_vel           ──►                              │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│  Terminal 1: ros2 launch go2_robot_sdk robot.launch.py ...          │
│                                                                     │
│  twist_mux:  /cmd_vel  ──►  /cmd_vel_out                            │
│  go2_driver_node:  /cmd_vel_out  ──►  WebRTC  ──►  Unitree Go2      │
└─────────────────────────────────────────────────────────────────────┘
```

**Important:** With the current `isaacsim_controller.py`, you **must** keep `teleop:=true` (the default) so `twist_mux` is running. If you later change the controller to publish directly to `/cmd_vel_out`, use `teleop:=false` instead (see [Launch flags](#5-launch-flags-explained)).

---

## 2. Prerequisites

### Hardware

- Unitree Go2 on the same network as your computer
- Robot IP reachable (default in this workspace: `192.168.10.3`)
- NVIDIA GPU recommended for inference (CPU fallback is very slow)

### Software (this workspace layout)

| Component | Path |
|-----------|------|
| OmniVLA repo | `/workspace/workspace/omni-VLA/OmniVLA` |
| Go2 SDK workspace | `/workspace/workspace/real_robot_SDKs/unofficial_sdk_unitree_go_2` |
| Go2 SDK setup doc | `real_robot_SDKs/unofficial_sdk_unitree_go_2/RUNNING_GO2_SDK.md` |

### Conda environments

You need **one Python environment** that has **both**:

1. OmniVLA dependencies (PyTorch, etc.) — see [SETUP.md](SETUP.md)
2. ROS 2 Humble Python packages (`rclpy`, `geometry_msgs`, `sensor_msgs`)

In this workspace we use the RoboStack conda env **`sim`** for ROS and run inference from the same env after installing OmniVLA:

```bash
conda activate sim
cd /workspace/workspace/omni-VLA/OmniVLA
pip install -e .
```

If you use a separate `omnivla` conda env, that env must also have ROS 2 available (e.g. via RoboStack). The inference script imports `rclpy` and will fail without it.

### Model checkpoint

Download OmniVLA-edge weights (one-time):

```bash
cd /workspace/workspace/omni-VLA/OmniVLA
git clone https://huggingface.co/NHirose/omnivla-edge
```

Expected file:

```
OmniVLA/omnivla-edge/omnivla-edge.pth
```

### Go2 SDK built

The Go2 SDK must be built at least once. See [One-time setup → Go2 SDK](#32-go2-sdk-one-time-build) or the full guide in `RUNNING_GO2_SDK.md`.

---

## 3. One-time setup

### 3.1 OmniVLA Python environment

```bash
# If starting fresh, follow SETUP.md for the omnivla env.
# For this workspace, we use the existing RoboStack env:
conda activate sim

cd /workspace/workspace/omni-VLA/OmniVLA
pip install -e .
```

Confirm the checkpoint exists:

```bash
ls -lh ./omnivla-edge/omnivla-edge.pth
```

### 3.2 Go2 SDK (one-time build)

```bash
conda activate sim

cd /workspace/workspace/real_robot_SDKs/unofficial_sdk_unitree_go_2

# Clean build if copied from another machine or after env changes
rm -rf build install log

colcon build --symlink-install
```

Install ROS dependencies via **mamba** (not `apt`) if packages are missing:

```bash
mamba install -c conda-forge -c robostack-humble \
  ros-humble-foxglove-bridge \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-twist-mux \
  ros-humble-teleop-twist-keyboard
```

Pin setuptools for colcon Python builds:

```bash
pip install "setuptools<80" "packaging<25"
```

After a successful build, verify:

```bash
source install/setup.bash
ros2 pkg list | grep go2
# Expected: go2_interfaces, go2_robot_sdk
```

---

## 4. Every session: command sequence

Use **three terminals** (or two if you skip verification). Run steps in order.

### Step 0 — Pre-flight: kill stray command publishers

Before starting the robot, ensure no leftover debug processes are publishing velocity:

```bash
ps aux | grep -E "ros2 topic pub|teleop_twist" | grep -v grep
```

If you see any `ros2 topic pub /cmd_vel` or `/cmd_vel_out` processes from old sessions, kill them:

```bash
kill <PID>
```

---

### Step 1 — Terminal 1: Start the Go2 SDK driver

```bash
conda activate sim

cd /workspace/workspace/real_robot_SDKs/unofficial_sdk_unitree_go_2
source install/setup.bash

export ROBOT_IP="192.168.10.3"    # change to your robot's IP
export CONN_TYPE="webrtc"

ros2 launch go2_robot_sdk robot.launch.py nav2:=false slam:=false joystick:=false
```

**Wait until you see:**

```
Robot 0 validated and ready
```

The robot may **stand up automatically** on connect (`BalanceStand` is sent after WebRTC validation). That is normal posture motion, not OmniVLA driving.

**Do not proceed** if the launch terminal spamming:

```
ERROR: Error in async send command:
WARNING: Data channel is not open
```

Rebuild/restart the driver with the patched SDK (see [Troubleshooting](#9-troubleshooting)).

---

### Step 2 — Terminal 2: Verify ROS topics and nodes

```bash
conda activate sim
cd /workspace/workspace/real_robot_SDKs/unofficial_sdk_unitree_go_2
source install/setup.bash
```

Check nodes:

```bash
ros2 node list | grep -E "go2_driver|twist_mux"
```

Expected:

```
/go2_driver_node
/twist_mux
```

Check the command pipeline:

```bash
ros2 topic info /cmd_vel -v
ros2 topic info /cmd_vel_out -v
ros2 topic info /camera/image_raw -v
```

Expected:

| Topic | Publisher | Subscriber |
|-------|-----------|------------|
| `/cmd_vel` | *(none until OmniVLA starts)* | `twist_mux` |
| `/cmd_vel_out` | `twist_mux` | `go2_driver_node` |
| `/camera/image_raw` | `go2_driver_node` | *(none until OmniVLA starts)* |

Confirm camera frames are flowing:

```bash
ros2 topic hz /camera/image_raw
# Should show a non-zero rate (typically ~15–30 Hz)
```

Confirm **no unexpected publishers** on `/cmd_vel_out`:

```bash
ros2 topic info /cmd_vel_out -v | grep -A2 "Publisher"
```

Only `twist_mux` should publish there before OmniVLA runs.

---

### Step 3 — Terminal 3: Run OmniVLA-edge inference

```bash
conda activate sim

# Source Go2 workspace so ROS message types resolve consistently
cd /workspace/workspace/real_robot_SDKs/unofficial_sdk_unitree_go_2
source install/setup.bash

cd /workspace/workspace/omni-VLA/OmniVLA
python inference/run_omnivla_edge.py
```

On startup you should see:

```
Using device: cuda:0
Loading model from ./omnivla-edge/omnivla-edge.pth
[FAILSAFE] Press Enter or 'q' + Enter to stop the robot.
```

Each inference tick prints velocities, e.g.:

```
linear angular 0.12 -0.05
```

Output figures are saved under `./inference/` (e.g. `1_ex_omnivla_edge.jpg`).

---

### Step 4 — Stop the robot and shut down

**To stop OmniVLA during a run:**

- Press **Enter** or type **`q`** + Enter in the OmniVLA terminal (e-stop thread).

**Full shutdown (always do this in order):**

1. `Ctrl+C` in the OmniVLA terminal (Terminal 3)
2. `Ctrl+C` in the Go2 launch terminal (Terminal 1)

Verify nothing is still publishing:

```bash
ros2 topic info /cmd_vel_out -v
```

---

## 5. Launch flags explained

Recommended launch for OmniVLA with the **current** `isaacsim_controller.py`:

```bash
ros2 launch go2_robot_sdk robot.launch.py nav2:=false slam:=false joystick:=false
```

| Flag | Value | Why |
|------|-------|-----|
| `nav2` | `false` | Prevents Nav2 `velocity_smoother` from flooding `/cmd_vel` with zeros |
| `slam` | `false` | Not needed for policy control |
| `joystick` | `false` | Prevents gamepad teleop from competing on `/cmd_vel` |
| `teleop` | `true` *(default)* | **Required** — starts `twist_mux` so `/cmd_vel` reaches `/cmd_vel_out` |

Optional flags to reduce overhead:

```bash
ros2 launch go2_robot_sdk robot.launch.py \
  nav2:=false slam:=false joystick:=false \
  rviz2:=false foxglove:=false
```

### Alternative layout (if you change `isaacsim_controller.py`)

If you update `isaacsim_controller.py` to publish directly to `/cmd_vel_out`:

```python
self.pub = self.create_publisher(Twist, '/cmd_vel_out', 10)
```

Then launch with:

```bash
ros2 launch go2_robot_sdk robot.launch.py nav2:=false slam:=false teleop:=false joystick:=false
```

---

## 6. Configure goals and behavior

Edit `inference/run_omnivla_edge.py` before running.

### Goal modality (line ~454)

```python
pose_goal = False
satellite = False
image_goal = False
lan_prompt = True   # language instruction goal (default)
```

### Language goal (line ~465)

```python
lan_inst_prompt = "move to the door"
```

### Image goal (line ~473)

```python
goal_image_PIL = Image.open("./inference/goal_img.jpg").convert("RGB").resize(imgsize)
```

Replace `goal_img.jpg` with your target scene image.

### GPS goal (lines ~467–469)

```python
goal_lat, goal_lon, goal_compass = 37.8738930785863, -122.26746181032362, 0.0
```

**Note:** The current test loop overwrites GPS-derived goals with hardcoded values around line 168–175. Comment that block out if you want live GPS-based navigation.

### Inference rate and run length

In class `Inference`:

```python
self.tick_rate = 3   # Hz — one model inference every ~333 ms
```

In `run()`:

```python
def run(self, max_ticks=80):   # total ticks before auto-stop
```

### Velocity scaling

In `inference/isaacsim_controller.py`:

```python
def publish_velocity(self, linear, angular, isaac_scale=1.0 / 0.6):
```

The PD controller in `run_omnivla_edge.py` caps velocities at ~0.3 m/s and ~0.3 rad/s. `isaac_scale` maps those into the range expected on the Go2. **Tune this on hardware** if motion is too fast or too slow.

---

## 7. Safety and e-stop

1. **Clear the area** around the robot before launching.
2. **Pre-flight check** — no stray `ros2 topic pub` processes (Step 0).
3. **OmniVLA e-stop** — Enter or `q`+Enter in the inference terminal.
4. **Emergency** — `Ctrl+C` the Go2 launch terminal to cut the driver connection.

**Known issue:** `IsaacSimPublisher.stop()` publishes zero velocity to `/cmd_vel`, but the Go2 driver's `handle_cmd_vel()` **ignores all-zero commands**. E-stop may not physically stop the robot on hardware until you `Ctrl+C` the driver or send a non-zero then zero sequence via a future fix. Treat `Ctrl+C` on the launch terminal as the reliable hard stop.

---

## 8. Optional: test with keyboard teleop first

Before running OmniVLA, confirm the command pipeline works manually.

**Terminal 1** — same Go2 launch as above.

**Terminal 2** — keyboard teleop:

```bash
conda activate sim
cd /workspace/workspace/real_robot_SDKs/unofficial_sdk_unitree_go_2
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Click the teleop terminal and tap **`i`** repeatedly (one command per keypress). The robot should move forward.

If that works but OmniVLA does not, the issue is in inference or velocity scaling — not the SDK bridge.

---

## 9. Troubleshooting

### No camera images in OmniVLA (`No image received from Go2 camera yet`)

```bash
ros2 topic hz /camera/image_raw
```

- If rate is 0: WebRTC video may not be connected — check launch logs for H264 decode warnings (often harmless if rate > 0).
- Ensure both terminals sourced `install/setup.bash`.
- Camera topic must match: `/camera/image_raw` (default in `IsaacSimPublisher`).

### OmniVLA runs but robot does not move

1. Check `/cmd_vel` is being published:

   ```bash
   ros2 topic echo /cmd_vel
   ```

2. Check `/cmd_vel_out` receives it:

   ```bash
   ros2 topic echo /cmd_vel_out
   ```

3. Confirm `twist_mux` is running:

   ```bash
   ros2 node list | grep twist_mux
   ```

4. Check launch terminal for WebRTC send errors (see Step 1).

5. Check for competing publishers:

   ```bash
   ros2 topic info /cmd_vel_out -v
   ```

### `twist_mux` not running / crashed on startup

Error:

```
parameter 'topics.navigation.timeout' has invalid type: expected [double] got [integer]
```

Fix in Go2 SDK `config/twist_mux.yaml`: use `timeout: 2.0` (float), not `2`. Rebuild:

```bash
cd /workspace/workspace/real_robot_SDKs/unofficial_sdk_unitree_go_2
colcon build --packages-select go2_robot_sdk --symlink-install
source install/setup.bash
```

### `Package 'go2_robot_sdk' not found`

You forgot to source the workspace overlay:

```bash
source /workspace/workspace/real_robot_SDKs/unofficial_sdk_unitree_go_2/install/setup.bash
```

### Robot moves on its own without commands

Check for stale debug publishers:

```bash
ps aux | grep "ros2 topic pub"
```

Kill any leftover processes. Only OmniVLA or teleop should publish velocity during operation.

### `Model weights not found at ./omnivla-edge/omnivla-edge.pth`

```bash
cd /workspace/workspace/omni-VLA/OmniVLA
git clone https://huggingface.co/NHirose/omnivla-edge
```

Run inference from the `OmniVLA/` directory so relative paths resolve.

### `ImportError: No module named 'rclpy'`

Your inference conda env does not have ROS 2. Use the `sim` RoboStack env or install ROS 2 into your OmniVLA env.

### RTPS / shared-memory warnings

```
RTPS_TRANSPORT_SHM Error: Failed init_port ...
```

Harmless in Docker/containers. DDS falls back to UDP.

---

## 10. Known limitations of the current code

| Item | Status |
|------|--------|
| Publishes to `/cmd_vel` (needs `twist_mux`) | Current behavior |
| `stop()` / zero velocity on real Go2 | **Does not stop robot** — driver ignores zeros |
| Publish rate ~3 Hz | May feel jerky; consider republishing between ticks |
| `isaac_scale` | Tuned for Isaac Sim; **retune on real hardware** |
| GPS goal in `run_omnivla_edge.py` | Overwritten by hardcoded test values |
| Auto `BalanceStand` on connect | Robot stands up when driver connects |

---

## Quick reference (copy-paste)

### Terminal 1 — Go2 driver

```bash
conda activate sim
cd /workspace/workspace/real_robot_SDKs/unofficial_sdk_unitree_go_2
source install/setup.bash
export ROBOT_IP="192.168.10.3"
export CONN_TYPE="webrtc"
ros2 launch go2_robot_sdk robot.launch.py nav2:=false slam:=false joystick:=false
```

### Terminal 2 — Verify (optional)

```bash
conda activate sim
source /workspace/workspace/real_robot_SDKs/unofficial_sdk_unitree_go_2/install/setup.bash
ros2 topic hz /camera/image_raw
ros2 topic info /cmd_vel_out -v
```

### Terminal 3 — OmniVLA-edge

```bash
conda activate sim
source /workspace/workspace/real_robot_SDKs/unofficial_sdk_unitree_go_2/install/setup.bash
cd /workspace/workspace/omni-VLA/OmniVLA
python inference/run_omnivla_edge.py
```

---

## Related documentation

- [SETUP.md](SETUP.md) — OmniVLA conda / PyTorch install
- [README.md](README.md) — model overview and training
- [inference/isaacsim_controller.py](inference/isaacsim_controller.py) — ROS pub/sub bridge
- [inference/run_omnivla_edge.py](inference/run_omnivla_edge.py) — inference loop
- [Go2 SDK RUNNING_GO2_SDK.md](../../real_robot_SDKs/unofficial_sdk_unitree_go_2/RUNNING_GO2_SDK.md) — SDK build and debug
