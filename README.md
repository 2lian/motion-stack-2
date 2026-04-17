# motion-stack-2

## Install

```bash
pixi install
pixi run router  # start zenoh router (required)
```

## Examples

All examples use the Moonbot Zero (4 legs, 3 joints each, 12 joints total).
Communication uses [pyzeros](https://github.com/2lian/pyzeros2) -- real ROS 2
topics over zenoh, fully interoperable with rclpy, rviz, rosbag, etc.
No ROS installation required.

### Quick start: run from the terminal

No Python code needed. Compile the URDF, then launch:

```bash
# compile the URDF
pixi run python -c "from ms_moonbot_zero import load_moonbot_zero_urdf; \
  open('/tmp/mz.urdf','w').write(load_moonbot_zero_urdf())"

# run the robot
pixi run python -m ms_pyzeros_bridge.lvl1_exec \
  --ms-lvl1-json '{"urdf": "/tmp/mz.urdf"}'

# (other terminal) visualize in rerun
pixi run python -m ms_pyzeros_bridge.rerun_viz --urdf /tmp/mz.urdf
```

Works with any URDF. The `urdf` field accepts a file path or string.

> [!NOTE]
> Mesh paths inside the URDF must be absolute URIs (`file:///...`), not
> ROS package URIs (`package://...`), since there is no `ament_index` at runtime.

### Single-process robot ([`robot_demo.py`](src/ms_moonbot_zero/ms_moonbot_zero/robot_demo.py))

Low-level approach: you wire up `JointCore`, transport, and simulation
yourself. Use this when you need custom hardware drivers, processing hooks,
or a different transport.

```bash
pixi run python -m ms_moonbot_zero.robot_demo
pixi run python -m ms_moonbot_zero.robot_demo --rerun
```

> [!NOTE]
> Shortcut: `pixi run mz_robot` / `pixi run mz_robot --rerun`

> [!TIP]
> The robot starts idle. Send commands with the API client below, or with ROS 2 CLI:
> 
> ```bash
> ros2 topic list
> ros2 topic echo /joint_read
> ros2 topic pub /joint_set sensor_msgs/msg/JointState \
>   "{name: ['joint1_1', 'joint1_2'], position: [0.5, -0.3]}"
> ```

### Multi-process launcher ([`mz_launcher.py`](src/ms_moonbot_zero/ms_moonbot_zero/mz_launcher.py))

Same idea as the terminal quick start, but automated with `Popen`: compile
the URDF in Python, build `Lvl1Param` per leg, and spawn one `lvl1_exec`
process each. Use this when the defaults work and you just need to configure
and launch.

```bash
pixi run python -m ms_moonbot_zero.mz_launcher
pixi run python -m ms_moonbot_zero.mz_launcher --viz
```

> [!NOTE]
> Shortcut: `pixi run mz_launcher` / `pixi run mz_launcher --viz`

Each leg publishes on its own namespace (`leg1/joint_read`, `leg2/joint_read`, etc.).

### API client ([`api_demo.py`](src/ms_moonbot_zero/ms_moonbot_zero/api_demo.py))

Connects to a running robot and moves all joints via `AsyncJointSyncer`.
This is what application code looks like.

```bash
# terminal 1: start the robot
pixi run python -m ms_moonbot_zero.robot_demo
# or: pixi run python -m ms_moonbot_zero.mz_launcher

# terminal 2: start the API client
pixi run python -m ms_moonbot_zero.api_demo
```

> [!NOTE]
> Shortcuts: `pixi run mz_robot`, `pixi run mz_launcher`, `pixi run mz_api`

Works with both single-process and multi-process mode.
