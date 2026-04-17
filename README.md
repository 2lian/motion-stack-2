# motion-stack-2

## Install

```bash
pixi install
```

Start the zenoh router (required for all examples):

```bash
pixi run router
```

## Examples

All examples use the Moonbot Zero (4 legs, 3 joints each, 12 joints total).
Communication uses [pyzeros](https://github.com/2lian/pyzeros2) -- real ROS 2
topics over zenoh, fully interoperable with rclpy, rviz, rosbag, etc.
No ROS installation required.

### Quick start: run from the terminal

The fastest way to get your robot running -- no Python code needed.
Compile the URDF to a file, then launch `lvl1_exec` with a JSON string:

```bash
# step 1: compile the Moonbot Zero URDF from its jinja template
pixi run python -c "from ms_moonbot_zero import load_moonbot_zero_urdf; \
  open('/tmp/mz.urdf','w').write(load_moonbot_zero_urdf())"

# step 2: run a single node with all joints
pixi run python -m ms_pyzeros_bridge.lvl1_exec \
  --ms-lvl1-json '{"urdf": "/tmp/mz.urdf"}'

# step 3 (other terminal): start the rerun visualizer
pixi run python -m ms_pyzeros_bridge.rerun_viz --urdf /tmp/mz.urdf
```

This works with any URDF, not just the Moonbot Zero. The `urdf` field accepts
either a file path or the URDF string directly. Mesh paths inside the URDF
must use absolute URIs (`file:///home/user/meshes/link.stl`) instead of
ROS package URIs (`package://my_robot/meshes/link.stl`), since there is no
`ament_index` to resolve packages at runtime.

### Single-process robot (`robot_demo.py`)

Runs the full robot in a single process. This is the low-level approach: you
assemble the `JointCore`, transport bridge, and simulation loop yourself in
Python. Use this when you need full control -- custom hardware drivers, custom
processing hooks, or a transport other than the defaults.

Shows how to:

- Compile a URDF from a jinja template with absolute mesh paths
- Create a `JointCore` with `Lvl1Param` and simulate joint responses via `LoopBack`
- Bridge the core to ROS 2 topics using `SubscriberHookJSB` / `PublisherHookJSB`
- Optionally start rerun visualization

```bash
pixi run mz_robot
pixi run mz_robot --rerun
```

Publishes joint state on `joint_read` and `/continuous_joint_read`.
Receives commands on `joint_set`.

The robot starts idle -- no motion until you send commands. You can use the
API client (see below), or interact directly with ROS 2 CLI tools:

```bash
# list available topics
ros2 topic list

# listen to joint state
ros2 topic echo /joint_read

# send a command to move joints (position in radians)
ros2 topic pub /joint_set sensor_msgs/msg/JointState \
  "{name: ['joint1_1', 'joint1_2'], position: [0.5, -0.3]}"
```

### Multi-process launcher (`mz_launcher.py`)

Spawns each leg as a separate process using the pre-built `lvl1_exec` node.
Unlike `robot_demo.py`, you don't write any core/transport code -- you just
configure `Lvl1Param` and let the existing executable handle the rest.
Use this when the defaults work and you just need to launch your robot.

Shows how to:

- Create one `Lvl1Param` per leg (same URDF, different namespace and end effector)
- Serialize params to JSON and pass them on the command line
- Use the default `lvl1_exec` executable provided by `ms_pyzeros_bridge`

```bash
# terminal 1: start the robot
pixi run mz_launcher

# terminal 2: rerun visualization
pixi run mz_launcher --viz
```

Each leg publishes on its own namespace (`leg1/joint_read`, `leg2/joint_read`, etc.).

### API client (`api_demo.py`)

Connects to a running robot and moves all joints. This is what user code
typically looks like: just subscribe to joint data and send
commands through the `AsyncJointSyncer` API.

Shows how to:

- Use `AsyncJointSyncer` to read and command joints
- Bridge the syncer to ROS 2 topics
- Wait for joints to come online, then move them with `syncer.lerp()`

```bash
# terminal 1: start the robot
pixi run mz_robot
# or: pixi run mz_launcher

# terminal 2: start the API client
pixi run mz_api
```

Works with both single-process (`mz_robot`) and multi-process (`mz_launcher`) mode
by subscribing to both root and `leg1`-`leg4` namespaces.
