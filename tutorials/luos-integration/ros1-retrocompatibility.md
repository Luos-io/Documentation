---
custom_edit_url: null
---

import IconExternalLink from '@theme/IconExternalLink';

# ROS 1 retro-compatibility with Luos tutorial

## Luos with ROS 1

The `ROS 1 Bridge` allows ROS 2 packages such as `luos_ros2` to communicate with a ROS 1 ecosystem. However, both ROS 1 and 2 need to be installed, and the bridge takes care of the translation.
This procedure has been tested with ROS 1 Noetic + ROS 2 Foxy and Python 3.8.2 in Ubuntu. It might work with older distributions, although it has not been tested.

### 1. Install ROS 2 and Luos

Make sure you have first installed ROS 2 and managed to run the broker in ROS 2 with the command `ros2 launch luos_interface broker.launch.py`.

We assume your ROS 2 workspace is `~/ros2_ws`.

### 2. Install ROS 1 and the ROS 1 bridge

Then install <a href="http://wiki.ros.org/noetic/Installation/Ubuntu" target="_blank">ROS 1 Noetic on Ubuntu 20.04<IconExternalLink width="10" /></a>.
We assume your ROS 1 workspace is `~/ros_ws`.

### 3. Initialize the bridge

The bridge has its own workspace that is to be compiled with colcon (ROS 2):

```bash
mkdir -p ~/ros1_bridge_ws/src
cd ~/ros1_bridge_ws/src
git clone -b foxy https://github.com/ros2/ros1_bridge.git
source ~/ros_ws/devel/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros1_bridge_ws
colcon build --packages-select ros1_bridge --cmake-force-configure --cmake-args -DBUILD_TESTING=FALSE
```

### 4. Start Luos in ROS 1

**In terminal 1: ROS Core in workspace `~/ros_ws`**

```bash
source ~/ros_ws/devel/setup.bash
roscore
```

**In terminal 2: ROS 1 Bridge in workspace `~/ros1_bridge_ws`**

```bash
source ~/ros1_bridge_ws/install/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics
```

**In terminal 3: Luos broker in workspace `~/ros2_ws`**
Plug some Luos services before starting the broker.

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch luos_interface broker.launch.py
```

**In terminal 1: Your ROS 1 app in workspace `~/ros_ws`**
Let us consider that `rostopic` is the ROS 1 app you want to run with Luos services.

```bash
source ~/ros_ws/devel/setup.bash
rostopic list
```

You can then publish and subscribe to the available topics in ROS 1.
