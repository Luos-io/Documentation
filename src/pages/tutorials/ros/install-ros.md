# Install ROS

## Install ROS 2 and Luos

First install <a href="https://index.ros.org/doc/ros2/Installation/Foxy/" target="_blank">ROS 2 Foxy</a> for your OS with FastRTPS. Also install <a href="https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#install-colcon" target="_blank">`colcon`</a> as advised in the guidelines. If you are not familiar with ROS, you should go on with a couple of ROS 2 tutorials to get started.

Then clone `luos_ros2` to your workspace and compile it with colcon:

```bash
cd ros2_ws/src/    # Or replace by your ROS 2 workspace name, maybe also dev_ws/src
git clone https://github.com/aubrune/luos_ros2.git
cd luos_ros2/luos_interface
pip3 install --no-warn-script-location .
cd ~/ros2_ws && colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## Get started with Luos in ROS 2

Plug in your Luos gate and other services, such as IMU, Color or State, and run the broker. It will return the detected services:
```bash
~$ ros2 launch luos_interface broker.launch.py
[INFO] [luos_broker]: Connecting to /dev/ttyUSB0...
[INFO] [luos_broker]: Found services:
-------------------------------------------------
Type                Alias               ID
-------------------------------------------------
Gate                gate                1
Imu                 Imu_mod             2
State               button_mod          3
Color               rgb_led_mod         5
```

> **Note:** If you have several Luos gates, you need several brokers. Specify a port and a **unique** name for each of them:
> ```bash
> ros2 launch luos_interface broker.launch.py device:=/dev/ttyUSB1 name:=brokerUSB1
> ```

According to the services you have plugged-in, the broker will automatically publish the relevant topics in the namespace of your services' aliases.
Here we have plugged a `State` service (alias `button_mod`), a `Imu` service (alias `Imu_mod`) and a `Color` service (alias `rgb_led_mod`) to the gate; thus the broker publishes the following topics:
```bash
~$ ros2 topic list
/Imu_mod/acceleration
/Imu_mod/compass
/Imu_mod/imu
/Imu_mod/variables/gravity_vector/read
/Imu_mod/variables/gravity_vector/write
/Imu_mod/variables/heading/read
/Imu_mod/variables/heading/write
/Imu_mod/variables/pedometer/read
/Imu_mod/variables/pedometer/write
/Imu_mod/variables/walk_time/read
/Imu_mod/variables/walk_time/write
/button_mod/events/changed
/button_mod/events/pressed
/button_mod/events/released
/button_mod/variables/state/read
/button_mod/variables/state/write
/parameter_events
/rgb_led_mod/variables/color/read
/rgb_led_mod/variables/color/write
/rgb_led_mod/variables/time/read
/rgb_led_mod/variables/time/write
/rosout
```

Most variables advertise both `/read` and `/write` topics to get data, write data, or (de)activate this data source. Other types of data might be aggregates of Luos variables (such as the imu) or Luos events.

In order to echo messages from the terminal, use a regular ROS subscriber. For instance, here is the current IMU data:
```bash
~$ ros2 topic echo /Imu_mod/imu
header:
  stamp:
    sec: 1581267954
    nanosec: 5449
  frame_id: Imu_mod
orientation:
  x: 0.97814
  y: 0.052695
  z: -0.042831
  w: 0.196548
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: 0.177171
  y: 0.04968
  z: 0.176722
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```

In order to publish messages to the Luos services, use a regular ROS publisher. For instance, here is how to light up the Luos RGB service, in a pink color:
```bash
ros2 topic pub /rgb_led_mod/variables/color/write std_msgs/msg/ColorRGBA "{r: 64, g: 0, b: 64}" --once
```

## Get started with my own ROS2 package using Luos in Python

These command lines will create a new package `my_luos_ros2_package` relying on `luos_interface`:
```bash
cd ~/ros2_ws/src
ros2 pkg create my_luos_ros2_package --build-type ament_python --dependencies luos_interface
```
You can then add your ROS Python scripts, by taking example on the [bike sharing example](../tutorials/tutorials.md) page.


## Retro-compatibility ROS1

# Luos with ROS 1

The `ROS 1 Bridge` allows ROS 2 packages such as `luos_ros2` to communicate with a ROS 1 ecosystem. However, both ROS 1 and 2 need to be installed and the bridge takes care of the translation.
This procedure has been tested with ROS 1 Noetic + ROS 2 Foxy and Python 3.8.2 in Ubuntu. It might work with older distributions although it has not been tested.

## 1. Install ROS 2 and Luos

Make sure you have first [installed ROS 2](../ros.md) and managed to run the broker in ROS 2 with the command `ros2 launch luos_interface broker.launch.py`.
We assume your ROS 2 workspace is `~/ros2_ws`.

## 2. Install ROS 1 and the ROS 1 bridge

Then install <a href="http://wiki.ros.org/noetic/Installation/Ubuntu" target="_blank">ROS 1 Noetic on Ubuntu 20.04</a>.
We assume your ROS 1 workspace is `~/ros_ws`.

## 3. Initialize the bridge

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

## 4. Start Luos in ROS 1
### In terminal 1: ROS Core in workspace `~/ros_ws`
```bash
source ~/ros_ws/devel/setup.bash
roscore
```

### In terminal 2: ROS 1 Bridge in workspace `~/ros1_bridge_ws`
```bash
source ~/ros1_bridge_ws/install/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics
```

### In terminal 3: Luos broker in workspace `~/ros2_ws`
Plug some Luos services before starting the broker.
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch luos_interface broker.launch.py
```

### In terminal 1: Your ROS 1 app in workspace `~/ros_ws`
Let us consider here that `rostopic` is the ROS 1 app you want to run with Luos services.
```bash
source ~/ros_ws/devel/setup.bash
rostopic list
```
You can then publish and subscribe to the available topics in ROS 1.
