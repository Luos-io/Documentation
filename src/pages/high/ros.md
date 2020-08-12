# Luos with ROS1 and ROS2

Luos comes with a package for the [Robot Operating System](https://www.ros.org/).

ROS 2 is the default version but backward compatibility with ROS 1 is ensured via the official ROS 1 bridge. You can get an example of an application using Luos modules in ROS 2 with the [bike sharing example](https://github.com/aubrune/luos_bike_alarm_example).

In this tutorial, we will assume you're using ROS 2. If you want to communicate with a ROS 1 ecosystem, follow this quickstart anyway since ROS 2 needs to be installed and then refer to the [Retrocompatibility with ROS 1](./ros/ros1.md) page.

## A few ROS-applied-to-Luos concepts

Here is a summary of core concepts in ROS:
* **ROS workspace**: this is the directory in which you download all ROS software that is built from sourcecode. You will likely setup your own projects there but it is also common to download dependencies there, such as `luos_ros2`. Here we assume your workspace is `~/ros2_ws`
* **ROS node**: this is the name given to a single program or script being executed
* **ROS launchfile**: this is a file that launches multiple nodes at once
several nodes communicate with each other exchanging information in the form of structured pieces of data called **messages**, being exchanged through named communication channels called **topics**. Just like the filesystem of your hardrive, topics are hierarchised. E.g. `/something/into/something/else/data`
* **ROS package**: a package is a folder that contains resources such as nodes, launchfiles, messages, ... here we have 2 packages named `luos_msgs` and `luos_interface`

With Luos, data coming from/to Luos modules are messages sent on topics. Some messages representing popular types of data comply to existing message formats, e.g. `sensor_msgs/Imu` representing IMU data: this message is provided by the `sensor_msgs` package. Some other messages are Luos-specific and are provided by the `luos_msgs` package.

There is a particular node whose role is to connect the Luos ecosystem with the ROS ecosystem: the **broker**, provided by the `luos_interface` package. The broker is strongly associated to a Luos gate since it connects to 1 (and only 1) gate, and ensures the communication seamlessly. A broker can be launched with the command `ros2 launch luos_interface broker.launch.py`, which then attempts to connect to a serial Luos gate.

With Luos, topics names all start with a prefix being the module's alias and end with suffixes:
* `.../read`: this topic is read-only and a `.../write` topic exists
* `.../write`: this topic is write-only and a `.../read` topic exists. It is often a boolean type that (de)activates some source of data publishing
* no suffix: this topic is read-only and no other related topic exists

As an example, `/Imu_mod/variables/pedometer/read` allows to read-only the current pedometer variable from the IMU module named `Imu_mod`.

Now you know the basics, let's put in practice...

## Install ROS 2 and Luos

First install [ROS 2 Foxy](https://index.ros.org/doc/ros2/Installation/Foxy/) for your OS with FastRTPS. Also install [`colcon`](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#install-colcon) as advised in the guidelines. If you are not familiar with ROS, you should maye go on with a couple of ROS 2 tutorials to get started.

Then clone `luos_ros2` to your workspace and compile it with colcon:

```
cd ros2_ws/src/    # Or replace by your ROS 2 workspace name, maybe also dev_ws/src
git clone https://github.com/aubrune/luos_ros2.git
cd luos_ros2/luos_interface
pip3 install --no-warn-script-location .
cd ~/ros2_ws && colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## Get started with Luos in ROS 2

Just plug in your Luos gate and other modules, such as Imu, Color or State, and run the broker. It will output the detected modules: 
```
~$ ros2 launch luos_interface broker.launch.py
[INFO] [luos_broker]: Connecting to /dev/ttyUSB0...
[INFO] [luos_broker]: Found modules:
-------------------------------------------------
Type                Alias               ID   
-------------------------------------------------
Gate                gate                1    
Imu                 Imu_mod             2    
State               button_mod          3    
Color               rgb_led_mod         5    
```

**Note:** If you have several Luos gates, you need several brokers. Specify a port and a **unique** name for each of them:
```
ros2 launch luos_interface broker.launch.py device:=/dev/ttyUSB1 name:=brokerUSB1
```

According the modules you've plugged-in, the broker will automatically publish the revelant topics in the namespace of your modules' aliases.
Here we've plugged a `State` module (alias `button_mod`), a `Imu` module (alias `Imu_mod`) and a `Color` module (alias `rgb_led_mod`) to the gate ; thus the broker publishes the following topics:
```
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
/button_mod/events/falling
/button_mod/events/rising
/button_mod/variables/state/read
/button_mod/variables/state/write
/parameter_events
/rgb_led_mod/variables/color/read
/rgb_led_mod/variables/color/write
/rgb_led_mod/variables/time/read
/rgb_led_mod/variables/time/write
/rosout
```

Most variables advertise both `/read` and `/write` topics, to get data, write data, or (de)activate this data source. Other types of data might be aggregates of Luos variables (such as the imu) or Luos events.

In order to echo messages from the terminal, use a regular ROS subscriber. For instance here's the current IMU data:
```
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

In order to publish messages to the Luos modules, use a regular ROS publisher. For instance, here's how to light up the Luos RGB module, in pink color:
```
ros2 topic pub /rgb_led_mod/variables/color/write std_msgs/msg/ColorRGBA "{r: 64, g: 0, b: 64}" --once
```

## Get started with my own ROS2 package using Luos in Python

This command lines will create a new package `my_luos_ros2_package` relying on `luos_interface`:
```
cd ~/ros2_ws/src
ros2 pkg create my_luos_ros2_package --build-type ament_python --dependencies luos_interface
```
You can then add your ROS Python scripts, by taking example on the [bike sharing example](./ros/bike_alarm.md).
