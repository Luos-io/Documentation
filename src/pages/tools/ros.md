<h1><a href="#ros" class="header" id="ros"><img src="../../_assets/img/ros-logo.png" width="80px"> / Using Luos with ROS1 and ROS2</a></h1>

Luos comes with a package for the <a href="https://www.ros.org/" target="_blank">Robot Operating System</a>.

ROS 2 is the default version but backward compatibility with ROS 1 is ensured via the official ROS 1 bridge. You can get an example of an application using Luos services in ROS 2 with the <a href="https://github.com/aubrune/luos_bike_alarm_example" target="_blank">bike sharing example</a>.

In this tutorial, we will assume you're using ROS 2. If you want to communicate with a ROS 1 ecosystem, begin with this quickstart since ROS 2 needs to be installed, and then refer to the [Retrocompatibility with ROS 1 tutorial](../tutorials/tutorials.md).

## Basics: a few ROS-applied-to-Luos concepts

Here is a summary of core concepts in ROS:
* **ROS workspace**: this is the directory in which you download all the ROS software that is built from sourcecode. You will likely setup your own projects here but it is also common to download dependencies here, such as `luos_ros2`. Here we assume your workspace is `~/ros2_ws`
* **ROS node**: this is the name given to a single program or script being executed
* **ROS launchfile**: this is a file that launches multiple nodes at once.
Several nodes communicate with each other exchanging information in the form of structured pieces of data called **messages**, being exchanged through named communication channels called **topics**. Just like the filesystem of your hardrive, topics are hierarchised. E.g. `/something/into/something/else/data`
* **ROS package**: a package is a folder that contains resources such as nodes, launchfiles, messages, ... Here we have 2 packages named `luos_msgs` and `luos_interface`.

With Luos, data coming from/to Luos services are messages sent on topics. Some messages representing popular types of data comply to existing message formats, e.g. `sensor_msgs/Imu` representing IMU data: this message is provided by the `sensor_msgs` package. Some other messages are Luos-specific and are provided by the `luos_msgs` package.

There is a particular node whose role is to connect the Luos ecosystem with the ROS ecosystem: the **broker**, provided by the `luos_interface` package. The broker is strongly associated to a Luos gate since it connects to one (and only one) gate, and ensures the communication seamlessly. A broker can be launched with the command `ros2 launch luos_interface broker.launch.py`, which then attempts to connect to a serial Luos gate.

With Luos, topics names all start with a prefix being the service's alias and end with suffixes:
* `.../read`: this topic is read-only and a `.../write` topic exists
* `.../write`: this topic is write-only and a `.../read` topic exists. It is often a boolean type that (de)activates some source of data publishing
* no suffix: this topic is read-only and no other related topic exists

As an example, `/Imu_mod/variables/pedometer/read` allows to read-only the current pedometer variable from the IMU service named `Imu_mod`.

