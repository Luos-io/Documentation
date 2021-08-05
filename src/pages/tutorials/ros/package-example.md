# ROS package example: Bike sharing example

This is a Luos example using ROS 2, the bike sharing application, that works this way:

* The bike pops up in steady green when it is idle
* Shake the Imu when it is idle, meaning the bike is being stolen, the RGB alarm then flashes in red
* Press the state button to acknowledge the alarm
* Press again to start riding, the bike slightly blinks in green
* Press again to stop riding, it becomes idle again

It relies on the 3D vizualizer embedded in ROS 2, named `RViz 2`:

![Bike sharing example](https://raw.githubusercontent.com/aubrune/luos_bike_alarm_example/master/doc/img/rviz.png)

## Test the example

This example package relies on `luos_interface`, make sure you first installed it by following its own procedure.

Then, download the example package and build your workspace with `colcon`:
```bash
~/ros2_ws/src/$ git clone https://github.com/aubrune/luos_bike_alarm_example.git

~/ros2_ws/$ colcon build --symlink-install    # Build the ROS workspace
~/ros2_ws/$ source ~/.bashrc                  # Source all new launches messages and resources
```

Plug at least a Luos Imu node and a Gate to your computer, as well as optional RGB and State services. The expected Luos services' aliases are the default. If they are not, update the <a href="https://github.com/aubrune/luos_bike_alarm_example/blob/master/luos_bike_alarm_example/bike_alarm.py#L12-L15" target="_blank">topic names</a> with your custom aliases.

Then, start the bike example from its launchfile:
```bash
~/ros2_ws/$ ros2 launch luos_bike_alarm_example example.launch.py
```

`RViz2` will pop up and show a 3D bike. Shake the Luos Imu node in order to update the RViz2 view in real time. If the bike is displayed but does not actuate, make sure that Imu data comes from the expected topic `/Imu_mod/imu`, or try to change the topic name.


# Package example
