---
custom_edit_url: null
---

import IconExternalLink from '@theme/IconExternalLink';

# ROS 2 package example: Shared-bike example tutorial

This is a Luos example using ROS 2, the shared-bike application, that works this way:

- The bike's LED pops up in steady green when it is idle.
- Shake the IMU while it is idle, meaning the bike is being stolen; the RGB alarm then flashes in red.
- Press the state button to acknowledge the alarm.
- Press again to start riding the bike, the bike's LED slightly blinks in green.
- Press again when you stopped riding the bike, it becomes idle again.

It relies on the 3D visualizer embedded in ROS 2, named `RViz 2`:

![Shared-bike example](https://raw.githubusercontent.com/aubrune/luos_bike_alarm_example/master/doc/img/rviz.png)

## Test the example

This example package relies on `luos_interface`. Make sure you first installed it by following its own procedure.

Then, download the example package and build your workspace with `colcon`:

```bash
~/ros2_ws/src/$ git clone https://github.com/aubrune/luos_bike_alarm_example.git

~/ros2_ws/$ colcon build --symlink-install    # Build the ROS workspace
~/ros2_ws/$ source ~/.bashrc                  # Source all new launches messages and resources
```

Plug at least a Luos Imu node and a gate to your computer, as well as optional RGB and State services. The expected Luos services' aliases are the default. If they are not, update the <a href="https://github.com/aubrune/luos_bike_alarm_example/blob/master/luos_bike_alarm_example/bike_alarm.py#L12-L15" target="_blank">topic names<IconExternalLink width="10" /></a> with your custom aliases.

Then, start the bike example from its launchfile:

```bash
~/ros2_ws/$ ros2 launch luos_bike_alarm_example example.launch.py
```

`RViz2` will pop up and show a 3D bike. Shake the Luos Imu node in order to update the RViz2 view in real-time. If the bike is displayed but does not actuate, make sure that Imu data comes from the expected topic `/Imu_mod/imu`, or try to change the topic name.
