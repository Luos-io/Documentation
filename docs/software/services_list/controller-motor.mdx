# Controller-motor API

This service type allows to control a motor with a reduction and a sensor (usually called motor-reducer or speed-reducer).
This service computes PID for speed, position and motion planning.

You can find basic information about PID control here: <a href="https://medium.com/luosdeviceics/an-introduction-to-pid-control-with-dc-motor-1fa3b26ec661" target="_blank">An introduction to PID control with DC motor</a>, and a code example to tune your PID at the end of this page.

Its type has access to all common capabilities.

### services’s type settings:

> **Warning:** This service doesn't save any of the following parameters, they must be set each time your service reboots.

Before using your Controller-motor service, you have to setup the resolution, motor reduction and eventually the wheel size, if you plan to use translation modes. To check the configuration, just make a complete turn on the motor shaft with your hand and check if the rotation position value is OK.

Both PID’s values have to be set accordingly to the motor-reducer plugged to the board. Each different motor-reducer will have different PID’s values for position and speed control, and you have to define them by yourself.
The default values `[0, 0, 0]` won’t have any effect on the motor, and must be changed if you plan to use any position or speed control mode.
To setup your PID please refer to the example at the end of this page.

> **Warning:** PID for position and speed must be set in your code as an initialization before starting to use your service with position or speed control.

Now that everything is configured, you can enable the control modes you want to use. You can use position and speed mode simultaneously. Power mode is only usable alone. The Controller-motor is now ready to use, you can disable compliance to start moving the motor.

----

## Functions

| **Function name and parameters** | **Action** | **Comment** |
|:---:|:---:|:---:|
| setToZero(self) | Resets current position of the motor to 0 | You can use it to initialize the position of the motor |
| control(self) | Displays service's type graphical interface | Only available using Jupyter notebook |

## Variables

### Motor settings

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| positionPid | Sets position PID used for rotation position mode and translation position mode | read / write: \[float P, float I, float D\] |
| speedPid | Sets speed PID used for rotation speed mode and translation speed mode | read / write: \[float P, float I, float D\] |
| encoder_res | Defines the motor sensor resolution, in steps by rotation.<br/>*This service considers that the sensor is placed before the reduction. If it is not your case, just setup a reduction ratio of 1.* | read / write: float |
| reduction | Defines the motor reduction.<br/>Set this value to 1 if your sensor measures after the reduction. | read / write: float |
| wheel_size | Defines wheel size used for translation mode, in mm. | read / write: float |

### Motor control modes

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| compliant | - True: disables the motor driver, you can use it to move the motor by hand.<br/> - False: Enables the motor driver. | read / write: Boolean (True or False) |
| power_mode | Enables/Disables the power control mode.<br/>*Disables any other control mode if enabled.* | read / write: Boolean (True or False) |
| rot_position_mode | Enables/Disables the motor rotation position control mode.<br/>*Disables power mode and translation position mode if enabled.*<br/>*Doesn't work if no position PID is configured.* | read / write: Boolean (True or False) |
| rot_speed_mode | Enables/Disables the motor rotation speed control mode.<br/>*Disables power mode and translation speed mode if enabled.*<br/>*Doesn't work if no speed PID configured.* | read / write: Boolean (True or False) |
| trans_position_mode | Enables/Disables the motor translation position control mode.<br/>*Disables power mode and rotation position mode if enabled.*<br/>*Doesn't work if no position PID configured.* | read / write: Boolean (True or False) |
| trans_speed_mode | Enables/Disables the motor translation speed control mode.<br/>*Disables power mode and rotation speed mode if enabled.*<br/>*Doesn't work if no speed PID configured.* | read / write: Boolean (True or False) |

### Motor sensing

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| rot_position | Reads rotation position in °<br/>*Reading it auto-Enables actualization.* | read only: float |
| rot_position | Starts/Stops rotation position measurement actualization | write only: Boolean (True or False) |
| rot_speed | Reads rotation speed in °/s<br/>*Reading it auto-Enables actualization.* | read only: float |
| rot_speed | Starts/Stops rotation speed measurement actualization | write only: Boolean (True or False) |
| trans_position | Reads translation position in mm<br/>*Reading it auto-Enables actualization.* | read only: float |
| trans_position | Starts/Stops translation position measurement actualization | write only: Boolean (True or False) |
| trans_speed | Reads translation speed in mm/s<br/>*Reading it auto-enables actualization.* | read only: float |
| trans_speed | Starts/Stops translation speed measurement actualization | write only: Boolean (True or False) |
| current | Reads the current consumption in A<br/>*Reading it auto-enables actualization.* | read only: float |
| current | Starts/Stops current measurement actualization | write only: Boolean (True or False) |

### Motor commands

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| power_ratio | Sets the power quantity send to the motor between -100% and 100%. | read / write: float |
| target_rot_position | Sets the target rotation position to reach in °. | read / write: float |
| target_rot_speed | Sets the target rotation speed to reach in °/s. | read / write: float |
| target_trans_position | Sets the target translation position to reach in mm. | read / write: float |
| target_trans_speed | Sets the target translation speed to reach in mm/s. | read / write: float |

## ROS topics
| **Topic name** | **Message type** | **Comment** |
|:----|:---:|:---:|
| /mod/variables/positionPid/read | geometry_msgs/msg/Vector3 |
| /mod/variables/positionPid/write | geometry_msgs/msg/Vector3 |
| /mod/variables/speedPid/read | geometry_msgs/msg/Vector3 |
| /mod/variables/speedPid/write | geometry_msgs/msg/Vector3 |
| /mod/variables/encoder_res/read | std_msgs/msg/Float32 |
| /mod/variables/encoder_res/write | std_msgs/msg/Float32 |
| /mod/variables/reduction/read | std_msgs/msg/Float32 |
| /mod/variables/reduction/write | std_msgs/msg/Bool |
| /mod/variables/wheel_size/read | std_msgs/msg/Float32 |
| /mod/variables/wheel_size/write | std_msgs/msg/Float32 |
| /mod/variables/compliant/read | std_msgs/msg/Bool |
| /mod/variables/compliant/write | std_msgs/msg/Bool |
| /mod/variables/power_mode/read | std_msgs/msg/Bool |
| /mod/variables/power_mode/write | std_msgs/msg/Bool |
| /mod/variables/rot_position_mode/read | std_msgs/msg/Bool |
| /mod/variables/rot_position_mode/write | std_msgs/msg/Bool |
| /mod/variables/rot_speed_mode/read | std_msgs/msg/Bool |
| /mod/variables/rot_speed_mode/write | std_msgs/msg/Bool |
| /mod/variables/trans_position_mode/read | std_msgs/msg/Bool |
| /mod/variables/trans_position_mode/write | std_msgs/msg/Bool |
| /mod/variables/trans_speed_mode/read | std_msgs/msg/Bool |
| /mod/variables/trans_speed_mode/write | std_msgs/msg/Bool |
| /mod/variables/rot_position/read | std_msgs/msg/Float32 |
| /mod/variables/rot_position/write | std_msgs/msg/Bool |
| /mod/variables/rot_speed/read | std_msgs/msg/Float32 |
| /mod/variables/rot_speed/write | std_msgs/msg/Bool |
| /mod/variables/trans_position/read | std_msgs/msg/Float32 |
| /mod/variables/trans_position/write | std_msgs/msg/Bool |
| /mod/variables/trans_speed/read | std_msgs/msg/Float32 |
| /mod/variables/trans_speed/write | std_msgs/msg/Bool |
| /mod/variables/current/read | std_msgs/msg/Float32 |
| /mod/variables/current/write | std_msgs/msg/Bool |
| /mod/variables/power_ratio/read | std_msgs/msg/Float32 |
| /mod/variables/power_ratio/write | std_msgs/msg/Float32 |
| /mod/variables/target_rot_position/read | std_msgs/msg/Float32 | value in radians
| /mod/variables/target_rot_position/write | std_msgs/msg/Float32 | value in radians
| /mod/variables/target_rot_speed/read | std_msgs/msg/Float32 | value in radians
| /mod/variables/target_rot_speed/write | std_msgs/msg/Float32 | value in radians
| /mod/variables/target_trans_position/read | std_msgs/msg/Float32 |
| /mod/variables/target_trans_position/write | std_msgs/msg/Float32 |
| /mod/variables/target_trans_speed/read | std_msgs/msg/Float32 |
| /mod/variables/target_trans_speed/write | std_msgs/msg/Float32 |

## Example code

### PID Setting example code

The PID values allow your motor to stick to the target command as fast as possible. The quality of a set of PID values depends on time to reach the target position and position precision.
Tuning a PID is something difficult and takes a lot of practice. It's really important to have simple ways to evaluate PID values impact on your motor before starting to tune these values.
Here is the code we use at Luos to tune a PID by ourself. Use it with Jupyter notebook to get your plot instantly.

The main code:
```python
%matplotlib inline
from pyluos import Device
from IPython.display import clear_output
import time
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# 1. Connect your Luos network (here using an USB service for example)
r = Device('/dev/cu.usbserial-DN2AAOVK')
r.services

# 2. Select the service of your network you need to configure
service = r.controller_moto

# 3. Setup service basic settings
service.encoder_res = 48
service.reduction = 26.851

def run_speed_test(velocity_target):
    service.rot_position = False
    service.rot_speed = True
    service.rot_position_mode = False
    service.rot_speed_mode = True
    service.target_rot_speed = 0.0
    service.compliant = False
    target = []
    real = []
    test_time_vector = []
    test_start_time = time.time()
    target.append(service.target_rot_speed)
    real.append(service.rot_speed)
    test_time = time.time()
    test_time_vector.append(0.0)
    while (test_time < test_start_time + 0.5):
        target.append(service.target_rot_speed)
        real.append(service.rot_speed)
        test_time_vector.append(test_time - test_start_time)
        test_time = time.time()
    service.target_rot_speed = velocity_target
    while (test_time < test_start_time + 2.5):
        target.append(service.target_rot_speed)
        real.append(service.rot_speed)
        test_time_vector.append(test_time - test_start_time)
        test_time = time.time()
    service.compliant = True
    plot_test(test_time_vector, target, real)

def run_pos_test(pos_target):
    service.rot_speed = False
    service.rot_position = True
    service.rot_speed_mode = False
    service.rot_position_mode = True
    service.target_rot_position = 0.0
    service.compliant = False
    target = []
    real = []
    test_time_vector = []
    test_start_time = time.time()
    target.append(service.target_rot_position)
    real.append(service.rot_position)
    test_time = time.time()
    test_time_vector.append(0.0)
    while (test_time < test_start_time + 1):
        target.append(service.target_rot_position)
        real.append(service.rot_position)
        test_time_vector.append(test_time - test_start_time)
        test_time = time.time()
    service.target_rot_position = pos_target
    while (test_time < test_start_time + 2.5):
        target.append(service.target_rot_position)
        real.append(service.rot_position)
        test_time_vector.append(test_time - test_start_time)
        test_time = time.time()
    service.compliant = True
    plot_test(test_time_vector, target, real)

def plot_test(test_time_vector, target, real):
    fig = plt.figure()
    ax = plt.subplot(111)
    ax.plot(test_time_vector,target,'r')
    ax.plot(test_time_vector,real,'b')
    plt.show()
    plt.close(fig)
```

Now, you are ready to tune the PID values for position and speed control modes. To do that, you have to try values to get best the result possible. In order to succeed, we advise you to do it step by step:

 1. Set P, I, and D values to 0.
 2. Increase the P value until you have a small oscillation around the target.
 3. Increase the D value until you have a fast and stable position.
 4. Increase with really small figures the I value to improve the motor precision.

The code you can use to tune your speed PID:
```python
# Speed PID settings
service.speedPid = [0.1,0.1,0] # speed PID [P, I, D]
run_speed_test(100.0)
```

The code you can use to tune your position PID:
```python
# Position PID settings
service.positionPid = [4.0,0.02,100] # position PID [P, I, D]
run_pos_test(100.0)
```

### Example command from ROS topics

By publishing on 3 topics you will take control over the Controller-motor named `controller_moto` to a velocity command of 1.57 rad/s:

```bash
# Launch the broker. Note: warnings will be displayed, please ignore them
ros2 launch luos_interface broker.launch.py

# Start the command in velocity mode
ros2 topic pub -1 /controller_moto/variables/rot_speed_mode/write std_msgs/msg/Bool data:\ true\ 
ros2 topic pub -1 /controller_moto/variables/target_rot_speed/write std_msgs/msg/Float32 data:\ 90.0\ 
ros2 topic pub -1 /controller_moto/variables/compliant/write std_msgs/msg/Bool data:\ false\ 
```

Then publish `true` to the `/controller_moto/variables/compliant/write` topic to stop the driver.
