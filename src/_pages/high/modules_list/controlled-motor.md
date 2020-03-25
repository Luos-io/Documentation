# Controlled-motor module type

This module type allows to control a motor with a reduction and a sensor (usually called motor-reducer or speed-reducer).
This module computes PID for speed, position and motion planning.

You can find basic information about PID control here: <a href="https://medium.com/luosdeviceics/an-introduction-to-pid-control-with-dc-motor-1fa3b26ec661" target="_blank">An introduction to PID control with DC motor</a>, and a code example to tune your PID at the end of this page.

Its type has access to all common capabilities.

### Modules’s type settings:

> **Warning:** This module doesn't save any of the following parameters, they must be set each time your module reboots.

Before using your controlled motor module, you have to setup the resolution, motor reduction and eventually the wheel size, if you plan to use translation modes. To check the configuration, just make a complete turn on the motor shaft with your hand and check if the rotation position value is OK.

Both PID’s values have to be set accordingly to the motor-reducer plugged to the board. Each different motor-reducer will have different PID’s values for position and speed control, and you have to define them by yourself.
The default values `[0, 0, 0]` won’t have any effect on the motor, and must be changed if you plan to use any position or speed control mode.
To setup your PID please refer to the example at the end of this page.

> **Warning:** PID for position and speed must be set in your code as an initialization before starting to use your module with position or speed control.

Now that everything is configured, you can enable the control modes you want to use. You can use position and speed mode simultaneously. Power mode is only usable alone. The controlled motor is now ready to use, you can disable compliance to start moving the motor.

----

## Functions

| **Function name and parameters** | **Action** | **Comment** |
|:---:|:---:|:---:|
| setToZero(self) | Resets current position of the motor to 0 | You can use it to initialize the position of the motor |
| control(self) | Displays module type graphical interface | Only available using Jupyter notebook |

## Variables

### Motor settings

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| positionPid | Sets position PID used for rotation position mode and translation position mode | read / write: [float P, float I, float D] |
| speedPid | Sets speed PID used for rotation speed mode and translation speed mode | read / write: [float P, float I, float D] |
| encoder_res | Defines the motor sensor resolution, in steps by rotation.<br/>*This module considers that the sensor is placed before the reduction. If it's not your case, just setup a reduction ratio of 1.* | read / write: float |
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

## PID Setting example code

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

# 1. Connect your Luos network (here using an USB module for example)
r = Device('/dev/cu.usbserial-DN2AAOVK')
r.modules

# 2. Select the module of your network you need to configure
module = r.controlled_moto

# 3. Setup module basic settings
module.encoder_res = 48
module.reduction = 26.851

def run_speed_test(velocity_target):
    module.rot_position = False
    module.rot_speed = True
    module.rot_position_mode = False
    module.rot_speed_mode = True
    module.target_rot_speed = 0.0
    module.compliant = False
    target = []
    real = []
    test_time_vector = []
    test_start_time = time.time()
    target.append(module.target_rot_speed)
    real.append(module.rot_speed)
    test_time = time.time()
    test_time_vector.append(0.0)
    while (test_time < test_start_time + 0.5):
        target.append(module.target_rot_speed)
        real.append(module.rot_speed)
        test_time_vector.append(test_time - test_start_time)
        test_time = time.time()
    module.target_rot_speed = velocity_target
    while (test_time < test_start_time + 2.5):
        target.append(module.target_rot_speed)
        real.append(module.rot_speed)
        test_time_vector.append(test_time - test_start_time)
        test_time = time.time()
    module.compliant = True
    plot_test(test_time_vector, target, real)

def run_pos_test(pos_target):
    module.rot_speed = False
    module.rot_position = True
    module.rot_speed_mode = False
    module.rot_position_mode = True
    module.target_rot_position = 0.0
    module.compliant = False
    target = []
    real = []
    test_time_vector = []
    test_start_time = time.time()
    target.append(module.target_rot_position)
    real.append(module.rot_position)
    test_time = time.time()
    test_time_vector.append(0.0)
    while (test_time < test_start_time + 1):
        target.append(module.target_rot_position)
        real.append(module.rot_position)
        test_time_vector.append(test_time - test_start_time)
        test_time = time.time()
    module.target_rot_position = pos_target
    while (test_time < test_start_time + 2.5):
        target.append(module.target_rot_position)
        real.append(module.rot_position)
        test_time_vector.append(test_time - test_start_time)
        test_time = time.time()
    module.compliant = True
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
module.speedPid = [0.1,0.1,0] # speed PID [P, I, D]
run_speed_test(100.0)
```

The code you can use to tune your position PID:
```python
# Position PID settings
module.positionPid = [4.0,0.02,100] # position PID [P, I, D]
run_pos_test(100.0)
```

<div class="cust_edit_page"><a href="https://{{gh_path}}{{modules_path}}/controlled-motor.md">Edit this page</a></div>
