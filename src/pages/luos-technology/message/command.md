# Command and object dictionary

Each message includes a command value that defines the type of the contents of the message's data.

Another feature included in Luos technology is the [Object Dictionary (OD)](./command.md), which aims to maintain interoperability of data format and command type between <span class="cust_tooltip">services<span class="cust_tooltiptext">{{service_def}}</span></span>.

## Commands

Commands is a simple enum list from 0 to N allowing you to choose the data format you want to use on your message.
Internally Luos also use some messages to manage and detect your system, so Luos have some reserved commands at the begining of this list.
User command start at `LUOS_LAST_RESERVED_CMD`.

### Common register for all services

| Command | Function |
| :---: | :---: |
| GET_CMD | asks a service to publish its data |
| SET_CMD | set some undefined data |

### Generic data
| Command | Function |
| :---: | :---: |
| COLOR| color_t (R, G, B)|
| IO_STATE| char (True/False)|
| RATIO| ratio_t (percentage %)|
| PEDOMETER| long[2] (step number and step time millisecond)|
| ILLUMINANCE| illuminance_t (lx)|
| VOLTAGE| voltage_t (Volt)|
| CURRENT| current_t (Ampere)|
| POWER| power_t (Watt)|
| TEMPERATURE| temperature_t (°C)|
| TIME| time Second (float)|
| FORCE| force_t (Newton N)|
| MOMENT| moment_t (Newton meter N.m)|
| CONTROL| control_mode (control_mode_t)|

### Configuration commands
| Command | Function |
| :---: | :---: |
| REGISTER | a register data [reg_add, data[]] |
| REINIT | char (True/False) |
| PID | pid_t float[3] = {p, i, d} |
| RESOLUTION | resolution parameter for a sensor float |
| REDUCTION | reduction factor (mechanical for example) float |
| DIMENSION | dimention of an element m linear_position_t |
| OFFSET | decay float |
| SETID | Set Dynamixel ID |

### Space positioning
| Command | Function |
| :---: | :---: |
| ANGULAR_POSITION | angular_position_t (deg) |
| ANGULAR_SPEED | angular_speed_t (deg/s) |
| LINEAR_POSITION | linear_position_t (m) |
| LINEAR_SPEED | linear_speed_t (m/s) |
| ACCEL_3D | long[3](X, Y, Z axis linear acceleration data in Gees) |
| GYRO_3D | long[3](X, Y, Z axis rotational acceleration data in degrees per second) |
| QUATERNION | long[4] (sensor fused w, x, y, z rotational angles) |
| COMPASS_3D | long[3](magnetic field data in micro tesla on each axis) |
| EULER_3D | long[3](Pitch, roll, yaw based in degrees with frame reference) |
| ROT_MAT | short[9] (linear math 9 element matrix representation) |
| LINEAR_ACCEL | float[3] (linear acceleration in body frame coordinates) |
| GRAVITY_VECTOR | float[3] (Which access gravity effects) |
| HEADING | long (360 degrees from North with Y+ axis as the pointer) |

### Space positioning limits
| Command | Function |
| :---: | :---: |
| ANGULAR_POSITION_LIMIT | min angular_position_t (deg), max angular_position_t (deg) |
| LINEAR_POSITION_LIMIT | min linear_position_t (m), max linear_position_t (m) |
| RATIO_LIMIT | float(%) |
| CURRENT_LIMIT | float(A) |
| ANGULAR_SPEED_LIMIT | min angular_speed_t (deg/s), max angular_speed_t (deg/s) |
| LINEAR_SPEED_LIMIT | min linear_speed_t (m/s), max linear_speed_t (m/s) |
| TORQUE_LIMIT | max moment_t (Nm) |
| TEMPERATURE_LIMIT | Max temperature_t (°C) |

### Specific register
| Command | Function |
| :---: | :---: |
| PARAMETERS | depend on the service, can be : servo_parameters_t, imu_report_t, motor_mode_t|
| ERROR_CMD | |


You can find the complete list of commands <a href="https://github.com/Luos-io/Luos/blob/master/inc/luos_list.h" target = "_blank">here</a>.

If you want to create new commands for your custom services, you can create and add your own starting at `LUOS_LAST_STD_CMD`. [See how](../../tutorials/tutorials.md).

