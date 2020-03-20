---
layout: post
title:  "Stepper"
date:   2019-03-15 17:58:00 +0100
categories: -_modules_list
---
{% assign module = "Stepper" %}
{% include var.md %}

# Introduction to the {{ module }} module type

This module type allows to control a stepper motor.
This module computes micro-stepping and motion planning.

The {{ module }} module type has access to all common capabilities.

### Modules’s type settings:

<blockquote class="warning"><strong>Warning:</strong> This module doesn't save any of the following parameters, they must be set each time your module reboots.</blockquote><br />

The number of steps per turn must be defined, as well as the wheel diameter at the output of the motor if you wahnt to use translation. These specs may figure in your motor’s datasheet.

----

## Functions

| **Function name and parameters** | **Action** | **Comment** |
| setToZero(self) | Resets current position of the motor to 0 | You can use it to initialize the position of the motor |
| control(self) | Displays module type graphical interface | Only available using Jupyter notebook |

## Variables

### Motor settings

| **Variable name** | **Action** | **Type** |
| stepPerTurn | Defines the stepper resolution | read / write: float |
| wheel_size | Defines wheel size used for translation mode | read / write: float |

### Motor control modes

| **Variable name** | **Action** | **Type** |
| compliant | - True: disables the motor driver, you can use it to move the motor by hand.<br/> - False: Enables the motor driver. | read / write: Boolean (True or False) |
| rot_position_mode | Enables/Disables the motor rotation position control mode.<br/>*Disables power mode and translation position mode if enabled.*<br/>*Doesn't work if no position PID is configured.* | read / write: Boolean (True or False) |
| rot_speed_mode | Enables/Disables the motor rotation speed control mode.<br/>*Disables power mode and translation speed mode if enabled.*<br/>*Doesn't work if no speed PID configured.* | read / write: Boolean (True or False) |
| trans_position_mode | Enables/Disables the motor translation position control mode.<br/>*Disables power mode and rotation position mode if enabled.*<br/>*Doesn't work if no position PID configured.* | read / write: Boolean (True or False) |
| trans_speed_mode | Enables/Disables the motor translation speed control mode.<br/>*Disables power mode and rotation speed mode if enabled.*<br/>*Doesn't work if no speed PID configured.* | read / write: Boolean (True or False) |

### Motor commands

| **Variable name** | **Action** | **Type** |
| target_rot_position | Sets the target rotation position to reach in °. | read / write: float |
| target_rot_speed | Sets the target rotation speed to reach in °/s. | read / write: float |
| target_trans_position | Sets the target translation position to reach in mm. | read / write: float |
| target_trans_speed | Sets the target translation speed to reach in mm/s. | read / write: float |

<div class="cust_edit_page"><a href="https://{{gh_path}}{{modules_path}}/stepper.md">Edit this page</a></div>
