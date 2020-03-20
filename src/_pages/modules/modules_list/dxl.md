---
layout: post
title:  "Dynamixel motor"
date:   2019-03-15 17:58:00 +0100
categories: -_modules_list
tags: [sensor, interface]
---
{% assign module = "DynamixelMotor" %}
{% include var.md %}

# Introduction to the {{ module }} module type

The {{ module }} module allows to control Dynamixel motors.

The {{ module }} module type has access to all common capabilities.

----

## Functions

| **Function name and parameters** | **Action** | **Comment** |
| set_id(self, id) | Changes motor ID | This new Id will be saved by the Dynamixel motor. You have to detect motors again to make it work after this change. |
| detect(self) | Launches a motor detection | You have to run a luos detection to include or exclude new motors. |
| register(self, register, val) | Sets a Dynamixel register value. | This register only manage *word* size register. Use it only if you know what you do. |
| control(self) | Displays module type graphical interface | Only available using Jupyter notebook |

## Variables

| **Variable name** | **Action** | **Type** |
| compliant | - True: disables the motor power, you can use it to move the motor by hand.<br/> - False: Enables the motor power. | read / write: Boolean (True or False) |
| target_rot_position | Sets the target rotation position to reach in °. | read / write: Float |
| target_rot_speed | Sets the target rotation speed to reach in °/s. | read / write: Float |
| wheel_mode | Enables or disables wheel mode on motor | read / write: Boolean (True or False) |
| rot_position | Measured position of the motor in °. | read / write: Float |
| temperature | Measured temperature of the motor in °C. | read / write: Float |
| positionPid | Sets position PID used for rotation position mode and translation position mode | read / write: [float P, float I, float D] |
| power_ratio_limit | Max power limit in %. | read / write: Float |
| rot_position_limit | Min and Max rotation position limit in °. | read / write: [Float(min), Float(max)] |

<div class="cust_edit_page"><a href="https://{{gh_path}}{{modules_path}}/dxl.md">Edit this page</a></div>
