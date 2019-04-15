---
layout: post
title:  "Controlled-motor module"
date:   2019-03-15 17:58:00 +0100
categories: module
tags: [actuation]
---
{% assign module = "Controlled-motor" %}
{% include var.md %}

# How to start with the {{ module }} module

This guide contains all the basic notions you will need to use the {{ module }} module.

<div class="sheet" markdown="1">
<p class="sheet-title" markdown="1">**Name:** {{module}}</p>
<p class="sheet-title" markdown="1">**Type:** {{type}}</p>
<p class="sheet-title" markdown="1">**Image**</p>
<p class="indent" markdown="1"><img height="150" src="/assets/img/{{ module | downcase }}-module.png" alt="{{ tag | Capitalize }}"></p>
<p class="sheet-title" markdown="1">**Categories**</p>
<p class="indent" markdown="1">
{% for tag in page.tags %}
  <a href="{{ "/" | absolute_url }}tags.html"><img height="50" src="/assets/img/sticker-{{ tag }}.png" alt="{{ tag | capitalize }}"></a>
{% endfor %}
</p>
</div>

## How to connect your motor-reducer to your modules

The {{ module }} module is designed to control motors with a reducer and a sensor. It provides PH connector with 6 pins, where the motor can be plugged.
This module accepts supply voltage from 7V to 24V.

<blockquote class="warning"><strong>Warning:</strong> The USB module provides too weak power to drive a motor-reducer with this module. A power module such as Battery module or Power plug module shall be used.</blockquote><br />


## How to use the {{ module }} module with Pyluos
This module is able to control DC motors with a reduction and a sensor (usually called motor-reducer or speed-reducer). To control regular DC motors (without reduction neither sensor), please refer to [DC motor module’s documentation]({{ site.baseurl }}{% post_url modules/2019-03-15-dc-motor-module %}).

The {{ module }} module provides a PID control on the output position, and PID control on the output speed, taking into account the reducer and the encoder. 

You can find basic information about PID control here: [<big>An introduction to PID control with DC motor</big>](https://medium.com/luosrobotics/an-introduction-to-pid-control-with-dc-motor-1fa3b26ec661).

To control the {{ module }} module, you have to connect it in a Luos network and connect this network through a communication module to Pyluos. Then you can access to your module and control it.


### Module’s type settings:

<blockquote class="warning"><strong>Warning:</strong> These settings must be set in your code as an initialization before starting to use your module.</blockquote><br />

Both PID’s values have to be set accordingly to the motor-reducer plugged to the module. Each different motor-reducer will have different PID’s values, for position and speed control.
The default values `[0, 0, 0]` won’t have any effect on the motor, and must be changed. The following `set` values are examples (`robot` and `controlled_moto` are to be replaced by the actual names of your robot and {{ module }} module). 

Set the position PID values `[P, I, D]`:
```python
robot.controlled_moto.positionPid = [3,0.002,100]
 ```
Set the speed PID values `[P, I, D]`:
```python
robot.controlled_moto.speedPid = [0.4,0.02,0]
 ```
Set the encoder resolution before reduction:
```python
robot.controlled_moto.encoder_res =16
 ```
Set the gear reduction ratio:
```python
robot.controlled_moto.reduction = 131
 ```
Set the output wheel’s diameter, in mm:
```python
robot.controlled_moto.wheel_size = 100
 ```
Reinitialize the position measurement:
```python
robot.controlled_moto.setToZero()
 ```
### Module’s driving modes:

This module can drive a motor-reducer in several modes:

* Power mode: A value between `-100` and `100` is applied, defining the percentage of full power of the motor, in both possible directions.
* Rotational speed mode: An angular speed value in degrees / second is applied.
* Rotational position mode: An angle in degrees is applied, for the motor-reducer to reach.
* Linear speed mode: A linear speed in millimeters / second is applied (wheel’s diameter must be set).
* Linear position mode: A position in millimeter is applied, for the wheel to reach.

The following commands allow to enable or disable the different driving modes. Only one mode can be enabled at a time.

Power mode (this is the only mode which is enabled by default):
```python
robot.controlled_moto.power_mode(True)
``` 
Angular speed mode:
```python
robot.controlled_moto.rot_speed_mode(False)
``` 
Angular position mode:
```python
robot.controlled_moto.rot_position_mode(False)
``` 
Linear speed mode:
```python
robot.controlled_moto.trans_speed_mode(False)
``` 
Linear position mode:
```python
robot.controlled_moto.trans_position_mode(False)
``` 
### Module’s control values:

Several values can be get or set, using the following commands:

Get a value:
```python
robot.controlled_moto.****
``` 
Set a value:
```python
robot.controlled_moto.****(value)
``` 
The characters \*\*\*\* must be replaced by one of the following:

* `compliant` (Return or set `True` or `False`)
* `power_ratio` (Return or set a value from `-100.0` to `100.0`)
* `target_rot_speed` (Return or set the target value of the output’s speed in degrees/second)
* `target_rot_position` (Return or set the target value of the position in degrees)
* `target_trans_speed` (Return or set the target value of the linear speed of the wheel, in millimeters / second)
* `target_trans_position` (Return or set the target value of the wheel’s position in millimeters)

> Note: By default, the {{module}} module is compliant (ie. `robot.controlled_moto.compliant` is `True`).

### Module’s report data:

Measurements are available to be read, and can be enabled or disabled.

Read a value:
```python
robot.controlled_moto.****
``` 
Enable or disable a value measurement:
```python
robot.controlled_moto.**** = True (or False)
``` 
The characters \*\*\*\* must be replaced by one of the following:

* `rot_position` (Return the rotational angle of the output, `°`)
* `rot_speed` (Return the rotational speed of the output, `°/s`)
* `trans_position` (Return the position of the wheel, `mm`)
* `trans_speed` (Return the linear speed of the wheel, `mm/s`)
* `current` (Return the current into the motor, `A`)

### Other:

Display the controls of the module:
```python
robot.controlled_moto.control()
``` 

----

## Functions
List of functions of {{module}} module:

| **setToZero(self)** | - | - | 
| **power_mode(self, enable)** | - | - | 
| **rot_speed_mode(self, enable)** | - | - | 
| **trans_speed_mode(self, enable)** | - | - | 
| **trans_position_mode(self, enable)** | - | - | 
| **rot_position_mode(self, enable)** | - | - | 
| **control(self)** | - | - | 
| **change_config(rot_speed_report, rot_position_report, trans_speed_report, trans_position_report, current_report, compliant_mode, power_mode, power_ratio, rot_speed_mode, rot_speed, rot_position_mode, rot_position, trans_speed_mode, trans_speed, trans_position_mode, trans_position)** | - | - | 
| **-** | - | - | 

## Variables
List of variables of {{module}} module:
 
| **positionPid(self, new_pid)** | - | - | 
| **speedPid(self, new_pid)** | - | - | 
| **encoder_res** | - | - | 
| **reduction(self, s)** | - | - | 
| **wheel_size(self, s)** | - | - | 
| **compliant(self, enable)** | - | - | 
| **power_ratio(self, s)** | - | - | 
| **target_rot_speed(self, s)** | - | - | 
| **target_rot_position(self, s)** | - | - | 
| **target_trans_speed(self, s)** | - | - | 
| **target_trans_position(self, s)** | - | - | 
| **compliant(self, enable)** | - | - | 
| **target_trans_speed(self)** | - | - | 
| **rot_position(self, enable)** | - | - | 
| **rot_speed(self, enable)** | - | - | 
| **trans_position(self, enable)** | - | - | 
| **trans_speed(self, enable)** | - | - | 
| **current(self, enable)** | - | - | 
| **-** | - | - | 

## Events
List of events of {{module}} module:

| **-** | - | - | 

