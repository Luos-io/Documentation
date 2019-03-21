---
layout: post
title:  "Stepper module"
date:   2019-03-15 17:45:00 +0100
categories: modules
tags: [Actuation]
---
{% include var.md %}
{% assign module = "Stepper" %}

# How to start with the {{ module }} module

This guide contains all the basic notions you will need to use the {{ module }} module.

<div class="sheet" markdown="1">

<p class="sheet-title" markdown="1">**Name**</p>

<p class="indent" markdown="1">{{module}}</p>

<p class="sheet-title" markdown="1">**Type**</p>

<p class="indent" markdown="1">`Stepper`</p>

<p class="sheet-title" markdown="1">**Image**</p>

<p class="indent" markdown="1">![{{ module }} module](/assets/img/stepper-module.png)</p>

<p class="sheet-title" markdown="1">**Categories**</p>

<p class="indent" markdown="1">
{% for tag in page.tags %}
  <a href="{{ "/" | absolute_url }}tags.html">{{ tag }}</a>
{% endfor %}
</p>
</div>



## Module categories

|<a href="{{ "/" | absolute_url }}tags.html">{{act_title}}</a>|
|:-|
|![{{ act_title }}]({{ act_img }})|
|{{act_desc}}|

## How to connect the stepper motor to the module
The Stepper motor module have one PH connector with 4 pins where a stepper motor can be plugged.

This module accepts supply voltage from `7V` to `24V`.

<blockquote class="warning"><strong>Warning:</strong> USB module provides too weak power to drive a motor-reducer with this module. A power module such as Battery module or Power plug module shall be used.</blockquote><br />


## How to use the Stepper motor module with Pyluos
This module can control a stepper motor.

To control {{module}} module, you have to connect it to a Luos network, and connect this network through a communication module to Pyluos. Then you can access to your module and control it.

### Module’s type settings:

<blockquote class="warning"><strong>Warning:</strong> These settings must be set in your code as an initialization before starting to use your module.</blockquote><br />

The number of steps per turn must be defined, as well as the wheel diameter at the output of the motor. These specs may figure in your motor’s datasheet. The following set values are examples (*robot* and *stepper* are to be replaced by the actual names of your robot and {{module}} module). 

Number of steps per turn:

```python
robot.stepper.stepPerTurn = 200
 ```
 
Wheel diameter, in `mm`:

```python
robot.stepper.wheel_size = 100
 ```
 
Reinitialize the position measurement:

```python
robot.stepper.setToZero()
 ```

### Module’s control values:

Several values can be get or set, using the following commands:

Get a value:

```python
robot.stepper.****
 ```
 
Set a value:

```python
robot.stepper.****(value)
 ```
 
The characters `****` must be replaced by one of the following:

* `compliant` (Return or set `True` or `False`)
* `target_rot_speed` (Return or set the target value of the output’s speed in degrees/second)
* `target_rot_position` (Return or set the target value of the position in degrees)
* `target_trans_speed` (Return or set the target value of the linear speed of the wheel, in millimeters / second)
* `target_trans_position` (Return or set the target value of the wheel’s position in millimeters)

> Note: By default, the {{module}} module is compliant (ie. `robot.stepper.compliant` is `True`).

Other:

Display the controls of the module:

```python
robot.stepper.control()
```
 

----

## Functions
List of functions of {{module}} module:

| **-** | - | - | 

## Variables
List of variables of {{module}} module:

| **-** | - | - | 

## Events
List of events of {{module}} module:

| **-** | - | - | 
