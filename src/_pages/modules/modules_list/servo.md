---
layout: post
title:  "Servo"
date:   2019-03-15 17:58:00 +0100
categories: -_modules_list
---
{% assign module = "Servo" %}
{% include var.md %}

# Introduction to the {{ module }} module type

The {{ module }} allows to drive RC elements like servomotor

The {{ module }} module type has access to all common capabilities.

----

## Functions

| **Function name and parameters** | **Action** | **Comment** |
| control(self) | Displays module type graphical interface | Only available using Jupyter notebook |

## Variables

| **Variable name** | **Action** | **Type** |
| rot_position | Rotation position in °. | read / write: float |
| max_angle | Sets `max_angle` value, in degrees (`°`) (`180.0` by default) | read / write: Float |
| min_pulse | Sets PWM minimum pulse value, in seconds (`s`) (`0.0005` by default)| read / write: Float |
| max_pulse | Sets PWM maximum pulse value, in seconds (`s`) (`0.0015` by default)| read / write: Float |

<div class="cust_edit_page"><a href="https://{{gh_path}}{{modules_path}}/servo.md">Edit this page</a></div>
