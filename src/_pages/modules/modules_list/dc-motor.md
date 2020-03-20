---
layout: post
title:  "DC-motor"
date:   2019-03-15 17:57:00 +0100
categories: -_modules_list
tags: [actuation]
---
{% assign module = "DCMotor" %}
{% include var.md %}

# Introduction to the {{ module }} module type

The {{ module }} module allows to drive a DC motor using only power mode.

The {{ module }} module type has access to all common capabilities.

----

## Functions

| **Function name and parameters** | **Action** | **Comment** |
| control(self) | Displays module type graphical interface | Only available using Jupyter notebook |

## Variables

| **Variable name** | **Action** | **Type** |
| power_ratio | Sets the power quantity send to the motor between -100% and 100%. | read / write: float |

<div class="cust_edit_page"><a href="https://{{gh_path}}{{modules_path}}/dc-motor.md">Edit this page</a></div>
