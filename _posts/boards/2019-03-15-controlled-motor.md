---
layout: post
title:  "Controlled motor"
date:   2019-03-15 17:59:00 +0100
categories: board
tags: [sensor, interface]
---
{% assign board = "Controlled-motor" %}
{% assign alias = "controlled_moto" %}
{% include var.md %}

# How to start with the {{ board }} board
{% include card.md %}

## How to connect your motor-reducer to your boards

The {{ board }} board is designed to control motors with a reducer and a sensor. It provides PH connector with 6 pins, where the motor can be plugged.
This board accepts supply voltage from 7V to 24V.

To control regular DC motors (without reduction neither sensor), please refer to [DC motor board’s documentation](/module/dc-motor).

<blockquote class="warning"><strong>Warning:</strong> The USB board provides too weak power to drive a motor-reducer with this board. A power board such as Battery board or Power plug board shall be used.</blockquote><br />

This board is able to control DC motors with a reduction and a sensor (usually called motor-reducer or speed-reducer).

The {{ board }} board provides a PID control on the output position, and PID control on the output speed, taking into account the reducer and the encoder.

You can find basic information about PID control here: [<big>An introduction to PID control with DC motor</big>](https://medium.com/luosrobotics/an-introduction-to-pid-control-with-dc-motor-1fa3b26ec661).
