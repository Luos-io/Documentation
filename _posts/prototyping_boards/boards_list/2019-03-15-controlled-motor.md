---
layout: post
title:  "Controlled motor"
date:   2019-12-11 17:20:00 +0100
categories: -_boards_list
tags: [actuation]
---
{% assign board = "Controlled motor" %}
{% assign alias = "controlled_moto" %}
{% assign type = "[Controlled motor](/../modules_list/controlled-motor)" %}
{% include var.md %}

# How to start with the {{ board }} board
{% include card.md %}

## How to connect your motor-reducer to your boards

The {{ board }} board is designed to control motors with a reducer and a sensor. It provides PH connector with 6 pins, where the motor can be plugged.

### Connector's reference

![Male connector]({{ "/" | absolute_url }}/assets/img/ctrl_mot_male_connector.jpg){: .small_img } Male connector's reference (on the board): [**B6B-PH-K-S(LF)(SN)**](https://octopart.com/b6b-ph-k-s%28lf%29%28sn%29-jst-248872)

![Female connector]({{ "/" | absolute_url }}/assets/img/ctrl_mot_female_connector.jpg){: .small_img } Female connector's reference (on the wire): [**PHR-6**](https://octopart.com/phr-6-jst-279165)

![Crimp]({{ "/" | absolute_url }}/assets/img/ctrl_mot_crimp.jpg){: .small_img } Crimp's reference (on the wire): [**BPH-002T-P0.5S**](https://octopart.com/bph-002t-p0.5s-jst-8407485)

### Pinout and characteristics

![Pinout]({{ "/" | absolute_url }}/assets/img/controlled_motor_pinout.png)<br />*PHR-6 connector pinout.*



This board accepts supply voltage from 7V to 24V.

To control regular DC motors (without reduction neither sensor), please refer to [DC motor board’s documentation](/boards_list/dc-motor).

<blockquote class="warning"><strong>Warning:</strong> The USB board provides too weak power to drive a motor-reducer with this board. A power board such as Battery board or Power plug board shall be used.</blockquote><br />

This board is able to control DC motors with a reduction and a sensor (usually called motor-reducer or speed-reducer).

The {{ board }} board provides a PID control on the output position, and PID control on the output speed, taking into account the reducer and the encoder.

You can find basic information about PID control here: [<big>An introduction to PID control with DC motor</big>](https://medium.com/luosrobotics/an-introduction-to-pid-control-with-dc-motor-1fa3b26ec661) and a example code to tune your PID on the [Controlled motor module](/module/controlled-motor) page of this documentation.
