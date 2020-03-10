---
layout: post
title: "Luos architecture"
categories: 0_first_steps
desc: How Luos is organized
order: 2
wip: 1
---
{% include var.md %}

<div class="wip_img"></div>
<blockquote class="warning"><strong>Work in progress</strong><br /><br />We are working on this page...</blockquote><br />

# Luos architecture

A Luos architecture is composed of the following elements:
 - **Robus API:** Communication BUS based on RS485 ([see the differences with others BUS](/../others/bus-comparison)). It handles the low level messages.
 - **Luos API:** It's the layer around Robus, managing the messages between modules and the route table.
 - **Modules:** They are the bricks added for each function ([app](/../modules/apps) type modules) or each hardware element such as sensors or actuators ([driver](/../modules/drivers) type modules) into the Luos architecture. They communicate together
 - **Main:** the main code managing modules' communication. Users can customize Luos (e.g. add new <span class="tooltip">[OD objects](/../2_modules/od)<span class="tooltiptext">{{ od_def }}</span></span>, add new module types, etc.) according to their needs. The Luos and Robus API must never be changed, because all the changes will be erased at each update.
 - **Pyluos:** The library used to program high-level robot behaviors on a computer, in Python language.

![Luos architecture](../../assets/img/archit.png)



