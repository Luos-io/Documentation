---
layout: post
title: "Electronic design"
categories: 1_integrating_luos
desc: Rules of design for Luos electronic boards.
order: 0
wip: 1
---
{% include var.md %}

<div class="wip_img"></div>
<blockquote class="warning"><strong>Work in progress</strong><br /><br />We are working on this page...</blockquote><br />

# Integrating Luos into an electronic board

Luos is uploaded into every microcontrollers of every electronic boards in the device, and can be upgraded with the future versions to come. 
Electronic boards must respect some design rules in order to properly works in a Luos network.

A Luos-friendly electronic board must contains *at least* the following elements:
 - **1** [**MCU**](https://en.wikipedia.org/wiki/Microcontroller) (microcontroller unit): it hosts the Luos firmware along with the different modules (drivers and apps).
 - **2 Luos connectors**, reference: [*DF11-8DP-2DS(24)*](https://octopart.com/df11-8dp-2ds%2824%29-hirose-39521447): These are the connectors used to connect electronic boards together with Luos cables.


