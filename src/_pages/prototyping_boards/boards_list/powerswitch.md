---
layout: post
title:  "Power switch"
date:   2019-03-15 17:59:00 +0100
categories: -_boards_list
tags: [actuation]
---
{% assign board = "Power-switch" %}
{% assign alias = "switch_mod" %}
{% assign type = "[State](/../modules_list/state)" %}
{% include var.md %}

# How to start with the {{ board }} board
{% include card.md %}

## Power considerations
The {{ board }} board allows you to interrupt another circuit up to 10A on 230V AC or 30V DC. The blue led on the module indicates when the link between 2 entries on the screw connector is closed.
The {{ board }} board supports 5V to 24V DC input.
