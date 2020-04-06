# Servo board
<div class="cust_sheet" markdown="1">
<p class="cust_sheet-title" markdown="1"><strong>Default Alias:</strong> servo1_mod, servo2_mod, servo3_mod, servo4_mod</p>
<p class="cust_sheet-title" markdown="1"><strong>Type:</strong> <a href="/pages/high/modules_list/servo.md">Servo</a></p>
<p class="cust_sheet-title" markdown="1"><strong>Number of module(s):</strong> 4</p>
<p class="cust_sheet-title" markdown="1"><strong>Image</strong></p>
<p class="cust_indent" markdown="1"><img height="150" src="{{img_path}}/servo-module.png"></p>
<p class="cust_sheet-title" markdown="1"><strong>Category(-ies)</strong></p>
<p class="cust_indent" markdown="1">
<img height="50" src="{{img_path}}/sticker-actuation.png" title="Actuation">
</p>
<p class="cust_sheet-title" markdown="1"><strong>Project source: </strong><a href="https://github.com/Luos-io/Mod_servo" target="_blank">Mod_servo</a></p>
</div>

## How to connect your servo-motors to your modules

The Servo board has 4 servo-motor ports ordered as shown on the following picture, from `S1` to `S4`.

![Servomotor ports]({{img_path}}/servo-1.png)

## Power considerations
The Servo board accepts supply voltage from `5V` to `7V`. Watch out to always power your motor with an appropriate voltage.

> **Warning:** A USB board generally provides too weak power to drive a servomotor properly. It must be plugged to a Power plug board, for example.

<div class="cust_edit_page"><a href="https://{{gh_path}}{{boards_path}}/servo.md">Edit this page</a></div>
