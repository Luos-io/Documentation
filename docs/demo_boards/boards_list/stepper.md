# Stepper board

<div class="cust_sheet" markdown="1">
<p class="cust_sheet-title" markdown="1"><strong>Default Alias:</strong> stepper_mod</p>
<p class="cust_sheet-title" markdown="1"><strong>Type:</strong> <a href="../../software/containers_list/stepper.md">Stepper</a></p>
<p class="cust_sheet-title" markdown="1"><strong>Number of container(s):</strong> 1</p>
<p class="cust_sheet-title" markdown="1"><strong>Image</strong></p>
<p class="cust_indent" markdown="1"><img height="150" src="/img/stepper-container.png" alt="" /></p>
<p class="cust_sheet-title" markdown="1"><strong>Category(-ies)</strong></p>
<p class="cust_indent" markdown="1">
<img height="50" src="/img/sticker-actuation.png" title="Actuation" alt="" />
</p>
<p class="cust_sheet-title" markdown="1"><strong>Project source </strong></p>
<a class="github-button" data-size="large" aria-label="Star Luos-io/Luos on GitHub" href="https://github.com/Luos-io/Examples/blob/master/Projects/l0/Stepper" target="_blank">Stepper</a>
</div>

## How to connect a stepper motor to the Stepper board

The Stepper board has one 4-pin PH connector where a stepper motor can be plugged.

## Current limitations

The current allowed in the stepper motor must be limited in order to avoid overheat into the board. The trimming potentiometer situated at the left of the radiator on the board is used to control the output current.

<img height="150" src="/img/steppermotor-potar.jpg" title="Stepper motor" alt="" />

The trimming potentiometer is used as followed:

1. Turning the trimming potentiometer clockwise (towards pin 3) has the effect of **reducing** the output current.
2. Turning the trimming potentiometer counterclockwise (towards pin 1) has the effect of **rising** the output current (with overheating risks).

<img height="100" src="/img/potentiometer-stepper-motor.png" title="Potentiometer Stepper motor" alt="" />

> **Warning:** Considering these specifications, be sure to have the trimming potentiometer fully turned clockwise to the pin 3 in order to limit the current and avoid overheat. Then you can set the right amount of output current by slowly turning the trimming potentiometer counterclockwise.

## Power considerations

This board accepts supply voltage from `7V` to `24V`.

> **Warning:** USB Gate board doesn't provide enough voltage through USB cable to power the Stepper board nor drive a stepper motor. A power board such as Battery board or Power plug board shall be used.
