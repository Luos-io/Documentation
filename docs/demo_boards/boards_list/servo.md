# Servo board

<div className="cust_sheet" markdown="1">
<p className="cust_sheet-title" markdown="1"><strong>Default Alias:</strong> servo1_mod, servo2_mod, servo3_mod, servo4_mod</p>
<p className="cust_sheet-title" markdown="1"><strong>Type:</strong> <a href="../../software/containers_list/servo.md">Servo</a></p>
<p className="cust_sheet-title" markdown="1"><strong>Number of container(s):</strong> 4</p>
<p className="cust_sheet-title" markdown="1"><strong>Image</strong></p>
<p className="cust_indent" markdown="1"><img height="150" src="/img/servo-container.png" alt="" /></p>
<p className="cust_sheet-title" markdown="1"><strong>Category(-ies)</strong></p>
<p className="cust_indent" markdown="1">
<img height="50" src="/img/sticker-actuation.png" title="Actuation" alt="" />
</p>
<p className="cust_sheet-title" markdown="1"><strong>Project source </strong></p>
<a className="github-button" data-size="large" aria-label="Star Luos-io/Luos on GitHub" href="https://github.com/Luos-io/Examples/blob/master/Projects/l0/Servo" target="_blank">Servo</a>
</div>

## How to connect your servo-motors to your containers

The Servo board has 4 servo-motor ports ordered as shown on the following picture, from `S1` to `S4`.

![Servomotor ports](/img/servo-1.png)

## Power considerations

The Servo board accepts supply voltage from `5V` to `7V`. Watch out to always power your motor with an appropriate voltage.

> **Warning:** A USB board generally provides too weak power to drive a servomotor properly. It must be plugged to a Power plug board, for example.
