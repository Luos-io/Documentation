---
layout: post
title:  "IMU module"
date:   2019-03-15 17:53:00 +0100
categories: modules
tags: [sensor]
---
{% assign module = "IMU" %}
{% include var.md %}

# How to start with the {{ module }} module

This guide contains all the basic notions you will need to use the {{ module }} module.

<div class="sheet" markdown="1">
<p class="sheet-title" markdown="1">**Name:** {{module}}</p>
<p class="sheet-title" markdown="1">**Type:** {{type}}</p>
<p class="sheet-title" markdown="1">**Image**</p>
<p class="indent" markdown="1"><img height="150" src="/assets/img/{{ module }}-module.png" alt="{{ tag | Capitalize }}"></p>
<p class="sheet-title" markdown="1">**Categories**</p>
<p class="indent" markdown="1">
{% for tag in page.tags %}
  <a href="{{ "/" | absolute_url }}tags.html"><img height="50" src="/assets/img/sticker-{{ tag }}.png" alt="{{ tag | capitalize }}"></a>
{% endfor %}
</p>
</div>

## How to use the {{ module }} module with Pyluos
To control the {{ module }} module, you have to connect it to a Luos network and connect this network through a communication module to Pyluos.

Then you can access to your module and control it.

IMU module can measure:

* Compass – magnetic field data in micro tesla on each axis
* Gyro – X, Y, Z axis rotational acceleration data in degrees per second
* Accel – X, Y, Z axis linear acceleration data in G
* Heading – 360 degrees from North with Y+ axis as the pointer
* Rotational Matrix – linear math 9 element matrix representation
* Euler Angles – Pitch, roll, yaw based in degrees with frame reference
* Quaternions – sensor fused w, x, y, z rotational angles
* Linear Acceleration – linear acceleration in body frame coordinates
* Gravity Vector – Which access gravity effects
* Pedometer – step number
* walk time – duration (second) of the walk

By default the module will send you only quaternions to keep the number of data low and avoid bus congestion. Retrieving any other type of measure requires to enable it first.

The easiest way to enable a measure is by using it, as pyluos enables automatically a called measure. For example, to retrieve the linear acceleration value when it’s disabled, you have to execute:

```python
robot.Imu_mod.linear_acceleration
```
 
This command doesn’t allow you to disable the value after using it in order to keep your robot responsive. Another way to enable or disable a value is to set it to `True` or `False`. For example, if you want to disable the linear acceleration measure previously activated, you can execute:

```python
robot.Imu_mod.linear_acceleration = False
```

If you are using *jupyter notebook* you can use the `control()` command available for each module. This command will list all available data from your module and you can see witch one is enabled and clic on it to enable or disable it:

```python
robot.Imu_mod.control()
```

## Example of use of your IMU module using jupyter notebook

In this example, we will display in 3D the sensor rotation using quaternions. In order to do that, we will use the *pythreejs* lib and *jupyter notebook*.

First, we have to install and enable this library by typing these commands in a terminal:

```bash
pip install pythreejs

jupyter nbextension install --py --symlink --sys-prefix pythreejs

jupyter nbextension enable --py --sys-prefix pythreejs
```
 
Now, restart *jupyter notebook* and add this code to a new python script:

```python
from pyluos import Robot
import time
from pythreejs import *

# Create a cube to move following our sensor
cube = Mesh(
BoxBufferGeometry(3, 3, 3),
MeshPhysicalMaterial(color='green'),
position=[0, 0, 0],
castShadow = True
)

# Create a floor
plane = Mesh(
PlaneBufferGeometry(100, 100),
MeshPhysicalMaterial(color='gray'),
position=[0, -1.5, 0], receiveShadow = True)
plane.rotation = (-3.14/2, 0, 0, 'XYZ')

# Create a directional light following our cube
key_light = SpotLight(position=[0, 10, 10], angle = 0.3, penumbra = 0.1, target = cube, castShadow = True)
key_light.shadow.mapSize = (2048, 2048)

# Create a camera
c = PerspectiveCamera(position=[4, 12, 10], up=[0, 1, 0],
aspect=800/400)

# Create a scene
scene = Scene(children=[plane, cube, c, key_light, AmbientLight()])

# Display the scene with shadow and everything.
renderer = Renderer(camera=c,
scene=scene,
controls=[OrbitControls(controlling=c)],
width=800, height=400,
)
renderer.shadowMap.enabled = True
renderer.shadowMap.type = 'PCFSoftShadowMap'
display(renderer)

# Connect your Luos network (here using an USB module)
r = Robot('/dev/cu.usbserial-DN38OIYT')

# Control the rotation of the cube with the rotation of the Imu sensor
while(True):
cube.quaternion = r.Imu_mod.quaternion
time.sleep(0.05)

 
# Connect your Luos network (here using an USB module)
r = Robot('/dev/cu.usbserial-DN38OIYT')
 
# Control the rotation of the cube with the rotation of the Imu sensor
while(True):
cube.quaternion = r.Imu_mod.quaternion
time.sleep(0.05)
```
 
You should obtain something like that:

<iframe width="800" height="450" src="https://www.youtube.com/embed/eTRd8a0ABMM?feature=oembed" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

----

## Functions
List of functions of {{module}} module:

| **bit(self, i, enable)** | - | - | 
| **control(self)** | - | - | 
| **-** | - | - | 

## Variables
List of variables of {{module}} module:

| **quaternion(self, enable)** | - | - | 
| **acceleration(self, enable)** | - | - | 
| **gyro(self, enable)** | - | - | 
| **compass(self, enable)** | - | - | 
| **euler(self, enable)** | - | - | 
| **rotational_matrix(self, enable)** | - | - | 
| **pedometer(self, enable)** | - | - | 
| **walk_time(self, enable)** | - | - | 
| **linear_acceleration(self, enable)** | - | - | 
| **gravity_vector(self, enable)** | - | - | 
| **heading(self, enable)** | - | - | 
| **-** | - | - | 

## Events
List of events of {{module}} module:

| **-** | - | - | 
