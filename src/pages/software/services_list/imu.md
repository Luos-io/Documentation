
# Imu API

The Imu service handles an inertial sensor.

Its type has access to all common capabilities.

Imu services can measure:

* Compass – Magnetic field data in micro-tesla on each axis
* Gyro – X, Y, Z axis rotational acceleration data in degrees per second
* Accel – X, Y, Z axis linear acceleration data in G
* Heading – 360 degrees from North with Y+ axis as the pointer
* Rotational Matrix – linear math 9 element matrix representation
* Euler Angles – Pitch, roll, yaw based in degrees with frame reference
* Quaternions – Sensor fused w, x, y, z rotational angles
* Linear Acceleration – Linear acceleration in body frame coordinates
* Gravity Vector – Which access gravity effects
* Pedometer – Step number
* walk time – Duration (second) of the walk

By default, the service will send only quaternions to keep a low number of data and avoid bus congestion. Retrieving any other types of measures requires to enable them first.

The easiest way to enable a measure is by using it, as Pyluos automatically enables a called measure. For example, to retrieve the linear acceleration value when it’s disabled, you can execute:

```python
device.Imu_mod.linear_acceleration
```

This command doesn’t allow you to disable the value after using it in order to keep your device responsive. Another way to enable or disable a value is to set it to `True` or `False`. For example, if you want to disable the linear acceleration measure previously activated, you can execute:

```python
device.Imu_mod.linear_acceleration = False
```

----

## Functions

| **Function name and parameters** | **Action** | **Comment** |
|:---:|:---:|:---:|
| control(self) | Displays service type graphical interface | Only available using Jupyter notebook |

## Variables

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| compass | Magnetic field data in micro-tesla on each axis | read only: \[Float, Float, Float\] |
| compass | Starts/Stops compass measurement actualization | write only: Boolean (True or False) |
| gyro | X, Y, Z axis rotational acceleration data in degrees per second | read only: \[Float, Float, Float\] |
| gyro | Starts/Stops gyro measurement actualization | write only: Boolean (True or False) |
| acceleration | X, Y, Z axis linear acceleration data in G | read only: \[Float, Float, Float\] |
| acceleration | Starts/Stops acceleration measurement actualization | write only: Boolean (True or False) |
| heading | 360 degrees from North with Y+ axis as the pointer | read only: \[Float, Float, Float\] |
| heading | Starts/Stops heading measurement actualization | write only: Boolean (True or False) |
| rotational_matrix | Linear math 9 element matrix representation | read only: \[Float, Float, Float, Float, Float, Float, Float, Float, Float\] |
| rotational_matrix | Starts/Stops rotational_matrix measurement actualization | write only: Boolean (True or False) |
| euler | Pitch, roll, yaw based in degrees with frame reference | read only: \[Float, Float, Float\] |
| euler | Starts/Stops euler measurement actualization | write only: Boolean (True or False) |
| quaternion | Sensor fused w, x, y, z rotational angles | read only: \[Float, Float, Float, Float\] |
| quaternion | Starts/Stops quaternion measurement actualization | write only: Boolean (True or False) |
| linear_acceleration | Linear acceleration in body frame coordinates | read only: \[Float, Float, Float\] |
| linear_acceleration | Starts/Stops linear_acceleration measurement actualization | write only: Boolean (True or False) |
| gravity_vector | Which access gravity effects | read only: \[Float, Float, Float\] |
| gravity_vector | Starts/Stops gravity_vector measurement actualization | write only: Boolean (True or False) |
| pedometer | Step number | read only: int |
| pedometer | Starts/Stops pedometer measurement actualization | write only: Boolean (True or False) |
| walk_time | Duration (second) of the walk | read only: Float  |
| walk_time | Starts/Stops walk_time measurement actualization | write only: Boolean (True or False) |

## ROS topics
| **Topic name** | **Message type** |
|:----|:---:|
| /Imu_mod/variables/pedometer/read | std_msgs/msg/UInt32
| /Imu_mod/variables/pedometer/write | std_msgs/msg/Bool
| /Imu_mod/variables/walk_time/read | std_msgs/msg/Float32
| /Imu_mod/variables/walk_time/write | std_msgs/msg/Bool
| /Imu_mod/variables/gravity_vector/read | geometry_msgs/msg/Vector3
| /Imu_mod/variables/gravity_vector/write | std_msgs/msg/Bool
| /Imu_mod/variables/heading/read | std_msgs/msg/UInt32
| /Imu_mod/variables/heading/write | std_msgs/msg/Bool
| /Imu_mod/acceleration | geometry_msgs/msg/Accel
| /Imu_mod/imu | sensor_msgs/msg/Imu
| /Imu_mod/compass | sensor_msgs/msg/MagneticField


## Example of use of your IMU service using Jupyter notebook

In this example, we will display in 3D the rotation sensor by using quaternions. In order to do that, we will use the *pythreejs* lib and *jupyter notebook*.

First, install and enable this library by typing these commands in a terminal:

```bash
pip install pythreejs

jupyter nbextension install --py --symlink --sys-prefix pythreejs

jupyter nbextension enable --py --sys-prefix pythreejs
```

Now, restart *jupyter notebook* and add this code to a new python script:

```python
from pyluos import Device
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

# Connect your Luos network (here using an USB service)
r = Device('/dev/cu.usbserial-DN38OIYT')

# Control the rotation of the cube with the rotation of the Imu sensor
while(True):
cube.quaternion = r.Imu_mod.quaternion
time.sleep(0.05)
```

You should obtain a result like this:

<iframe width="800" height="450" src="https://www.youtube.com/embed/eTRd8a0ABMM?feature=oembed" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
