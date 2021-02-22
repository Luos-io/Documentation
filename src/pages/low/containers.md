# Containers

A container is a block of code which will provide functionality. Containers are able to communicate with any other containers present in the Luos network. 

A container can be an [application](./containers/create-containers.html#apps-guidelines) or a [driver](./containers/create-containers.html#drivers-guidelines).

Each container provides a particular set of tasks such as managing a motor, handling a laser range finder, or more complex operations like computing an inverse-kinematics.

Each container is hosted in a single <span class="cust_tooltip">node<span class="cust_tooltiptext">{{node_def}}</span></span> (MCU), but a node can handle several containers at the same time and manage communication between them and between other containers hosted in other nodes, using the same network interface.

**As a developer you will always develop your functionalities into containers, and never into the `main()` program.** The only information that should be put on the `main()` code are MCU setup parameters and containers' run functions.

## Container properties
To properly work, each container has some properties allowing other containers to recognize and access it:

| Name | Description | Format |
| :---: | :---: | :---: |
| **ID** | The ID is a unique number given to each container depending on their physical position. The system automatically assigns each ID during the [detection phase](../overview/general-basics.html#container-detection). If you move a container from a microcontroller A to a microcontroller B on a given device, the ID will change. In the same way, if you change the wiring order of a microcontroler on the network on a given device, the ID will change too. | Integer<br />e.g. `Id=1` |
| **TYPE** | The type defines the container purpose. A few types are predefined and can be used, or new ones can be created. The container type can't be changed after container initialization. | String<br />e.g. `type=DISTANCE_MOD` |
| **ALIAS** | Alias is the name of the container. It's used to easily identify a container. Each container has a **default alias** which can be changed by users. For example, a container with the default alias `motor_mod` can be named `left_knee_motor` by user. This new name will be stored in the non-volatile memory of the board. As we don't want to have multiple containers with the same name, a duplicate name on your system will be automatically assigned with an incrementing number at its end, in the network. You can go back to the default name by setting a void name (`""`) to a container. | String<br />e.g. `alias="gate"` |


