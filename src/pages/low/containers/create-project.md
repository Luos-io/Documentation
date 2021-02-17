# Create Luos Projects
## How to properly organize your Luos projects

### How to run Luos
Luos is like a task that has to be run regularly. So you will have to run it by adding `luos_init()` and `luos_loop()` in the `main()` of your program.<br/>
Basically, your `main()` will look like this:

```C
#include "luos.h"

int main(void)
{
    Luos_Init();
    while(1)
    {
        Luos_Loop();
    }
    return 0;
}

```
Putting this code into a <span class="cust_tooltip">node<span class="cust_tooltiptext">{{node_def}}</span></span> makes it able to react to a Luos network. It's now ready to host your containers.

**As a developer you will always develop your functionalities into containers and never into the `main()` program.**

> **Note:** The only information that should be put on the `main()` code are MCU setup parameters and containers' run functions.

### How to add containers in your project
A node can host multiple containers, and a container has to be as portable as possible. In order to do that, containers have to be independent code folders that can be easily copied and pasted in another project.<br/>
To make it at Luos we always use the same way to organize our projects: we put the containers into a `containers` folder and name the containers' code files with the name of each container:

```AsciiDoc
 Project
    │
    ├─── containers
    │    ├─── container_1
    │    │    ├─── container_1.c
    │    │    └─── container_1.h
    │    └─── container_2
    │         ├─── container_2.c
    │         └─── container_2.h
    │
    ├─── Inc
    │    ├─── Luos
    │    └─── luos_hal
    │
    └─── Src
         └─── Main.c
```

### Basic containers functions
We choose to put the public functions of our containers in the `container.h` file. Like Luos, containers are similar to tasks that need to be run regularly, so we choose to use the exact same stategy as presented for Luos functions by providing a `Container_Init()` and a `Container_Loop()` functions and to add them in the `main()`.
Following the previous folder organization, the `main()` code looks like this:

```C
#include "luos.h"
#include "container1.h"
#include "container2.h"

int main(void)
{
    Luos_Init();
    Container1_Init();
    Container2_Init();
    while(1)
    {
        Luos_Loop();
        Container1_Loop();
        Container2_Loop();
    }
    return 0;
}

```

This way, it is easy to manage all your containers and to add as many of them in the `main()` file as you want.

### How to use Luos hardware abstraction layer
To ease the use of Luos Library on a specific target, Luos compagny provides luos_hal.c, luos_hal.h and luos_hal_config.h from LuosHAL folder for a lot of different MCU families. The purpose of these files is to make Los communcation to work directly on the chosen MCU. however, this default configuration for a MCU Family should be adapted to fit with your design. See how to do it on [Luos HAL](../hardware_topics/luos-hal.md) page.

