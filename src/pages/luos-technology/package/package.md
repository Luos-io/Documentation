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
Putting this code into a <span class="cust_tooltip">node<span class="cust_tooltiptext">{{node_def}}</span></span> makes it able to react to a Luos network. It's now ready to host your services.

**As a developer you will always develop your functionalities into services and never into the `main()` program.**

> **Note:** The only information that should be put on the `main()` code are MCU setup parameters and services' run functions.

### How to add services in your project
A node can host multiple services, and a service has to be as portable as possible. In order to do that, services have to be independent code folders that can be easily copied and pasted in another project.<br/>
When designing projects at Luos we always use the same way to organize our code: we put the services into a `services` folder and name the services' code files with the name of each service:

```AsciiDoc
 Project
    │
    ├─── services
    │    ├─── service_1
    │    │    ├─── service_1.c
    │    │    └─── service_1.h
    │    └─── service_2
    │         ├─── service_2.c
    │         └─── service_2.h
    │
    ├─── Inc
    │    ├─── Luos
    │    └─── luos_hal
    │
    └─── Src
         └─── Main.c
```

### Basic services functions
We choose to put the public functions of our services in the `service.h` file. Like Luos, services are similar to tasks that need to be run regularly, so we choose to use the exact same stategy as presented for Luos functions by providing a `Service_Init()` and a `Service_Loop()` functions and to add them in the `main()`.
Following the previous folder organization, the `main()` code looks like this:

```C
#include "luos.h"
#include "service1.h"
#include "service2.h"

int main(void)
{
    Luos_Init();
    Service1_Init();
    Service2_Init();
    while(1)
    {
        Luos_Loop();
        Service1_Loop();
        Service2_Loop();
    }
    return 0;
}

```

This way, it is easy to manage all of your services and to add as many of them as you want in to `main()`.

### How to use Luos hardware abstraction layer

To ease the use of Luos Library on a specific target, the Luos company provides `luos_hal.c`, `luos_hal.h` and `luos_hal_config.h` from the LuosHAL folder for a lot of different MCU families. The purpose of these files is to make Luos communcations work directly on the chosen MCU. However, this default configuration for a MCU Family should be adapted to fit your design. See how to do it on the [Luos HAL](../hardware_topics/luos-hal.md) page.

