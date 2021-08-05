# Packages

## What are packages used for

As explained in [architecture](../basics/archi.md), luos is a platform that handles packages execution. It also manages what can be called "inter-package communication": the way packages interact with each other.

Traditionnal way of writing code is the "monolithic" way: all functionnalities are used in one big **main()** function and are tighly dependant from each others. This leads to difficult developpement, debug and maintainability when the code base goes bigger. Luos tries to shrink this monolithic architecture into smaller, simpliest blocks of code. It does so by giving to developpers high-level API to create these blocks and make them communicate with each others.

**Packages** represents these blocks of code. They contains independant functionnalities which will be runned by luos.

## Relation with services

From a logicial view, a package handles a functionnality independant from the rest of the system but we don't have any information of how this functionnality should to be executed: it can be runned as a single task or smaller tasks talking to each others. **Services** gives this level of control: a package can initialize as much as services it needs to run its functionnality. However each package has to run at least one service.

## How to properly organize your Luos projects

### How to add packages in your project

A luos [node](../node/node.md) can host multiple packages, and a package has to be as portable as possible. In order to do that, packages have to be independent code folders that can be easily copied and pasted in another project.<br/>
When designing projects at Luos we always use the same way to organize our code: we put packages into a `packages` folder:

```AsciiDoc
 Project
    │
    ├─── Packages
    │    ├─── package_1
    │    │    ├─── package_1.c
    │    │    └─── package_1.h
    │    └─── package_2
    │         ├─── package_2.c
    │         └─── package_2.h
    │
    ├─── Inc
    │    ├─── Luos
    │    └─── luos_hal
    │
    └─── Src
         └─── Main.c
```

### Basic packages functions

We choose to put the public functions of our services in the `package.h` file. As said above, services are lightweight tasks that need to be run regularly. We choose to use the exact same stategy as presented for Luos functions by providing a `Package_Init()` and a `Package_Loop()`: services are created in `Package_Init()` and the application code is placed in `Package_Loop()` (see [service creation](../services/service-api.md) for more informations).

Then packages are initialized and runned in the `main()` function:

```C
#include "luos.h"
#include "package1.h"
#include "package2.h"

int main(void)
{
    Luos_Init();
    Package1_Init();
    Package2_Init();
    while(1)
    {
        Luos_Loop();
        Package1_Loop();
        Package2_Loop();
    }
    return 0;
}

```

This way, it is easy to manage all of your services and to add as many of them as you want in to `main()`.

### How to use Luos hardware abstraction layer

To ease the use of Luos Library on a specific target, Luos provides `luos_hal.c`, `luos_hal.h` and `luos_hal_config.h` from the LuosHAL folder for a lot of different MCU families. The purpose of these files is to make Luos communications work directly on the chosen MCU. However, this default configuration for a MCU Family should be adapted to fit your design. See how to do it on the [Luos HAL](../node/luos-hal.md) page.
