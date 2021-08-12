# Packages

## What are packages used for

As explained in the [architecture page](../basics/archi.md), Luos is a platform that handles packages execution. It also manages what can be called "inter-package communication", which is the way packages interact with each other.

The traditional way of writing code is the "monolithic" way: all functionalities are used in one big `main()` function and are tightly dependant on each other. This leads to complex development, debug and maintainability when the code base grows bigger. Luos tries to shrink this monolithic architecture into smaller and more simple blocks of code. It does so by giving to developers high-level APIs to create these blocks and make them communicate with each other.

**Packages** represent these blocks of code. They contain independent functionalities which will be run by Luos.

## Relation with services

From a logical view, a package handles a functionality independent from the rest of the system. Still, we don't have any information on how this functionality should be executed: it can be run as a single task or smaller tasks talking to each other. **Services** give this control level: a package can initialize as much as services it needs to run its functionalities. However, each package has to run at least one service.

## How to properly organize your Luos projects

### How to add packages in your project
A Luos [node](../node/node.md) can host multiple packages, and a package has to be as portable as possible. In order to do that, packages have to be independent code folders that can be easily copied and pasted in another project.

When designing projects at Luos we always use the same way to organize our code: we put packages into a `packages` folder:

```AsciiDoc
 Node
    │
    ├─── Lib
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
We choose to put the public functions of our services in the `package.h` file. As said above, services are lightweight tasks that need to be run regularly. We choose to use the exact same strategy as presented for Luos functions by providing a `Package_Init()` and a `Package_Loop()`: services are created in `Package_Init()`, and the application code is placed in `Package_Loop()` (see [service creation page](../services/service_api.md) for more information).

Then packages are initialized and run in the `main()` function:

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

This way, it is easy to manage all of your services and add as many of them into `main()`.
