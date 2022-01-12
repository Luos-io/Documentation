---
custom_edit_url: null
---

import IconExternalLink from '@theme/IconExternalLink';

# Packages

## What are packages used for

As explained in the [architecture page](/luos-technology/basics/orga.md), Luos is a platform that handles packages execution.

The traditional way of writing code is still "monolithic": all functionalities are used in one big `main()` function and are tightly dependent with each other. This leads to complex development, debug, and maintainability needs when the code base grows bigger. It also leads to complex collaboration with other developers. Luos tries to separate this monolithic architecture into independant and <a href="https://en.wikipedia.org/wiki/Loose_coupling" target="_blank">weak-coupled<IconExternalLink width="10" /></a> blocks of code. It does so by giving to developers high-level APIs to create these blocks and make them communicate with each other.

**Packages** represent these blocks of code. They contain independent functionalities which will be run by Luos.

## Relation with services

From a logical view, a package handles one or a group of functionalities, independent from the rest of the system. Still, we don't have any information on how thess functionalities should be executed: it can be run either as a single task or as smaller tasks talking to each other. **Services**, however, give this control level: a package can initialize as much services as it needs to run its functionalities.

> **Note:** Each package has to run at least one service.

## How to properly organize your Luos projects

### How to add packages in your project

A Luos [node](/luos-technology/node/node.md) can host multiple packages, and a package has to be as portable as possible. In order to do that, packages have to be independent code folders that can be easily copied and pasted in another project.

When designing projects at Luos, we always use the same way to organize our code: we put packages into a `lib` folder on a node project, and every packages have their own `package` folder allowing them to be easily copied and pasted into any other node project:

```AsciiDoc
 Node
    │
    ├─── lib
    │    ├─── package_1
    │    │    ├─── package_1.c
    │    │    └─── package_1.h
    │    └─── package_2
    │         ├─── package_2.c
    │         └─── package_2.h
    │
    ├─── inc
    │    ├─── luos
    │    └─── luos_hal
    │
    └─── src
         └─── main.c
```

### Basic packages functions

We choose to put the public functions of our services in the `package.h` file. As said above, services are lightweight tasks that need to be run regularly. We choose to use the exact same strategy as presented for Luos functions by providing a `Package_Init()` and a `Package_Loop()`: services are created in `Package_Init()`, and the application code is placed in `Package_Loop()` (see [service creation page](/luos-technology/services/service-api.md) for more information).

Then packages are initialized and run in the `main()` function:

```c
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
