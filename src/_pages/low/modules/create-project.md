# Create Luos Projects
## How to properly organize your Luos projects
### How to run Luos
Luos is like a task that has to be ran regularly. So you will have to mrun it by adding `luos_init()` and `luos_loop()` in the `main()` of your program.<br/>
Basically, your `main()` will look like this:

```C
#include "luos.h"

int main(void)
{
    luos_init();
    while(1)
    {
        luos_loop();
    }
    return 0;
}

```
Putting this code into a <span class="cust_tooltip">node<span class="cust_tooltiptext">{{node_def}}</span></span> makes it able to react to a Luos network. It's now ready to host your modules.

**As a developer you will always develop your functionalities into modules and never into the `main()` program.**

> **Note:** The only information that should be put on the `main()` code are MCU setup parameters and modules' run functions.

### How to add modules in your project
A node can host multiple modules, and a module has to be as portable as possible. In order to do that, modules have to be independent code folders that can be easily copied and pasted in another project.<br/>
To make it at Luos we always use the same way to organize our projects: we put the modules into a `modules` folder and name the modules' code files with the name of each module:

```AsciiDoc
 Project
    │
    ├─── modules
    │    ├─── module_1
    │    │    ├─── module_1.c
    │    │    └─── module_1.h
    │    └─── module_2
    │         ├─── module_2.c
    │         └─── module_2.h
    │
    ├─── Inc
    │    ├─── Luos
    │    └─── Robus
    │
    └─── Src
         └─── Main.c
```

### Basic modules functions
We choose to put the public functions of our modules in the `module.h` file. Like Luos, modules are similar to tasks that need to be run regularly, so we choose to use the exact same stategy as presented for Luos functions by providing a `module_init()` and a `module_loop()` functions and to add them in the `main()`.
Following the previous folder organization, the `main()` code looks like this:

```C
#include "luos.h"
#include "module_1.h"
#include "module_2.h"

int main(void)
{
    luos_init();
    module_1_init();
    module_2_init();
    while(1)
    {
        luos_loop();
        module_1_loop();
        module_2_loop();
    }
    return 0;
}

```

This way, it's easy to manage all your modules and to add as many of them in the `main()` file as you want.

<div class="cust_edit_page"><a href="https://{{gh_path}}/_pages/low/modules/create-project.md">Edit this page</a></div>
