# Create Luos Projects
## How to properly organize your Luos projects
### How to make Luos run
Luos is like a task that you will have to run regularly. So you will have to make it run by adding `luos_init()` and `luos_loop()` in the main of your program.<br/>
Basically your main will look like :

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
Putting this code on your node make it able to react to a Luos network. It's now ready to host your modules.

**As a developer you will ALWAYS develop your functionalities into modules and NEVER into the main program.**
> **Note:** The only things that should be put on the main code are MCU setup things and modules run functions.

### How to put modules in your project
A node can host multiple modules, and we want a module as portable as possible. To do that modules have to be independent code folders that can be easily copy and paste in another project.<br/>
To make it at Luos we always use the same way to organize our projects. We put modules on a `modules` folder and name modules code files with the name of the module :

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
At Luos we make our modules always expose the same functions in the `module.h` file. The same as Luos need it, modules are like tasks that you will have to run regularly, so we choose to use the exact same stategy as presented for Luos functions by exposing a `module_init()` and `module_loop()` functions and to add it on the main.
Following the previous folder organization our main code looks like :

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
This way it's easy to manage all your modules and to add it on your main file as you want.
