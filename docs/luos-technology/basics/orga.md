---
custom_edit_url: null
---

import Image from '/src/components/Images.js';

# Code organization

Because Luos allows you to manage your features as sharable blocks of code, we organized code in a specific way allowing to easily integrate and share packages from your projects with as little friction as possible.

### Luos Levels

Different levels on the embedded code are defined, corresponding to the different levels of information needed across your entire product.

|  Level  |                                                                            Description                                                                            |
| :-----: | :---------------------------------------------------------------------------------------------------------------------------------------------------------------: |
| Product | A node or a set of nodes that communicate with each other. This is the place where you will have the configurations and information intended for all your boards. |
|  Node   |                                   This is the board's code. This is your actual Eclipse, PlatformIO, IAR, or any other project.                                   |
| Package |                                              A sharable folder containing code files managing one or more services.                                               |
| Service |                       Luos services can be drivers or apps. Each service can follow Luos profiles to be standard, but it can be custom too.                       |

<div align="center">
    <Image src="/img/architecture.svg" darkSrc="/img/architecture-dark.svg"/>
</div>

### Product code organization with Luos

```AsciiDoc
 Project
  │
  ├─── product_config.h
  ├─── Node_1
  │    ├─── node_config.h
  │    ├─── main.c
  │    ├─── main.h
  │    ├─── Package_1
  │    │     ├─── package_1.c
  │    │     └─── package_1.h
  │    └─── Package_2
  │          ├─── package_2.c
  │          └─── package_2.h
  ├─── Node_2
  │    ├─── node_config.h
  │    ├─── main.c
  │    ├─── main.h
  │    └─── Package_3
  │          ├─── package_3.c
  │          └─── package_3.h
  │
  └─── Node_3
       ├─── node_config.h
       ├─── main.c
       ├─── main.h
       ├─── Package_4
       │     ├─── package_4.c
       │     └─── package_4.h
       ├─── Package_5
       │     ├─── package_5.c
       │     └─── package_5.h
       └─── Package_6
             ├─── package_6.c
             └─── package_6.h

```

- **Product level**

  **product_config.h**: This file describes the general configuration for the product, such as baudrate, timeout duration, etc. It also groups the list of custom types, profiles, and commands that allow services to communicate together. This folder is the same for all the nodes of the same product and should be included at the node level.

- **Node level**

  **node_config.h**: This file describes the configuration of the board's pinout, USART for communication, timer, and DMA. It also configures the buffer size for Luos communication and the number of tasks.
  **main.c/.h**: The main files for the node that calls _luos.h_, initializes and calls the package (all the different Service_Init, Service_Loop).

- **Package level**

  **package.c/.h**: Contains the Service_Init, Service_Loop, and Service_Msg_Handler. In these files, you can create services dedicated to your functions with Luos API.

- **Service level**

  Uses Luos API and Service Profile on your package's code to create either a standard or a custom service that anybody can use.

### Where is Luos?

Luos Library is used at the node level. You will need to initialize Luos and call a loop function on your main program to be able to use the Luos API on your packages.

<div align="center">
    <Image src="/img/luos_mcu_platform.svg" darkSrc="/img/luos_mcu_platform-dark.svg"/>
</div>
