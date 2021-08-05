# Architecture

### Luos Levels

| Level | Description |
| :---: | :---: |
| Product | A node or a set of nodes communicate together through Luos and forming a complete product. |
| Nodes | MCUs executing one or more package code. |
| Package | A sharable folder containing cross-platform code files managing one or more services. |
| Services | Luos services can be Driver or Apps. Each service can follow Luos template and Luos list of command but can be custom too|

![](../../../_assets/img/architecture.png)

### Product architecture with Luos

```AsciiDoc
 Project
  │
  ├─── Product_config.h
  ├─── Node 1
  │    ├─── Node_config.h
  │    ├─── Main.c
  │    ├─── Main.h
  │    ├─── container 1
  │    │     ├─── container_1.c
  │    │     └─── container_1.h
  │    └─── container 2
  │          ├─── container_2.c
  │          └─── container_2.h
  ├─── Node 2
  │    ├─── Node_config.h
  │    ├─── Main.c
  │    ├─── Main.h
  │    └─── container 3
  │          ├─── container_3.c
  │          └─── container_3.h
  │
  └─── Node 3
       ├─── Node_config.h
       ├─── Main.c
       ├─── Main.h
       ├─── container 4
       │     ├─── container_3.c
       │     └─── container_3.h
       ├─── container 4
       │     ├─── container_3.c
       │     └─── container_3.h
       └─── container 5
             ├─── container_2.c
             └─── container_2.h

```


- **Product level**

    **Product_config.h**: this file describes the general configuration for the product such as  Baudrate, timeout duration, etc. It also regroups the list of custom `service_profiles` and custom command that allow services to communicate together

- **Node level**

    **Node_config.h**: this file describes a configuration of the board Pinout, USART for communication, Timer, and DMA. It also configures buffer size for Luos communication and the number of task
    **Main.c/.h** : Main files for the node to call luos.h initialize and call package (all the different Service_Init, Server_Loop)

- **Package level**

    **Package.c/.h**: Contain the Service_Init, Service_Loop, and Service_Msg_Handler. In these files, you can create, with luos API, services dedicated to your function

- **Service level**
    use luos API and Service Profile to create a standard or a custom service that can be used by anybody

### Where is Luos?

Luos Library is use at node level

![](../../../_assets/img/luos_mcu_platform.png)
