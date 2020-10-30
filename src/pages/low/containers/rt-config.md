# Message Handling configuration

> **Warning:** Make sure to read and understand how to [Create Luos containers](/pages/low/containers/create-containers.md) before reading this page.

Message callbacks of containers can be really difficult to use when a project include high real-time constraints.<br/>
Luos provides two different configurations allowing you to choose the best way for you to deal with messages.
The message handling configuration is set during the [initialization of a container](/pages/low/containers/create-containers.md).

|Configuration|execution type|
|:---:|:---:|
|[Callback (default)](#Callback-configuration)|runtime callback|
|[Polling](#polling-configuration)|no callback|

The following sections detail how the different configurations work.

## Callback configuration
This configuration is the default and most common setup. In this configuration, Luos directly calls the container callback during runtime. The time between the physical reception of a message and the callback may vary depending on the `luos_loop()` function call frequency.<br/>
With this configuration, you have no real constraints on the callback's time of execution, you can reply to a message directly on the callback.

To setup this configuration you have to simply setup the callback at container creation.

Here is a code example with a button:
```c
void Button_MsgHandler(container_t *container, msg_t *msg) {
    if (msg->header.cmd == ASK_PUB_CMD) {
        // The message is filled with global variable with proper data
        msg_t pub_msg;
        pub_msg.header.cmd = IO_STATE;
        pub_msg.header.target_mode = ID;
        pub_msg.header.target = msg->header.source;
        pub_msg.header.size = sizeof(char);
        pub_msg.data[0] = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
        // Sending the message
        Luos_SendMsg(container, &pub_msg);
        return;
    }
}

void Button_Init(void) {
    // container creation: (callback, container type, Default alias)
    container_t* container = Luos_CreateContainer(Button_MsgHandler, STATE_MOD, "button_mod");
}

void Button_Loop(void) {
}
```

## Polling configuration
This configuration is often used into Arduino libraries to receive information in a basic way. This method allows to manage the messages only when the user wants to do it on the loop of the container.

To setup this configuration, you have to create your container without any callback.

See the following code as an example, with a button:

```c
container_t* container;
void Button_Init(void) {
    container = Luos_CreateContainer(0, STATE_MOD, "button_mod");
}

void Button_Loop(void) {
    if (Luos_NbrAvailableMsg()) {
        msg_t *msg = Luos_ReadMsg(container);
        if (msg->header.cmd == ASK_PUB_CMD) {
            // The message is filled with global variable with proper data
            msg_t pub_msg;
            pub_msg.header.cmd = IO_STATE;
            pub_msg.header.target_mode = ID;
            pub_msg.header.target = msg->header.source;
            pub_msg.header.size = sizeof(char);
            pub_msg.data[0] = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
            // Sending the message
            Luos_SendMsg(container, &pub_msg);
        }
    }
}
```


