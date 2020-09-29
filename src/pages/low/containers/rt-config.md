# Real-time configuration

> **Warning:** Make sure to read and understand how to [Create Luos containers](/pages/low/containers/create-containers.md) before reading this page.

Message callbacks of containers can be really difficult to use when a project include high real-time constraints.<br/>
Luos provides different "real-time" configurations allowing you to choose the best way for you to deal with messages.
The configuration of real-time is set during the [initialization of a container](/pages/low/containers/create-containers.md).

|Configuration|execution type|
|:---:|:---:|
|[No real-time (default)](#no-real-time-configuration)|runtime callback|
|[Polling](#polling-configuration)|no callback|
|[Real-time](#real-time-configuration)|interrupt callback|

The following sections detail how the different configurations work.

## No real-time configuration
This configuration is the default and most common setup. In this configuration, Luos calls the container callback during runtime (not interrupt). The time between the physical reception of a message and the callback may vary depending on the `luos_loop()` function call frequency.<br/>
With this configuration, you have no real constraints on the callback's time of execution, you can reply to a message directly on the callback.

To setup this configuration you have to simply setup the callback at container creation.

Here is a code example with a button:
```c
void rx_btn_cb(container_t *container, msg_t *msg) {
    if (msg->header.cmd == ASK_PUB_CMD) {
        // The message is filled with global variable with proper data
        msg_t pub_msg;
        pub_msg.header.cmd = IO_STATE;
        pub_msg.header.target_mode = ID;
        pub_msg.header.target = msg->header.source;
        pub_msg.header.size = sizeof(char);
        pub_msg.data[0] = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
        // Sending the message
        luos_send(container, &pub_msg);
        return;
    }
}

void button_init(void) {
    // container creation: (callback, container type, Default alias)
    container_t* container = luos_container_create(rx_btn_cb, STATE_MOD, "button_mod");
}

void button_loop(void) {
}
```

## Polling configuration
This configuration is often used into Arduino libraries to receive information in a basic way. This method allows to manage the messages only when the user wants to do it on the loop of the container.

To setup this configuration, you have to create your container without any callback.

See the following code as an example, with a button:

```c
container_t* container;
void button_init(void) {
    container = luos_container_create(0, STATE_MOD, "button_mod");
}

void button_loop(void) {
    if (luos_message_available()) {
        msg_t *msg = luos_read(container);
        if (msg->header.cmd == ASK_PUB_CMD) {
            // The message is filled with global variable with proper data
            msg_t pub_msg;
            pub_msg.header.cmd = IO_STATE;
            pub_msg.header.target_mode = ID;
            pub_msg.header.target = msg->header.source;
            pub_msg.header.size = sizeof(char);
            pub_msg.data[0] = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
            // Sending the message
            luos_send(container, &pub_msg);
        }
    }
}
```

## Real-time configuration
This method is adapted to high-frequency messages updates and requires an advanced understanding of embedded code and hardware capabilities. This configuration is not set by default.

In this configuration, container's callback is called in interruption. It means that if the execution time is too long into container's callback, it will block all the other interruptions and any other messages reception. The callback execution must be ended before any new message.

With this configuration you don't have time to send a reply back in the callaback. The response will then have to be deported into the main loop.

In order to use this configuration, you have to setup the callback at container creation and enable the real time mode.

Here is a code example with a button:
```c
static container_t *container_pointer;
static volatile msg_t pub_msg;
static volatile int pub = LUOS_PROTOCOL_NB;

void rx_btn_cb(container_t *container, msg_t *msg) {
    // /!\ execution in interruption
    if (msg->header.cmd == ASK_PUB_CMD) {
        // The message is filled with global variable with proper data
        pub_msg.header.cmd = IO_STATE;
        pub_msg.header.target_mode = ID;
        pub_msg.header.target = msg->header.source;
        pub_msg.header.size = sizeof(char);
        pub_msg.data[0] = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
        // The request is saved to be executed into the regular program
        pub = ASK_PUB_CMD;
        container_pointer = container;
        return;
    }
}

void button_init(void) {
    //  container creation: (callback, container type, Default alias)
    container_t* container = luos_container_create(rx_btn_cb, STATE_MOD, "button_mod");

    // Hard real-time activation
    luos_container_enable_rt(container);
}

void button_loop(void) {
    if (pub != LUOS_PROTOCOL_NB) {
        // Sending the generated message into the callback
        luos_send(container_pointer, &pub_msg);
        pub = LUOS_PROTOCOL_NB;
    }
}
```

<div class="cust_edit_page"><a href="https://{{gh_path}}/pages/low/containers/rt-config.md">Edit this page</a></div>
