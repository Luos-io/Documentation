
# Modules communication is based on messages
> **Warning:** Make sure to read and understand how to [Create Luos modules](/_pages/low/modules/create-project.md) before reading this page.

As a developer, you will have to create and use Luos messages to exchange informations between modules. In order to do that, you have to understand how messages works.

## Message structure

Luos messages are managed by the `msg_t` structure:

```C
typedef struct{
    header_t header;
    unsigned char data[MAX_DATA_MSG_SIZE];
}msg_t;
```

All messages have a header. A header is a 7-byte field containing all information allowing modules to understand messages context. All modules on the network catch and decode the header of each sent and received message.

`data` is a table containing informations.

> **Info:** MAX_DATA_MSG_SIZE represenst the maximum size of messages (default value is 128 bytes);

## Header
To send data to any modules you want, you will have to fill some information on the header.

here is the `header_t` structure:
```C
typedef struct{
    unsigned short protocol : 4;       /*!< RESERVED Protocol version. */
    unsigned short target : 12;        /*!< Target address, it can be (ID, Multicast/Broadcast, Type). */
    unsigned short target_mode : 4;    /*!< Select targeting mode (ID, ID+ACK, Multicast/Broadcast, Type). */
    unsigned short source : 12;        /*!< Source address, it can be (ID, Multicast/Broadcast, Type). */
    unsigned char cmd;                 /*!< msg definition. */
    unsigned short size;                /*!< Size of the data field. */
}header_t;
```

- **Protocol (4 bits)**: This field provides the protocol revision. This field is automatically filled, you don't have to deal with it.
- **Target (12 bits)**: This field contains the target address. Make sure to understand the real destination of this field, you have to know the addressing mode contained on the *Target_mode* field.
- **Target_mode (4 bits)**: This field indicates the addressing mode and how to understand the *Target* field. It can take different values:
  - **ID**: This mode allows to communicate with a unique module using its ID **without** acknowledgment return.
  - **ID_ACK**: This mode allows to communicate with a unique module using its ID **with** acknowledgment return.
  - **Multicast/Broadcast**: This mode allows multiple modules to catch a message. In this case, the message contains a type of data used by multiple modules.
  - **Type**: This mode sends a message to all modules with a given type, for example all "Sharp digital distance sensor".
- **Source (12 bits)**: The unique ID of the transmitter module.
- **CMD (8 bits)**: The command defines the transmitted data's type.
- **Size (16 bits)**: Size of the incoming data.

# Receive and send a basic message
To send a message you have to:
 1) Create a message variable
 2) Set the **target_mode**
 3) Set the **target**
 4) Set the **cmd**
 5) Set your data **size**
 6) Set your data
 7) Send it.

Here is a basic reply example that you can find in a module reception callback:
```c
void modules_cb(module_t *module, msg_t *msg) {
    if (msg->header.cmd == ASK_PUB_CMD) {
        // fill the message info
        msg_t pub_msg;
        pub_msg.header.target_mode = ID;
        pub_msg.header.target = msg->header.source;
        pub_msg.header.cmd = IO_STATE;
        pub_msg.header.size = sizeof(char);
        pub_msg.data[0] = 0x01;
        luos_send(module, &pub_msg);
        return;
    }
}
```

## Large data
You will sometimes have to deal with large data that could be larger than the maximum 128-byte data on a Luos message. Fortunately, Luos is able to automatically fragment and de-fragment the data above this side. To do that, you will have to use another send function that will take care of setting the messages' size, and the data fields.

For example, here is how to send a picture:
```c
// fill the large message info
msg_t msg;
color_t picture[300*300] = {/*Your favorite cat picture data*/};
msg.header.target_mode = ID_ACK;
msg.header.target = 12;
msg.header.cmd = COLOR;
luos_send_data(module, &msg, picture, sizeof(color_t)*300*300);
return;
```

In the reception callback, here is the code for retrieve the message with the receiving module (the one with ID 12):
```c
color_t picture[300*300];
void modules_cb(module_t *module, msg_t *msg) {
    if (msg->header.cmd == COLOR) {
        luos_get_data(module, msg, (void*)picture);
    }
}
```


<div class="cust_edit_page"><a href="https://{{gh_path}}/_pages/low/modules/msg-handling.md">Edit this page</a></div>
