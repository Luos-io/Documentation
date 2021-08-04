# Receive and send a basic message

To send a message you have to:
 1) Create a message variable
 2) Set the **target_mode**
 3) Set the **target**
 4) Set the **cmd**
 5) Set your data **size**
 6) Set your data
 7) Send it.

Below is a basic reply example that you can find in a service reception callback. For more information on handling a message received, see message handling configuration page.
```c
void services_MsgHandler(service_t *service, msg_t *msg) {
    if (msg->header.cmd == ASK_PUB_CMD) {
        // fill the message info
        msg_t pub_msg;
        pub_msg.header.target_mode = ID;
        pub_msg.header.target = msg->header.source;
        pub_msg.header.cmd = IO_STATE;
        pub_msg.header.size = sizeof(char);
        pub_msg.data[0] = 0x01;
        Luos_SendMsg(service, &pub_msg);
        return;
    }
}
```
> **Note:** the function _Luos_SendMsg_ return an error_status that inform the user if the message was stored in buffer and is ready to be sent. User can monitor the return value and make a blocking statement to be sure that the message is ready to be sent.

```c
while(Luos_SendMsg(service, &pub_msg) ! SUCCEED);
```
