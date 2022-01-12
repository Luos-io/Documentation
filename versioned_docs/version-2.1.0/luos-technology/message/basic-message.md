---
custom_edit_url: null
---

# Send a basic message

To send a message, you have to:

1.  Create a message variable
2.  Set the **target_mode**
3.  Set the **target**
4.  Set the **cmd**
5.  Set your data **size**
6.  Set your data
7.  Send it.

```c

// Create and fill the message info
msg_t pub_msg;
pub_msg.header.target_mode = ID;
pub_msg.header.target      = msg->header.source;
pub_msg.header.cmd         = IO_STATE;
pub_msg.header.size        = sizeof(char);
pub_msg.data[0]            = 0x01;
Luos_SendMsg(service, &pub_msg);
```

:::info infos

1. The function _Luos_SendMsg_ returns an error_status that informs the user whether the message was stored in a buffer and is ready to be sent. User can monitor the returned value to be sure that the message will be sent.<br/><br/>
2. The _Luos_SendMsg_ function is non blocking. This function loads the message into a Luos memory and Luos, leaded by your hardware, will send it as soon as possible.

:::
