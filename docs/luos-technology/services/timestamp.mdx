---
hide_table_of_contents: true
custom_edit_url: null
---

# Timestamp

**Timestamp** is a feature that enables you to track the date at which an event occurs. You can track as many events as you want, timing informations are saved in the system with the value associated to events.

Let's take an example with a button driver: each time we read the button state, we want to timestamp this event. First we have to create a global variable called a **token**, it's used to track our event in the system:

```C
#include "timestamp.h"

timestamp_token_t button_timestamp;
```

Then we can **tag** our data related to the button event with this token:

```C
ll_button_read(&button.state);
Timestamp_Tag(&button_timestamp, &button.state);
```
**Timestamp_Tag** links the token to the data. This enables several features:
- if you use button.state in an another function in your service, you can check if there is a timestamp linked to the data and get it from the token with **Timestamp_GetToken** and **Timestamp_GetTimeFromToken** functions.
- if you send button.state through a luos message, the timestamp will be copied in the message. Then you can get it from the message in the receiving service with the **Timestamp_DecodeMsg** function.

All **timestamp** API functions can be found in Luos/Robus/inc/timestamp.h. You can also find [examples](https://github.com/Luos-io/Examples/blob/master/Projects/l0/) which are using this feature. For now, examples using timestamp are : 
- button
- distance
- light sensor
- potentiometer

## Timestamp API

|                        Description                         |                                       Function                                        |      Return      |
| :--------------------------------------------------------: | :-----------------------------------------------------------------------------------: | :--------------: |
|   Tag a data  with a token    |   `Timestamp_Tag(timestamp_token_t *token, void *target);`    |   `void`  |
|   Get a token from a data     |   `Timestamp_GetToken(void *target);` | `timestamp_token_t *` |
|   Check if a message is timestamped     |   `Timestamp_IsTimestampMsg(msg_t *msg);` | `bool` |
|   Update the timestamp in a message     |   `Timestamp_TagMsg(msg_t *msg);` | `void` |
|   Encode a message with the timestamp protocol     |   `Timestamp_EncodeMsg(msg_t *msg, void *target);` | `void` |
|   Decode a timestamped message     |   `Timestamp_DecodeMsg(msg_t *msg, uint64_t *timestamp);` | `void` |