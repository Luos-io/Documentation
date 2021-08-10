# Application doesn't retrieve driver's data after pluging in a Gate to my system.

### Symptom(s) 

Detail the synptom(s) here.

### Explanation

Explain why there is an issue

### Resolution

Detail how to solve the issue

### Example

Give an example

#### Initial code

```C
uint16_t id_led, id_but;
void LedControl_MsgHandler(container_t *container, msg_t *msg)
{
    if (msg->header.cmd == IO_STATE)
    {
        // This is a message from the button, update the led state
        msg_t pub_msg;
        pub_msg.header.target      = id_led;
        pub_msg.header.target_mode = IDACK;
        pub_msg.header.cmd         = IO_STATE;
        pub_msg.header.size        = 1;
        pub_msg.header.data[0]     = msg->header.data[0];
        Luos_SendMsg(container, &pub_msg);
    }
}
 void LedControl_Init(void)
{
    revision_t revision = {.unmap = [0.0.1]};
    container_t* app = Luos_CreateContainer(LedControl_MsgHandler, LEDCONTROL_APP, "App_LedControl", revision);
    //Detect all containers of the network and create a routing_table
    RoutingTB_DetectContainers(app);
    //Get the ID of our LED from the routing table
    id_led = RoutingTB_IDFromAlias("myled");
    id_but = RoutingTB_IDFromAlias("mybutton");
    //Auto-update messages
    msg_t msg;
    msg.header.target      = id_but;
    msg.header.target_mode = IDACK;
    time_luos_t time       = TimeOD_TimeFrom_ms(100.0);
    TimeOD_TimeToMsg(&time, &msg);
    msg.header.cmd = UPDATE_PUB;
    Luos_SendMsg(app, &msg);
}
```

#### Solution


```C
 void LedControl_Loop(container_t *container, msg_t *msg)
    {
        static uint16_t previous_id = -1; allowing us to monitor the detection state
        // Check the detection status.
    if (RoutingTB_IDFromContainer(app) == 0)
    {
        // No ID, meaning either no detection occured, or a detection is occuring right now.
        if (previous_id == -1)
        {
            // This is the start period, we have to make a detection.
            // Be sure the network is powered-up 20 ms before starting a detection
            if (Luos_GetSystick() > 20)
            {
                // No detection occured, do the detection
                RoutingTB_DetectContainers(app);
            }
        }
        else
        {
            // A detection is being made, we let it finish.
            // reset the previous_id state to be ready to setup container at the end of detection:
            previous_id = 0;
        }
    }
        else
        {
            if (RoutingTB_IDFromContainer(app) != previous_id)
            {
                // This is the first loop execution after a detection, here goes the initial configuration:
                    
            //Get the ID of our LED from the routing table
            id_led = RoutingTB_IDFromAlias("myled");
            id_but = RoutingTB_IDFromAlias("mybutton");
            //Auto-update messages
            msg_t msg;
            msg.header.target      = id_but;
            msg.header.target_mode = IDACK;
            time_luos_t time       = TimeOD_TimeFrom_ms(100.0);
            TimeOD_TimeToMsg(&time, &msg);
            msg.header.cmd = UPDATE_PUB;
            Luos_SendMsg(app, &msg);
            }
        }
    }

```

<hr>

> **Associated documentation page(s):** 
> - [Page 1](https://google.com)
> - [Page 2](https://google.com)
> - [Page 3](https://google.com)

> **Associated documentation page(s):** 
> No associated page