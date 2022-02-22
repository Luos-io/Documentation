---
custom_edit_url: null
---

import Image from '/src/components/Images.js';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Part 3: Full embedded application

# Summary

1. Introduction
2. Add the package button
3. Find the button Services ID
4. Ask the button value
5. Switch the Led depending on the button value!
6. Try the button value changing
7. Exercice

## 1. Introduction

The purpose of this training is to create a full embedded application that detect all the services in the system and create a switcher the turn on or off a LED depending on a button.

Let’s code that and see how cool it is to do it with Luos!

## 2. Add the package button

For now in the project there is 2 packages! For the next step, we will add the button package into your main.c or Arduino.ino file.

<Tabs>
<TabItem value="Arduino" label="Arduino">

```c
#include <luos.h>
#include "led.h"
// the new line to copy and paste
#include "button.h"
#include "switcher.h"

#ifdef __cplusplus
}
#endif

/******************************************************************************
 * @brief Setup ardiuno
 * @param None
 * @return None
 ******************************************************************************/
void setup()
{
    Luos_Init();
    Led_Init();
	// the new line to copy and paste
    Button_Init();
    Switcher_Init();
}
/******************************************************************************
 * @brief Loop Arduino
 * @param None
 * @return None
 ******************************************************************************/
void loop()
{
    Luos_Loop();
    Led_Loop();
	// the new line to copy and paste
    Button_Loop();
    Switcher_Loop();
}
```

</TabItem>
<TabItem value="Nucleo" label="Nucleo">

```c
#include <luos.h>
#include "switcher.h"
#include "led.h"
// the new line to copy and paste
#include "button.h"

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    Luos_Init();
    SwitcherInit();
    Led_Init();
	// the new line to copy and paste
    Button_Init();

    while (1)
    {
        Luos_Loop();
        Switcher_Loop();
        Led_Loop();
		// the new line to copy and paste
        Button_Loop();
    }
}
```

</TabItem>
</Tabs>

:::caution
You must call the `Luos_Init()` API before any service init to make the library properly work!
:::

:::tip
The actual detection will be performed at `Luos_Loop()` execution. So It’s better to call the `Luos_Loop()` first allowing the services to start working as soon as possible.
:::

## 3. Find the button Services ID

Now the button package was add to our project. we must find its ID to send a message to it!

1. Create a variable where the ID of button service will be save

```c
/*******************************************************************************
 * Variables
 ******************************************************************************/
service_t *switcher_app; // This will be our switcher service
uint16_t ID_Led;
// the new line to copy and paste
uint16_t ID_Button;
```

2. Filter the button service alias in the routing table and ask it about it.

```c
void Switcher_MsgHandler(service_t *service, msg_t *msg)
{
    if (msg->header.cmd == END_DETECTION)
    {
		search_result_t filter_result;
        RTFilter_Reset(&filter_result); // Init your filter.
		// Now your filter_result have the entire routing table. #nofilter ;)
        RTFilter_Alias(&filter_result, "led"); // Filter your filter_result only keep the services with the alias "led"
		ID_Led = filter_result.result_table[0]->id;//recover the first service ID with alias "led"

		// the three new lines to copy and paste
		RTFilter_Reset(&filter_result); // Reset your filter.
		RTFilter_Alias(&filter_result, "button");
		ID_Button = filter_result.result_table[0]->id;

        if (ID_Led > 0)
        {
	        msg_t pub_msg;
	        pub_msg.header.cmd = IO_STATE;
	        pub_msg.header.target_mode = ID;
	        pub_msg.header.target =  ID_Led; // configure the target to be our led service ID
	        pub_msg.header.size = 1;
	        pub_msg.data[0] = 1;
	        Luos_SendMsg(switcher_app, &pub_msg);
		}
    }
}
```

## 4. Ask the button value

Now we have the button service ID, we can easily get the button value by sending a message to button service in `Switcher_Loop`. The button service will reply with a message containing the button value!

If we send this request at every Loop, this will create messages a lot and overload the network. This will consume a lot of RAM and create network congestion!

Let’s space them from 10ms!

1. **Execute request every 10ms**

On your Switcher_loop we will have to do something to be able to send a message every 10ms.

You can use the sysTick to get a 1ms precision date using `Luos_GetSystick();`. This tick give you the opportunity to count how many millisecond are passed!

Let’s create a variable to track the **LastAsk**:

```c
/*******************************************************************************
 * Variables
 ******************************************************************************/
service_t *switcher_app; // This will be our switcher service
uint16_t ID_Led;
uint16_t ID_Button;
// the new line to copy and paste
uint32_t LastAsk;

```

Init this variable at the service start:

```c
void Switcher_Init(void)
{
    revision_t revision = {1, 0, 0};
    switcher_app = Luos_CreateService(Switcher_MsgHandler, SWITCHER_APP, "Switcher", revision);
    Luos_Detect(switcher_app);
	// the new line to copy and paste
    LastAsk = Luos_GetSystick();
}
```

Check if 10ms have passed :

```c
void Switcher_Loop(void)
{
	// the new block to copy and paste
    // ask button value every 10ms
    if ((Luos_GetSystick() - LastAsk) > 10)
    {
        // Ask the button
        LastAsk = Luos_GetSystick();
    }
}
```

2. **Be sure a detection have been made before send a message on a network**

before sending a request value to the button service we have to be sure that a detection have been made. Luos provide a specific API allowing you to know if your system have been detected or not : `bool Luos_IsNodeDetected(void)`

This function return true when your system have been entirely detected :

```c
void Switcher_Loop(void)
{
	// the new block to copy and paste
    if (Luos_IsNodeDetected() == true) // Topology detection completed
    {
		// ask button value every 10ms
		if ((Luos_GetSystick() - LastAsk) > 10)
		{
			LastAsk = Luos_GetSystick();
		}
	}
}
```

3. **Finally ask the button value**

Now everything is ready, we can ask the button to send us it’s value.

```c
void Switcher_Loop(void)
{
    if (Luos_IsNodeDetected() == true) // Topology detection completed
    {
		    // ask button value every 10ms
		    if ((Luos_GetSystick() - LastAsk) > 10)
		    {
				// the new block to copy and paste
				if (ID_Button > 0)
		        {
					// Ask the button
				    msg_t pub_msg;
		            pub_msg.header.cmd = IO_STATE;
		            pub_msg.header.target_mode = ID;
		            pub_msg.header.target = ID_Button;
		            pub_msg.header.size = 0;
		            Luos_SendMsg(switcher_app, &pub_msg);
				}
		        LastAsk = Luos_GetSystick();
		    }
		}
}
```

Now we are asking the button to send us an update every 10ms!

## 5. Switch the Led depending on the button value!

In the `Switcher_Loop()` function we asking the button to send us an update every 10ms. The reply of the button will be received on our `Switcher_MsgHandler` function.

1. **Let’s add a new condition in our message handler allowing us to deal with the button messages**

```c
void Switcher_MsgHandler(service_t *service, msg_t *msg)
{
    if (msg->header.cmd == END_DETECTION)
    {
		search_result_t filter_result;
        RTFilter_Reset(&filter_result); // Init your filter.
		// Now your filter_result have the entire routing table. #nofilter ;)
        RTFilter_Alias(&filter_result, "led"); // Filter your filter_result only keep the services with the alias "led"
		ID_Led = filter_result.result_table[0]->id;//recover the first service ID with alias "led"

		RTFilter_Reset(&filter_result); // Reset your filter.
		RTFilter_Alias(&filter_result, "button");
		ID_Button = filter_result.result_table[0]->id;

		if (ID_Led > 0)
        {
	        msg_t pub_msg;
	        pub_msg.header.cmd = IO_STATE;
	        pub_msg.header.target_mode = ID;
	        pub_msg.header.target =  ID_Led // configure the target to be our led service ID
	        pub_msg.header.size = 1;
	        pub_msg.data[0] = 1;
	        Luos_SendMsg(switcher_app, &pub_msg);
		}
    }
	// the new block to copy and paste
	else if ((msg->header.cmd == IO_STATE) && (msg->header.source == ID_Button))
	{
		// Command the led accordingly to the button message
	}
}
```

To properly switch the led, we have to deal with this button message and send a command to the led!

2. **Put the code that send a message to turn on the led at the end of the detection inside the button reception message condition**

```c
void Switcher_MsgHandler(service_t *service, msg_t *msg)
{
    if (msg->header.cmd == END_DETECTION)
    {
		search_result_t filter_result;
        RTFilter_Reset(&filter_result); // Init your filter.
		// Now your filter_result have the entire routing table. #nofilter ;)
        RTFilter_Alias(&filter_result, "led"); // Filter your filter_result only keep the services with the alias "led"
		ID_Led = filter_result.result_table[0]->id;//recover the first service ID with alias "led"

		RTFilter_Reset(&filter_result); // Reset your filter.
		RTFilter_Alias(&filter_result, "button");
		ID_Button = filter_result.result_table[0]->id;
    }
	else if ((msg->header.cmd == IO_STATE) && (msg->header.source == ID_Button))
	{
		// the new block to copy and paste
		// Command the led accordingly to the button message
		if (ID_Led > 0)
        {
	        msg_t pub_msg;
	        pub_msg.header.cmd = IO_STATE;
	        pub_msg.header.target_mode = ID;
	        pub_msg.header.target =  ID_Led // configure the target to be our led service ID
	        pub_msg.header.size = 1;
	        pub_msg.data[0] = 1;
	        Luos_SendMsg(switcher_app, &pub_msg);
		}
	}
}
```

At this point everytime you receive a message from the button service, you will turn on the Led!

3. **To finish we simply need to send to the led the state of the button. This button value is recover in the first case of tab data of the message button.**

```c
...
	else if ((msg->header.cmd == IO_STATE) && (msg->header.source == ID_Button))
	{
		// Command the led accordingly to the button message
		if (ID_Led > 0)
        {
	        msg_t pub_msg;
	        pub_msg.header.cmd = IO_STATE;
	        pub_msg.header.target_mode = ID;
	        pub_msg.header.target =  ID_Led // configure the target to be our led service ID
	        pub_msg.header.size = 1;
			// the new line to copy and paste
	        pub_msg.data[0] = msg->data[0];
	        Luos_SendMsg(switcher_app, &pub_msg);
		}
	}
...
```

Because we ask the button to update it’s value every 10ms, we will receive a button update every 10ms. At this update reception we will send a new led command, so the led is also updated every 10ms.

This way your button control your led.

## 6. Try the button value changing

1. Compile and upload the project to the board. the led turn on at the end of the detection

2. Push on your button :

<Tabs>
<TabItem value="Arduino" label="Arduino">
To simulate a press button, connect a wire between the BTN_PIN (Pin 8) and GND.
<div align="center">
  <Image src="/img/your-first-message/your-first-message-2-1.png" darkSrc="/img/your-first-message/your-first-message-2-1-dark.png"/>
</div>

</TabItem>
<TabItem value="Nucleo1" label="STM32F072RB Nucleo/STM32F401RE Nucleo/STM32F410RB Nucleo">
Now push on the B1 button (the blue one) on your board
</TabItem>
<TabItem value="Nucleo2" label="STM32G431KB Nucleo/STM32L432KC Nucleo">
To simulate a press button, connect a wire between the BTN_PIN (Pin D10) and GND.
<div align="center">
  <Image src="/img/your-first-message/your-first-message-2-1.png" darkSrc="/img/your-first-message/your-first-message-2-1-dark.png"/>
</div>
</TabItem>
</Tabs>

Congratulation, You create your first full embedded application!!!

<div align="center">
  <img src ="https://media.giphy.com/media/l4q8cJzGdR9J8w3hS/giphy.gif" className="gif_tutorial"/>
</div>
You detect all the service in your project and command them through an application.

## 6. Exercice

Now try to move your Switcher package on another board and keep the button and led package on the first one!

Everything should work the same way!

Asking every 10ms the value of the button represent a not negligible portion of code.

Luos provide an update_time command to simplify it, check documentation [here](/docs/luos-technology/message/advanced-message).
