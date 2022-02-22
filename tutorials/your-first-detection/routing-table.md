---
custom_edit_url: null
---

import Image from '/src/components/Images.js';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Part 2: How to use the routing Table

# Summary

1. Introduction
2. Dynamic and scaling
3. Searching things in the routing table

## 1. Introduction

We just learn how to detect a Luos network. But what is the point of making a detection if we have to count the service number into the network to find the ID corresponding to the service we want?

<div align="center">
  <img src ="https://media.giphy.com/media/z1GQ9t8FxipnG/giphy-downsized-large.gif" className="gif_tutorial"/>
</div>

Don’t worry, that’s why the routing table exists.
Lets’s discover how to dynamically find an ID using it!

## 2. How routing table can make your code extremely flexible?

In a classic multi-board system, to send messages between entities (most of the time boards), you have to define a list of addresses for any of them allowing you to target messages.

If something moves on your system, you will have to update this list and probably re-compile all your board projects with this new list.

But most importantly all your code will be specific to this particular configuration, and it will be almost impossible to reuse any piece of your code in any other project.

The Luos detection dynamically assigns ID for you depending on the physical topology and regroup all the information regarding services in a Routing table.

The routing table is a kind of database with all the nodes and services information available and after detection, every service has access to it.

Instead of having defined ID on your system, you can dynamically find them to make your service work in any condition allowing dynamic, portable, and scalable systems.🤯

## 3. Searching things in the routing table

To find things on this routing table, Luos engine provides some filtering function allowing you to extract the services depending on some criteria. You can cumulate those filtering functions to create complex research and find the perfect resource fitting your needs.

:::warning
To learn more about routing table filters please feel free to check out [the related documentation page](/docs/luos-technology/services/routing-table).
:::

Instead of directly writing the ID of the led at the end of the detection, let’s find it and see how we can make it easily scale!

One of the characteristics of our led service is its alias.

In your Switcher.c file, on the Switcher_MsgHandler let’s find the service with the “led” alias to send it a message.

1. Create a variable where the ID of led service will be save

```c
/*******************************************************************************
 * Variables
 ******************************************************************************/
service_t *switcher_app; // This will be our switcher service
// the new line to copy and paste
uint16_t ID_Led;
```

1. Use routing table filter to find the id of the led service by its alias
   - Create a filter result variable that store the result after filtering
   - Reset this filter to get back the complet routing table
   - Apply you filter

```c
void Switcher_MsgHandler(service_t *service, msg_t *msg)
{
    if (msg->header.cmd == END_DETECTION)
    {
		// the five new lines to copy and paste
		search_result_t filter_result;
        RTFilter_Reset(&filter_result); // Init your filter.
		// Now your filter_result have the entire routing table. #nofilter ;)
        RTFilter_Alias(&filter_result, "led"); // Filter your filter_result only keep the services with the alias "led"
		ID_Led = filter_result.result_table[0]->id;//recover the first service ID with alias "led"

		if (ID_Led > 0)
        {
	        msg_t pub_msg;
	        pub_msg.header.cmd = IO_STATE;
	        pub_msg.header.target_mode = ID;
			// the new line to copy and paste
	        pub_msg.header.target =  ID_Led // configure the target to be our led service ID
	        pub_msg.header.size = 1;
	        pub_msg.data[0] = 1;
	        Luos_SendMsg(switcher_app, &pub_msg);
		}
    }
}
```

:::info
A field **result_nbr (**see **[search_result_t](/docs/luos-technology/services/routing-table)** structure**)** give you the match number of your research.
For example, in our system only one service have an alias “led” so **filter_result.result_nbr = 1;** the return of the filtering (ID of led) is place the first case of a tab: **filter_result.result_table[0].**
Using this filtering we can easily adapt our switcher code to be able to control as many led services we have on the network by sending the message in a loop until the **result_nbr** is match
:::

Compile and upload the project to the board, and the LED turns on at the end of the detection.

<div align="center">
  <img src ="https://media.giphy.com/media/zcCGBRQshGdt6/giphy.gif" className="gif_tutorial"/>
</div>
