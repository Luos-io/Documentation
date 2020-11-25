
# Create Luos containers
**As a developer you will always develop your functionalities into containers and never into the `main()` program.**

> **Warning:** Make sure to read and understand how to [Create Luos projects](/pages/low/containers/create-project.md) before reading this page.

## How to create and initialize a container

To create a container, you have to call this function:
```c
container_t* Luos_CreateContainer(void* callback, container_type_t type, char* default_alias, char* firm_revision);
```

The returned `container_t*` is a container structure pointer that will be useful to make your container act in the network after this initialization.

 **callback** is a pointer to a callback function called by Luos when your container receive messages from other containers (see [Real-time configuration page](/pages/low/containers/rt-config.md) for more details).
 This function needs to have a specific format:

 ```c
 void Container_MsgHandler(container_t *container, msg_t *msg)
 ```

 - **container** is the container pointer of the container receiving the data (basically, it is your container).
 - **msg** is the message your container received.

 **type** is the type of the your new container represented by a number. Some basic types (e.g. `DISTANCE_MOD`, `VOLTAGE_MOD`, etc.) are already available in the `container_type_t` enum structure of Luos. You can also create your own on top of the luos one.

 **default alias** is the alias by default for your new container. e.g. `Mycontainer02`. This alias is the one your container will take if no other alias is set by the user of your functionality hosted in your container. Aliases have a maximum size of 16 characters.

**firm_revision** is the version number of the container you are creating and which will be accessible via pyluos.

Following the [project rules](/pages/low/containers/create-project.html#basic-containers-functions), here is a code example for a button container:

```c
container_t* container_btn;

static void Button_MsgHandler(container_t *container, msg_t *msg){
    // Manage received messages
}

void Button_Init(void) {
	//STRINGIFY (VERSION) is used to get the container version in the container's library.json file
    container_t* container_btn = Luos_CreateContainer(Button_MsgHandler, STATE_MOD, "button_mod", STRINGIFY(VERSION));
}

void Button_Loop(void) {
}
```

> **Note:** According to the real-time configuration you chose, an additional line of code may be necessary. See [Real-time configuration page](/pages/low/containers/rt-config.md) for more details.

## Containers categories
To make your development as clean as possible, you have to understand in which category ([**Driver**](#drivers-guidelines) or [**App**](#apps-guidelines)) each container of the project is.

By following the categories guidelines, you will be able to make clean and reusable functionalities.

## Drivers guidelines
A driver is a type of container that drives hardware. Motors, distance sensors, LEDs are all drivers.

By designing a driver, you have to keep the following rules in mind:

 - A driver container always uses a standard Luos type to be usable by any other containers.
 - A driver container always uses standard <span class="cust_tooltip">object dictionary<span class="cust_tooltiptext">{{od_def}}</span></span> structures to be usable by any other containers.
 - A driver container never depends or uses any other containers (driver or app).
 - A driver container is "dumb", as it can't do anything else than manage its hardware feature (but it does it very well).

 You can have multiple driver containers on the same <span class="cust_tooltip">node<span class="cust_tooltiptext">{{node_def}}</span></span> managing different hardware functionalities of your board, it is your call to sort them depending on your design.

## Apps guidelines
An applications or app is a type of container that only manages software items such as functions or algorithms. Apps use other containers to make your device act, operate, and behave.
Apps can be placed in any <span class="cust_tooltip">[nodes](/pages/overview/general-basics.html#what-is-a-node)<span class="cust_tooltiptext">{{node_def}}</span></span> on a Luos network without any hardware or code modifications. However, the choice of the hosting node can impact global performances of the system.

By designing an app, you have to keep the following rules in mind:

 - An app can't have hardware dependencies.
 - An app can use custom container types.
 - An app must use standard <span class="cust_tooltip">object dictionary<span class="cust_tooltiptext">{{od_def}}</span></span> structures. If the structures used are not standard, Gate containers could be completely unable to manage them.

Apps are the embedded smartness of your device, and at least one of them should run a network detection in order to map every containers in every nodes in your device and make it work properly. Go to [Routing table](/pages/low/containers/routing-table.md) page for more informations.


