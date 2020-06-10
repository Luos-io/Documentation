
# Create Luos modules
**As a developer you will always develop your functionalities into modules and never into the `main()` program.**

> **Warning:** Make sure to read and understand how to [Create Luos projects](/pages/low/modules/create-project.md) before reading this page.

## How to create and initialize a module

To create a module, you have to call this function:
```c
module_t* luos_module_create(void* callback, module_type_t type, char* default_alias, char* firm_revision);
```

The returned `module_t*` is a module structure pointer that will be useful to make your module act in the network after this initialization.

 **callback** is a pointer to a callback function called by Luos when your module receive messages from other modules (see [Real-time configuration page](/pages/low/modules/rt-config.md) for more details).
 This function needs to have a specific format:

 ```c
 void module_cb(module_t *module, msg_t *msg)
 ```

 - **module** is the module pointer of the module receiving the data (basically, it is your module).
 - **msg** is the message your module received.

 **type** is the type of the your new module represented by a number. Some basic types (e.g. `DISTANCE_MOD`, `VOLTAGE_MOD`, etc.) are already available in the `module_type_t` enum structure of Luos. You can also create your own on top of the luos one.

 **default alias** is the alias by default for your new module. e.g. `MyModule02`. This alias is the one your module will take if no other alias is set by the user of your functionality hosted in your module. Aliases have a maximum size of 16 characters.

**firm_revision** is the version number of the module you are creating and which will be accessible via pyluos.

Following the [project rules](/pages/low/modules/create-project.html#basic-modules-functions), here is a code example for a button module:

```c
module_t* module_btn;

void rx_btn_cb(module_t *module, msg_t *msg){
    // Manage received messages
}

void button_init(void) {
	//STRINGIFY (VERSION) is used to get the module version in the module's library.json file
    module_t* module_btn = luos_module_create(rx_btn_cb, STATE_MOD, "button_mod", STRINGIFY(VERSION));
}

void button_loop(void) {
}
```

> **Note:** According to the real-time configuration you chose, an additional line of code may be necessary. See [Real-time configuration page](/pages/low/modules/rt-config.md) for more details.

## Modules categories
To make your development as clean as possible, you have to understand in which category ([**Driver**](#drivers-guidelines) or [**App**](#apps-guidelines)) each module of the project is.

By following the categories guidelines, you will be able to make clean and reusable functionalities.

## Drivers guidelines
A driver is a type of module that drives hardware. Motors, distance sensors, LEDs are all drivers.

By designing a driver, you have to keep the following rules in mind:

 - A driver module always uses a standard Luos type to be usable by any other modules.
 - A driver module always uses standard <span class="cust_tooltip">object dictionary<span class="cust_tooltiptext">{{od_def}}</span></span> structures to be usable by any other modules.
 - A driver module never depends or uses any other modules (driver or app).
 - A driver module is "dumb", as it can't do anything else than manage its hardware feature (but it does it very well).

 You can have multiple driver modules on the same <span class="cust_tooltip">node<span class="cust_tooltiptext">{{node_def}}</span></span> managing different hardware functionalities of your board, it is your call to sort them depending on your design.

## Apps guidelines
An applications or app is a type of module that only manages software items such as functions or algorithms. Apps use other modules to make your device act, operate, and behave.
Apps can be placed in any <span class="cust_tooltip">[nodes](#node)<span class="cust_tooltiptext">{{node_def}}</span></span> on a Luos network without any hardware or code modifications. However, the choice of the hosting node can impact global performances of the system.

By designing an app, you have to keep the following rules in mind:

 - An app can't have hardware dependencies.
 - An app can use custom module types.
 - An app must use standard <span class="cust_tooltip">object dictionary<span class="cust_tooltiptext">{{od_def}}</span></span> structures. If the structures used are not standard, Gate modules could be completely unable to manage them.

Apps are the embedded smartness of your device, and at least one of them should run a network detection in order to map every modules in every nodes in your device and make it work properly. Go to [Routing table](/pages/low/modules/routing-table.md) page for more informations.

<div class="cust_edit_page"><a href="https://{{gh_path}}/pages/low/modules/create-modules.md">Edit this page</a></div>
