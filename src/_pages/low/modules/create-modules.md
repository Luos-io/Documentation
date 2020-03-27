
# Create Luos modules

**As a developer you will always develop your functionalities into modules and never into the main program.**

> **Warning:** Make sure to read and understand how to [Create Luos projects](/_pages/low/modules/create-project.md) before reading this page.

## How to create and initialize a module

To create a module you simply have to call this function :
```c
module_t* luos_module_create(void* callback, module_type_t type, char* default_alias);
```

The returned `module_t*` is a module structure pointer that will be useful to make your module act in the network after this initialization.


 **callback** is a pointer to a callback function called by Luos when your module receive messages from other modules (see [Real-time configuration page](/_pages/low/modules/rt-config.md) for more details).
 This function needs to have a specific format:

 ```c
 void module_cb(module_t *module, msg_t *msg)
 ```

 - **module** is the module pointer of the module receiving the data (basically, it's your module).
 - **msg** is the message your module received.


 **type** is the type of the your new module represented by a number. Some basic types (e.g. `DISTANCE_MOD`, `VOLTAGE_MOD`, etc.) are already available in the `module_type_t` enum structure of Luos. You can also create your own on top of the luos one.

 **default alias** is the alias by default for your new module. e.g. `MyModule02`. This alias is the one your module will take if no other alias is set by the user of your functionality hosted in your module. Aliases have a maximum size of 16 characters.


Following the [project rules](/_pages/low/modules/create-project.html#basic-modules-functions), here is a code example for a button module:

```c
module_t* module_btn;

void rx_btn_cb(module_t *module, msg_t *msg){
    // Manage received messages
}

void button_init(void) {
    module_t* module_btn = luos_module_create(rx_btn_cb, STATE_MOD, "button_mod");
}

void button_loop(void) {
}
```

> **Note:** According to the real-time configuration you chose, an additional line of code may be necessary. See [Real-time configuration page](/_pages/low/modules/rt-config.md) for more details.

<div class="cust_edit_page"><a href="https://{{gh_path}}/_pages/low/modules/create-modules.md">Edit this page</a></div>
