# How to use Luos services profiles

## What is a service profile

Now that you understand what a service and a service type are, and how to create them, we can address the subject of profiles.

A **service profile** is pre-made API management for a specific **service type**. Basically, a profile gives a **data structure** with all variables you need to handle a specific  service type. The profile code knows how to deal with those variables and share them with any other services.

Thanks to this feature, your variables are shared and accessible by the entire system **in real time** without explicit message handling in the service code.

> **Example:** If you want to make a servo-motor service, you have to include the servo-motor profile for your service and use variables of the profile structure to set the rotor current position measurement, or to get the current motor target in your code.

Profiles are convenient for making your code clean and straightforward, complying with your development into a standard API, or sharing your service type with the community.
Luos provides some common profile models that you can use; feel free to contribute and to add your own to the standard profile bank with a pull request on <a href="https://github.com/Luos-io" target="_blank">Luos' GitHub page &#8599;</a>.

## How to use a profile in your service

To use this feature, you have to include the profile corresponding to your needs and instantiate a structure it brings to you:
```c
#include "profile_servo_motor.h"

profile_servo_motor_t servo_motor; // create a motor profile structure
```
Now your structure exists, you can access all variables it embeds. But it won't be updated by luos as described above, for this one step remains: you have to create a service and tell luos that you're using a profile and you want the platform handles it for you.

As far, you learned to create a bare service with the **luos_CreateService()** call, but we don't talk about giving informations to the platform about the profile you instantiated. To do so, every profile comes with a **Profile_CreateService()** function that will make it for you. This function is very similar to the **luos_CreateService()** routine except for the `type` field replaced by the profile you previously created.

To create a service with a **motor profile** for example the dedicated routine is showed here :

```c
service_t *ProfileServo_CreateService(SERVICE_CB callback, profile_servo_motor_t *profile_servo_motor, const char *alias, revision_t revision);
```

The returned `service_t*` is a service structure pointer that will be useful to make your service act in the network.

- **callback** is a pointer to the same callback function described in [the service management section](./service_api.html#how-to-create-and-initialize-a-service). The main difference between profile services and custom services is that you don't need to manage any message in this callback because the profile handles it for you.
 You can use this callback to make your code react to an event or manage custom messages on top of the profile.
- **profile_servo_motor** is the profile structure pointer you just created with a type depending on the profile you are using.
- **alias** is the alias by default for your new service, e.g. `Myprofile02`. This alias is the one your service will use if no other alias is set by the user of your functionality hosted in your service. Aliases have a maximum size of 16 characters.
- **revision** is the revision number of the service you are creating.

Following the [packages rules](../package/package.html#basic-services-functions), here is a code example for a button service using a state profile:

```c
#include "profile_state.h"

profile_state_t button;

void Button_Init(void)
{
    // service initialization
    revision_t revision = {.major = 1, .minor = 0, .build = 0};
    // Profile configuration
    button.access = READ_ONLY_ACCESS;
    // Service creation following state profile
    ProfileState_CreateService(&button, 0, "button", revision);
}

void Button_Loop(void) {
    button.state = (bool)HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
}
```

You can notice that you don't send any Luos message to share the button state: if an application wants to access this information, the state profile will share it for you. You only have to update the button state value in your code. Supported profiles are available in this [repository](https://github.com/Luos-io/Luos/tree/master/Profiles).
