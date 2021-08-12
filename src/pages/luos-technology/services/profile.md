# How to use Luos services profiles

## What is a service profile

Now that you understand what a service and a service type are, and how to create them, we can address the subject of profiles.

A **service profile** is pre-made API management for a specific service type.
Basically, a profile gives a service a **data structure** with all the variables you need with the service type you choose to exchange information with others. The profile code knows how to deal with those variables and **share the service information** with any other.

Thanks to profiles, your service code will be clean and straightforward, you will only **need to use variables**, and you won't need to deal with Luos messages and message handling anymore.

> **Example:** If you want to make a servo-motor service, you have to select the servo-motor profile for your service and use variables of the profile structure to set the rotor current position measurement, or to get the current motor target in your code.

Profiles are convenient for making your code clean and straightforward, complying with your development into a standard API, or sharing your service type with the community.
Luos provides some common profile models that you can use; feel free to contribute and to add your own to the standard profile bank with a pull request on <a href="https://github.com/Luos-io" target="_blank">Luos' GitHub page</a>.

Luos allows to create a service based on a specific profile. We call it a **service template**: 

> **service** + **profile** = **template**

## How to create and initialize a service template

To create a template, you have to import the profile corresponding to your needs and instantiate a template profile structure:
```c
#include "template_servo_motor.h"

template_servo_motor_t servo_motor_template;                        // Create the template
profile_servo_motor_t *servo_motor = &servo_motor_template.profile; // Get a pointer to the actual profile
```
As you can see in this example, you have to create a template structure containing a profile. You will need to give this template structure to the Luos template creation function. But in your service code, you will only need to deal with the profile structure, so you can create a pointer on the profile and use it like that on your code.

To actually create the service template on the Luos network, you have to call the creation function specific to your template:
```c
service_t *TemplateServoMotor_CreateService(CONT_CB callback, template_servo_motor_t *var, const char *default_alias, revision_t revision);
```

The returned `service_t*` is a service structure pointer that will be useful to make your service act in the network after this initialization.

- **callback** is a pointer to the same callback function described in [the service management section](./service_api.html#how-to-create-and-initialize-a-service). The main difference between template services and custom services is that you don't need to manage any message in this callback because the profile handles it for you.
 You can use this callback to make your code react to an event or manage custom messages on top of the profile.
- **var** is the template structure pointer you just createed.
- **default alias** is the alias by default for your new service, e.g. `Mytemplate02`. This alias is the one your service will use if no other alias is set by the user of your functionality hosted in your service. Aliases have a maximum size of 16 characters.
- **revision** is the revision number of the service you are creating.

Following the [packages rules](../package/package.html#basic-services-functions), here is a code example for a button template service:

```c
#include "template_state.h"

template_state_t button_template;
profile_state_t *button = &button_template.profile;

void Button_Init(void)
{
    // Profile configuration
    revision_t ButtonRevision = {.unmap = {0,0,7}};
    button->access = READ_ONLY_ACCESS;
    // Service creation following template
    service_btn = TemplateState_CreateService(0, &button_template, "button", ButtonRevision);
}

void Button_Loop(void) {
    button->state = (bool)HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
}
```

You can notice that you don't send any Luos message to share the button state: if an application wants to access this information, the state profile will share it for you. You only have to update the button state value in your code. Supported profiles are available in this [repository](https://github.com/Luos-io/Luos/tree/master/Templates).
