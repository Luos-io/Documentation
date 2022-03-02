---
hide_table_of_contents: true
custom_edit_url: null
---

# Types of services

As explained in the [service page](./service-api#messages), Luos enables message exchanges between services in a straightforward way for developers. Still, to easily use a service you need to know their interfaces or how to interact with them.

In your code, in order to know what are the capabilities and the purpose of any services of your product, you need to know their types.

## Standard interface between services

Types allow to define the interface of a service. Types create a **common API** between services.

The following example uses the most straightforward type available in Luos: STATE_TYPE defines the capability to manage state messages format. With this information, each application that wants to send a message to the LED driver knows how to communicate with this service; it only has to put the state information, and the LED can then be controlled.

To define the capabilities of a specific type, you can use or create [profiles](./profile). Profiles are API definitions that can be applied to a service.
