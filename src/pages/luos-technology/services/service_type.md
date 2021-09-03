# Types of services

As explained in the [service page](./service_api.html#messages), Luos enables message exchanges between services in a straightforward way for the developers. But to easily use a service you need to know their interface, or how to interact with them.

In your code, to know what are the capabilities and the purpose of any services of your product you need to know their types.

## Standard interface between services

Types allow to define the interface of a services. Types create a **common API** between services.

Let's take an example with the most straightforward type available in Luos: STATE_TYPE defines the capability to manage state msg format. With this information, each application that wants to send a message to the LED driver knows how to communicate with this service: it just has to put the state information, and the LED can be controlled.

To define the capabilities of a specific type you can use or create [profiles](./profile.md). Profiles are API definition that can be applied to a service.
