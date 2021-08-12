# Using types in services

In the previous sections, the basics of service creation and initialization were explained. Still, an argument of the **Luos_CreateService** function was used without further information: this section aims to give more details about the **type** feature.

## Standard interface between services

As explained in the [service page](./service_api.html#messages), Luos enables message exchanges between services in a straightforward way for the developers. 

Types are a helpful feature used to standardize how services exchange these messages: if one implements a specific type, it will handle all messages from any service with the same type. Types create a **common API** between services.

Let's take an example with the most straightforward type available in Luos: the STATE_TYPE. This type was already used to initialize services in [service](./service_api.md). This type can be used to control a LED driver: send false to this driver and the LED is turned-off, send true and it is turned-on. 

STATE_TYPE defines that the first data byte of each message contains the true/false information in a uint8 format. With this information, each application that wants to send a message to the LED driver knows how to communicate with this service: it just has to put the state information in the first byte of the message, and the LED can be controlled.

As all LED drivers implement this API, it can be changed without disturbing the system: applications will continue to send messages with the same format so that new drivers will be able to handle them.

Types can be used with [profiles](./profile.md) to share the API implementation between several services.