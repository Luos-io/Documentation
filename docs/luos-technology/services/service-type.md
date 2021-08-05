# Use type in services

In the previous sections, basics of service creation and initialization were explained, but an argument of the **Luos_CreateService** function was used without further informations: this section aims to give more information about the **type** feature.

## Standard interface between services

As explained in [service](./service_api.html#messages), Luos enables message exchanges between services in a very simple way for the developper.

Types are a useful feature used to standardize the way services exchanges these messages: if one implement a specific type, it will be able to handle all messages from any service with the same type. Types creates a **common API** between services.

Let's take an example with the simplest type available in luos: the STATE_TYPE. This type was already used to initalize services in [service](./service-api.md). This type can be used to control a LED driver : send false to this driver and the led is turned-off, send true and it is turned-on.

STATE_TYPE defines that the first data byte of each message contains the true/false information in a uint8 format. With this information, each application that wants to send a message to the led driver knows how to communicate with this service: it just has to put the state information in the first byte of the message and the LED can be controlled.

As all LED drivers implements this API, it can be change without disturbing the system: applications will continue to send messages whith the same format so new driver will be able to handle them.

Types can be used with [profile](./profile.md) to share the API implementation between several services.
