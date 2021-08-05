# Create Luos services

**As a developer you will always develop your functionalities into services and never into the `main()` program.**

> **Warning:** Make sure to read and understand the [package](../package/package.md) section before reading this page.

## How to create and initialize a service

To create a service, you have to call this function:

```c
service_t* Luos_CreateService(void* callback, service_type_t type, char* default_alias, revision_t revision);
```

The returned `service_t*` is a service structure pointer that will be useful to make your service act in the network after this initialization.

**callback** is a pointer to a callback function called by Luos when your service receive messages from other services (see [messages](../message/message.md) for more details).
This function needs to have a specific format:

```c
void Service_MsgHandler(service_t *service, msg_t *msg)
```

- **service** is the service pointer of the service receiving the data (basically, it is your service).
- **msg** is the message your service received.

**type** is the type of the your new service represented by a number. Some basic types (E.g. `DISTANCE_MOD`, `VOLTAGE_MOD`, etc.) are already available in the `service_type_t` enum structure of Luos. You can also create your own on top of the Luos one.

**default alias** is the alias by default for your new service. E.g. `Myservice02`. This alias is the one your service will use if no other alias is set by the user of your functionality hosted in your service. Aliases have a maximum size of 16 characters.

**revision** is the revision number of the service you are creating and which will be accessible via Pyluos.

Following the [packages rules](../package/package.md), here is a code example for a button service:

```c
service_t* service_btn;

static void Button_MsgHandler(service_t *service, msg_t *msg)
{
    // Manage received messages
}

void Button_Init(void)
{
    revision_t ButtonRevision = {.major = 0, .minor = 0, .build = 7};

    service_btn = Luos_CreateService(Button_MsgHandler, STATE_TYPE, "button", ButtonRevision);
}

void Button_Loop(void)
{
}
```

## Services categories

To make your development as clean as possible, you have to understand in which category ([**Driver**](#drivers-guidelines) or [**App**](#apps-guidelines)) each service of the project is.

By following the categories guidelines, you will be able to make clean and reusable functionalities.

## Drivers guidelines

A driver is a type of service that handles hardware. Motors, distance sensors, LEDs are all drivers.

By designing a driver, you have to keep the following rules in mind:

- A driver service always uses a standard Luos type to be usable by any other services.
- A driver service always uses standard <span className="cust_tooltip">object dictionary<span className="cust_tooltiptext">{{od_def}}</span></span> structures to be usable by any other services.
- A driver service never depends on or uses any other services (driver or app).
- A driver service is "dumb", as it can't do anything else than manage its hardware feature (but it does it very well).

You can have multiple driver services on the same <span className="cust_tooltip">node<span className="cust_tooltiptext">{{node_def}}</span></span> managing different hardware functionalities of your board, it is up to you to sort them depending on your design.

## Apps guidelines

An application or app is a type of service that only manages software items such as functions or algorithms. Apps use other services to make your device act, operate, and behave.
Apps can be placed in any <span className="cust_tooltip">[nodes](../node/node.md)<span className="cust_tooltiptext">{{node_def}}</span></span> on a Luos network without any hardware or code modifications. However, the choice of the hosting node can impact global performances of the system.

By designing an app, you have to keep the following rules in mind:

- An app can't have hardware dependencies.
- An app can use custom service types.
- An app must use standard <span className="cust_tooltip">object dictionary<span className="cust_tooltiptext">{{od_def}}</span></span> structures. If the structures used are not standard, Gate services could be completely unable to manage them.

Apps are the embedded smartness of your device, and at least one of them should run a network detection in order to map every services in every nodes in your device and make it work properly. Go to the [Routing table](./routing-table.md) page for more information.

## Services accessibility

Luos can define and manage the accessibility of services.

This accessibility allows you to specify the access the services can deal with. For example, a STATE_TYPE service (which can handle a basic True/False state) can be used either for a button (read-only) or for a relay (write-only).

By default, when you create a new service, it will be on **READ_WRITE_ACCESS**, telling any other services that they can "send to" or "receive from" this new service. You can change this configuration if you want to.

Services can have the following accessibility:

- READ_WRITE_ACCESS
- READ_ONLY_ACCESS
- WRITE_ONLY_ACCESS
- NO_ACCESS

For example, from the previous initialization example function of the button service, we should specify the accessibility of the service as **READ_ONLY_ACCESS**:

```c
service_t* service_btn;

void Button_Init(void)
{
    revision_t ButtonRevision = {.major = 0, .minor = 0, .build = 7};

    service_btn = Luos_CreateService(Button_MsgHandler, STATE_TYPE, "button", ButtonRevision);

    service_btn->access = READ_ONLY_ACCESS;
}
```

This doesn't change anything else on your service code, as it just allows external services to know the accessibility of your service.

## Messages

The core of luos technology enables to send and receive messages from services with a very simple API. More informations about [messages](../message/basic-message.md) can be found in the dedicated section.
