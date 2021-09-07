# Receive and Send Advanced messages

Luos provides advanced sending and receiving mechanisms allowing to deal with various data transmission needs or stategies.

## Large data

You will sometimes have to deal with extensive data that could be larger than the maximum 128-byte data on a Luos message. Fortunately, Luos can automatically fragment and de-fragment the data above this side. To do that, you will have to use another send function that will set the messages' size and the data fields.

For example, here is how to send a picture:

```c
// fill the large message info
msg_t msg;
color_t picture[300*300] = {/*Your favorite cat picture data*/};
msg.header.target_mode = ID_ACK;
msg.header.target = 12;
msg.header.cmd = COLOR;
Luos_SendData(service, &msg, picture, sizeof(color_t)*300*300);
return;
```

In the reception callback, here is the code for retrieve the message with the receiving service (the one with ID 12):

```c
color_t picture[300*300];
void services_MsgHandler(service_t *service, msg_t *msg) {
    if (msg->header.cmd == COLOR) {
        if (Luos_ReceiveData(service, msg, (void*)picture) == SUCCEED)
        {
            // The picture is completely received, you can enjoy your favorite cat picture!
        }
    }
}
```

> **Note:** If you have to deal with high-frequency real-time data, please read [the Streaming section](./advanced-message.html#streaming.md).

## Time-triggered update messages

Luos provides a standard command to ask a service to retrieve values from a sensor, called `ASK_PUB_CMD`. However, sometimes apps need to poll values from sensors. Still, the act of repeatedly retrieving a value using the `ASK_PUB_CMD` command may result in the use of a lot of bandwidth and take up valuable resources.

**You can use the time-triggered auto-update features available from any Luos service** in this kind of polling situation. This feature allows to ask a service to send an update of the `ASK_PUB_CMD` command each chosen amount of milliseconds.

To use it, you have to set up targeted service with a message containing a standard time <span class="cust_tooltip">object dictionary<span class="cust_tooltiptext">{{od_def}}</span></span> but with a specific command.

For example, to update a service each 10 ms:

```C
time_luos_t time = TimeOD_TimeFrom_ms(10);
msg_t msg;
msg.header.target = id;
msg.header.target_mode = IDACK;
TimeOD_TimeToMsg(&time, &msg);
msg.header.cmd = UPDATE_PUB; // Overload the TIME command defined by TimeOD
Luos_SendMsg(app, &msg);
```

> **Info:** services can handle only one time-triggered target, two services of the same network can't ask for a time-triggered value from the same service.

> **Warning:** To prevent any ID movement, auto-update configuration is reset on all services on each detection (see the [routing table page](../node/topology.md) for more information).

## Streaming

On occasion, you will have to deal with small high-frequency data with strong time constraints.

To make it easy, Luos manages streaming channels through ring buffers.

A streaming channel allows you to drastically reduce the time constraints of your Luos network thanks to 2 factors:

1. The first factor is a method that allows to have a *no-real-time* service dealing with a strict-real-time one. Both sides have their own loop frequency and time precision.
    - The real-time one is the service opening the streaming channel. It has a high-frequency function called at a precise **sampling frequency**.
    - The non-real-time one has a slower and unprecise timed function which is called at each **chunk_time**.

2. By using streaming channels, you will be able to use big data chunks at low frequencies to optimize the data rate efficiency of the bus. The idea is to exchange big chunks of data between services instead of large amounts of time-constrained small messages flooding all services.

## Example

A motor-driver service has strict real-time constraints. If you want to have a smooth positioning movement or measurement, you must update the motor position at a high-frequency (**sampling frequency**).

First, you have to define the **sampling frequency** allowing to have a smooth movement or measurement on the motor. Let's take 200 Hz in this example.

On the non-real-time side (the service commanding the motor), you can't have a 200 Hz loop because it probably has other things to do and perhaps doesn't have sufficient time precision. To simplify it, you will have to send trajectory chunks regularly (**chunk time**), let's say approximately every 1 second.

Based on those numbers, your data chunk size will be:

```AsciiDoc
chunk_size = chunk_time(s) x sampling_frequency(Hz)
chunk_size = 1 x 200 = 200 samples
 ```
 In our configuration, data chunk needs to be 200 position samples each second, allowing to feed the streaming channel.

 Following our example, if we want to send trajectory to the motor, we will have a *ring buffer* in the motor side managed by the *streaming channel*. Here are the different states of this ring_buffer:
<img src="../../../_assets/img/streaming.png" />

 1. The service that sends the trajectory must ensure that the motor service always has data to consume. To do that, you have to bootstrap your streaming flux by sending 2 data chunks to start and then send a new data chunk each *chunk_time*.
 This way, the receiver always has at least one data chunk (1s in this example) ready to be consumed.
 2. When data chunks are received, the receiver can start consuming data at its *sampling frequency*.
 3. One data chunk later (1s in our example), the receiver has consumed the first data chunk, and the sender can start to compute the next one.
 4. At the end of the data chunk computation, the sender sends the chunk. Luos adds it to the ring buffer.
 5. The sender sends the next data chunk at each second, and Luos adds it to the ring buffer. At the end of the buffer, Luos puts extra data at the beginning. The consumer pointer also goes back to the beginning of the buffer when it reaches the end. This way, we have infinite data stream without any discontinuity.
 6. You can continue to do this indefinitely.

> **Note:** You can play, pause, stop or record a stream flux with the standard **CONTROL** command using the **control_type_t** structure.

## How to use it

**A streaming channel is always created by the strict real-time service.** The other service (the non-real-time one) will send or receive its data chunks using [large data messages](./advanced-message.html#large-data).

### Streaming channel creation

Before starting to use the streaming method, you have to create a streaming channel linked to a buffer into the init function of your real-time service:

```c
#define BUFFER_SIZE 1024
volatile angular_position_t trajectory_ring_buf[BUFFER_SIZE];
streaming_channel_t trajectory;

void Motor_Init(void) {
    trajectory = Stream_CreateStreamingChannel(trajectory_ring_buf, BUFFER_SIZE, sizeof(angular_position_t));
    Luos_CreateService(Motor_MsgHandler, CONTROLLER_MOTOR_MOD, "motor_mod");
}
```

Now you can use this channel to receive or transmit a streaming flux:

 - **Reception** is suited to make our motor move smoothly. A non-real-time service will send us parts of trajectory approximately each second, and our motor will consume angular position at 200 Hz.
 - **Transmission** is suited to measure precisely the movements of the motor. We can use it to send in a non-real-time way real-time data. In our motor it could be angular position measurement at 200 Hz, for example.

### Streaming reception

The streaming reception is used to make a motor move.

When the streaming channel has been created, you can feed it with received messages on the reception callback:

```C
void Motor_MsgHandler(service_t *service, msg_t *msg) {
    // check message command
    if (msg->header.cmd == ANGULAR_POSITION) {
        // this is our trajectory reception
        Luos_ReceiveStreaming(service, msg, &trajectory);
    }
}
```

Now your service is able to receive trajectory chunks. For the next step, you need to have a real-time callback (using a timer, for example) which can manage the consumption of this trajectory at 200 Hz:

```C
void 200hz_callback(void) {
    Stream_GetSample(&trajectory, &motor.target_angular_position);
}
```

### Streaming transmission

The streaming transmission is used to measure the motor movements.

To go the other way and send a sampled signal such as a position measurement, you have to use your streaming channel in reception.
First, you have to put values into your streaming channel at 200 Hz:

```C
void 200hz_callback(void) {
    Stream_PutSample(&trajectory, &motor.angular_position);
}
```

This way, samples are buffered into your ring buffer, and then you can send this real-time information as you want:

```C
void Motor_MsgHandler(service_t *service, msg_t *msg) {
    msg_t pub_msg;
    // check message command
    if (msg->header.cmd == ASK_PUB_CMD) {
        // prepare a reply message and send
        pub_msg.header.target_mode = ID;
        pub_msg.header.target = msg->header.source;
        pub_msg.header.cmd = ANGULAR_POSITION;
        Luos_SendStreaming(service, &pub_msg, &measurement);
    }
}
```

The `Luos_SendStreaming` function sends available data on your streaming channel. You can continue to feed your channel with samples at the same time.
