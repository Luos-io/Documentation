
# Streaming
In occasion, you will have to deal with high-frequency small data with strong time constraints.

To make it easy, Luos manages streaming channels trough ring buffers.

A streaming channel allows you to reduce drastically the time constraints of your Luos network thanks to 2 effects:
 1) The first effect is a method which allows you to have a *no-real-time* container dealing with a strict-real-time one. Both side have their own loop frequency and time precision.
      - The real-time one is the container opening the streaming channel. It has a high-frequency function called at a precise **sampling frequency**.
      - The no-real-time one has a slower and unprecise timed function which is called at each **chunk_time**.

 2) By using streaming channels, you will be able to use big data chunks at low frequency to optimize the data rate efficiency of the bus. The idea is to exchange big chunks of data between containers instead of a tons of time-constrained small messages flooding all containers.

## Example
A motor-driver container has strict real-time constraints. If you want to have a smooth positioning movement or measurement you have to update the motor position at high-frequency (**sampling frequency**).<br/>
First, you have to define the **sampling frequency** allowing you to have a smooth movement or measurement on the motor. Let's take 200Hz in this example.

In the no-real-time side (the container commanding the motor), you can't have a 200Hz loop because it probably has other things to do and perhaps doesn't have a sufficient time precision. To simplify it you will have to send trajectory chunks regularly (**chunk time**), let's say approximately each 1 second.

Based on those numbers, your data chunk size will be:

```AsciiDoc
chunk_size = chunk_time(s) x sampling_frequency(Hz)
chunk_size = 1 x 200 = 200 samples
 ```
 In our configuration, data chunk needs to be 200 position samples each second, allowing to feed the streaming channel.

 Following our example, if we want to send trajectory to the motor, we will have a *ring buffer* in the motor side managed by the *streaming channel*. Here are the different states of this ring_buffer:
<img src="/_assets/img/streaming.png"/>

 1) The container who sends the trajectory has to make sure that the motor container always has data to consume. To do that you have to bootstrap your streaming flux by sending 2 data chunks to start and then send a new data chunk each *chunk_time*.
 This way, receiver always has at least one data chunk (1s in this example) ready to be consumed.
 2) When data chunks are received, receiver can start consuming data at its *sampling frequency*.
 3) One data chunk later (1s in our example), receiver has consumed the first data chunk, and the sender can start to compute the next one.
 4) At the end of the data chunk computation, sender sends the chunk. Luos adds it to the ring buffer.
 5) At each second, the sender sends the next data chunk and Luos add it to the ring buffer. At the end of the buffer, Luos puts extra data at the begining. The consumer pointer also goes back to the begining of the buffer when it reaches the end. This way we have infinite data stream without any discontinuity.
 6) You can continue this way indefinitely.

> **Note:** You can play pause, stop or record stream flux with the standard **CONTROL** command using the **control_type_t** structure.

## How to use it
**A streaming channel is always created by the strict real-time container.** The other container (the no-real-time one) will just send or receive its data chunks using [large data messages](/pages/low/containers/msg-handling.html#large-data).

### Streaming channel creation
Before starting using streaming method, you have to create a streaming channel linked to a buffer into the init function of your real-time container:

```c
#define BUFFER_SIZE 1024
volatile angular_position_t trajectory_ring_buf[BUFFER_SIZE];
streaming_channel_t trajectory;

void motor_init(void) {
    trajectory = create_streaming_channel(trajectory_ring_buf, BUFFER_SIZE, sizeof(angular_position_t));
    luos_container_create(rx_mot_cb, CONTROLLED_MOTOR_MOD, "motor_mod");
}
```

Now you can use this channel to receive or transmit a streaming flux:
 - **reception** is adapted to make our motor move smoothly. A no-real-time container will send us parts of trajectory approximately each second and our motor will consume angular position at 200Hz.
 - **transmission** is adapted to measure precisely the movements of the motor. We can use it to send in a no-real-time way real-time data. In our motor it could be angular position measurement at 200Hz for example.

### Streaming reception
This is used to make the motor move.<br/>
When your streaming channel has been created, you can feed it with received messages on your reception callback:
 ```C
void rx_mot_cb(container_t *container, msg_t *msg) {
    // check message command
    if (msg->header.cmd == ANGULAR_POSITION) {
        // this is our trajectory reception
        luos_receive_streaming(container, msg, &trajectory);
    }
}
```
Now your container is able to receive trajectory chunks. For the next step, you need to have a real-time callback (using a timer for example) which is able to manage the consumption of this trajectory at 200hz:
```C
void 200hz_callback(void) {
    get_sample(&trajectory, &motor.target_angular_position);
}
```

### Streaming transmission
This is used to measure the motor movements.<br/>
To go the other way and send a sampled signal such as a position measurement, you have to use your streaming channel in reception.
First you have to put values into your streaming channel at 200Hz:
```C
void 200hz_callback(void) {
    set_sample(&trajectory, &motor.angular_position);
}
```
This way, samples are buffered into your ring buffer, and you can send this real-time information as you want. For example only when someone ask you to:
 ```C
void rx_mot_cb(container_t *container, msg_t *msg) {
    msg_t pub_msg;
    // check message command
    if (msg->header.cmd == ASK_PUB_CMD) {
        // prepare a reply message and send
        pub_msg.header.target_mode = ID;
        pub_msg.header.target = msg->header.source;
        pub_msg.header.cmd = ANGULAR_POSITION;
        luos_send_streaming(container, &pub_msg, &measurement);
    }
}
```
The `luos_send_streaming` function sends data available on your streaming channel. You can continue to feed your channel with samples at the same time.

> **Warning:** This example doesn't work if your container is configured as real-time. Please read [Real-time configuration page](/pages/low/containers/rt-config.md) for more informations.

<div class="cust_edit_page"><a href="https://{{gh_path}}/pages/low/containers/streaming.md">Edit this page</a></div>
