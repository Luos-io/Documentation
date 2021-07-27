
# Streaming
On occasion, you will have to deal with high-frequency small data with strong time constraints.

To make it easy, Luos manages streaming channels trough ring buffers.

A streaming channel allows you to drastically reduce the time constraints of your Luos network thanks to 2 factors:
 1) The first factor is a method which allows you to have a *no-real-time* service dealing with a strict-real-time one. Both sides have their own loop frequency and time precision.
      - The real-time one is the service opening the streaming channel. It has a high-frequency function called at a precise **sampling frequency**.
      - The non-real-time one has a slower and unprecise timed function which is called at each **chunk_time**.

 2) By using streaming channels, you will be able to use big data chunks at low frequency to optimize the data rate efficiency of the bus. The idea is to exchange big chunks of data between services instead of large amounts of time-constrained small messages flooding all services.

## Example
A motor-driver service has strict real-time constraints. If you want to have a smooth positioning movement or measurement you have to update the motor position at high-frequency (**sampling frequency**).<br/>
First, you have to define the **sampling frequency** allowing you to have a smooth movement or measurement on the motor. Let's take 200Hz in this example.

In the non-real-time side (the service commanding the motor), you can't have a 200Hz loop because it probably has other things to do and perhaps doesn't have a sufficient time precision. To simplify it you will have to send trajectory chunks regularly (**chunk time**), let's say approximately every 1 second.

Based on those numbers, your data chunk size will be:

```AsciiDoc
chunk_size = chunk_time(s) x sampling_frequency(Hz)
chunk_size = 1 x 200 = 200 samples
 ```
 In our configuration, data chunk needs to be 200 position samples each second, allowing to feed the streaming channel.

 Following our example, if we want to send trajectory to the motor, we will have a *ring buffer* in the motor side managed by the *streaming channel*. Here are the different states of this ring_buffer:
<img src="../../../_assets/img/streaming.png"/>

 1) The service who sends the trajectory has to make sure that the motor service always has data to consume. To do that you have to bootstrap your streaming flux by sending 2 data chunks to start and then send a new data chunk each *chunk_time*.
 This way, the receiver always has at least one data chunk (1s in this example) ready to be consumed.
 2) When data chunks are received, the receiver can start consuming data at its *sampling frequency*.
 3) One data chunk later (1s in our example), the receiver has consumed the first data chunk, and the sender can start to compute the next one.
 4) At the end of the data chunk computation, the sender sends the chunk. Luos adds it to the ring buffer.
 5) At each second, the sender sends the next data chunk and Luos adds it to the ring buffer. At the end of the buffer, Luos puts extra data at the begining. The consumer pointer also goes back to the begining of the buffer when it reaches the end. This way we have infinite data stream without any discontinuity.
 6) You can continue to do this indefinitely.

> **Note:** You can play, pause, stop or record a stream flux with the standard **CONTROL** command using the **control_type_t** structure.

## How to use it
**A streaming channel is always created by the strict real-time service.** The other service (the non-real-time one) will just send or receive its data chunks using [large data messages](./msg-handling.html#large-data).

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
 - **reception** is suited to make our motor move smoothly. A no-real-time service will send us parts of trajectory approximately each second and our motor will consume angular position at 200Hz.
 - **transmission** is suited to measure precisely the movements of the motor. We can use it to send in a non-real-time way real-time data. In our motor it could be angular position measurement at 200Hz for example.

### Streaming reception
This is used to make the motor move.<br/>
When your streaming channel has been created, you can feed it with received messages on your reception callback:
 ```C
void Motor_MsgHandler(service_t *service, msg_t *msg) {
    // check message command
    if (msg->header.cmd == ANGULAR_POSITION) {
        // this is our trajectory reception
        Luos_ReceiveStreaming(service, msg, &trajectory);
    }
}
```
Now your service is able to receive trajectory chunks. For the next step, you need to have a real-time callback (using a timer for example) which is able to manage the consumption of this trajectory at 200hz:
```C
void 200hz_callback(void) {
    Stream_GetSample(&trajectory, &motor.target_angular_position);
}
```

### Streaming transmission
This is used to measure the motor movements.<br/>
To go the other way and send a sampled signal such as a position measurement, you have to use your streaming channel in reception.
First you have to put values into your streaming channel at 200Hz:
```C
void 200hz_callback(void) {
    Stream_PutSample(&trajectory, &motor.angular_position);
}
```
This way, samples are buffered into your ring buffer, and you can send this real-time information as you want:
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
