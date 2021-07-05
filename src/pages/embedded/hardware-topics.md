# Luos network

Luos uses a communication protocol called <span class="cust_tooltip">Robus<span class="cust_tooltiptext">{{robus_def}}</span></span> to communicate with other containers. 
Luos can work on a single node or create a network for the communication between containers located on different nodes. This communication goes through a physical layer, which must be adapted to your own design (through the file `luos_hal_config.h`). 

To create a Luos network, various serial bus technologies can be chosen to fit with your design, however Luos provides 2 default references designed to faciliate Luos Library integration and easy network creation. One design is based on RS485, responding to many technical issues of a serial bus like multimastering, speed, robustness and safety, etc. The second one is based on a usart half-duplex called One-wire.

List of advantages and drawbacks for each bus:

| Network | Advantages | Drawbacks
| :---: | :---: | :---: |
| One-wire | Needs only 4 GPIO; Simple; 100% Collision detection | No recessive bit and no collision avoidance; Short distance wires; Less nodes connectable to network; 1 Mbps |
| RS485 | Noise immunity; Long distance wires; 256 nodes connectable to network; 10 Mbps | Needs 5 GPIO and a RS485 tranceiver; Collision avoidance not secured |
