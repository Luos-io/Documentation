# Luos Communication bus

Luos uses a communication protocol called <span class="cust_tooltip">Robus<span class="cust_tooltiptext">{{robus_def}}</span></span> to communicate with other containers. 
Luos can work on a single node or create a network for the communication between containers located on different nodes. This communication goes through a physical layer, which must be adapted to your own design (through the file `luos_hal_config.h`). 

To create a Luos network, many serial bus technologies can be chosen to fit with your design, however Luos provides 2 default reference designed to faciliate Luos Library integration and easy network creation. One design is based on RS485, responding to many technical issues of a serial bus like multimastering, speed, Robus and safety, etc. The second one based on a usart half duplex call one wire

Advantage and inconvenient:

| Network | Advantage | inconvenient
| :---: | :---: | :---: |
| OneWire | Need only 4 GPIO. Simple. 100% Collision detection | no recessive bit/no collision avoidance. short distance wire. Less node connectable to network. 1Mbps |
| RS485 | Noise immunity. Long distance. 256 nodes connectable to network. 10Mbps  | Need 6 GPIO and RS485 tranceiver. Collision avoidance not secure|

