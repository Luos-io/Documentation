# Use a physical bus

Luos uses a communication protocol called <span class="cust_tooltip">Robus<span class="cust_tooltiptext">{{robus_def}}</span></span> to communicate with other containers. 
Luos can work on a single node or create a network for the communication between containers located on different nodes. This communication goes through a physical layer, which must be adapted to your own design (through the file `luos_hal_config.h`). 

To create a Luos network, many serial bus technologies can be chosen to fit with your design, however Luos provides a default reference designed to faciliate Luos Library integration and easy network creation. this design is based on RS485, responding to many technical issues of a serial bus like multimastering, speed, Robus and safety, etc.
