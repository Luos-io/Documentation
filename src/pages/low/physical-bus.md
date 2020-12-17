# Use a physical bus

Luos uses a communication protocol call <span class="cust_tooltip">Robus<span class="cust_tooltiptext">{{robus_def}}</span></span> to communicate with other containers. 
Luos can work on a single node or creates a network for the communication between containers located on different nodes. This communication goes through a physical layer, which must be adapted to your own design (through the file `luos_hal_config.h`). 

To create a Luos network many serial bus technology can be choose to fit with your design, However Luos provide a default reference design to faciliate Luos Library integration and easy network creation. this design is based on RS485 responding to many technical issues of a serial bus like Multimastering, speed, robus and safety etc.