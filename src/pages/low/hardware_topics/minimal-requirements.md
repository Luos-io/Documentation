# Minimum requirements
Luos Library uses hardware and software MCU ressources. Here are the minimum requirements to make Luos Library work properly:

## MCU consideration
| Hardware | Specification |
| :---: | :---: | 
| Flash | Luos Library uses less than 15 kB of Flash memory |
| RAM | Luos Library uses at least 2 kB of RAM for Library usage and protocol message buffering |
| Stack | 512 B | 
| Heap | 0 needed | 
| GPIO | 4 in one wire / 6 in RS485  |
| Ressources | 1 USART, 1 Timer (option : DMA and CRC)|

## Network consideration
| MCU frequency | Bus frequency |
| :---: | :---: | 
| >=48 Mhz | 1 Mbps |
| >=32 Mhz | 500 kbps |
| >=16 Mhz | 250 kbps | 
| >=8 Mhz | 100 kbps | 
