# Minimum requirements
Luos Library uses hardware and software MCU ressources. Here are the minimum requirements to make Luos Library work properly:

## MCU consideration
| Hardware | Specification |
| :---: | :---: | 
| Flash | Luos Library uses less than 15ko of Flash memory |
| RAM | Luos Library uses at least 2ko of RAM for Library usage and protocol message buffering |
| Stack | 512o | 
| Heap | 0 needed | 
| GPIO | 4 in one wire / 6 in RS485  |
| Ressources | 1 USART, 1 Timer (option : DMA and CRC)|

## Network consideration
| MCU frequency | Bus frequency |
| :---: | :---: | 
| >=48Mhz | 1Mbs |
| >=32Mhz | 500Kbs |
| >=16Mhz | 250Kbs | 
| >=8Mhz | 100Kbs | 
