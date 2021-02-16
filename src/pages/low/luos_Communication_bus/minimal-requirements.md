# Minimum Requirements
Luos Library will use hardware and software MCU ressource. this is the minimum requirement to make Luos Library work properly

## MCU consideration

| Hardware | Specification |
| :---: | :---: | 
| Flash | Luos Library use less than 15k of Flash |
| RAM | Luos Library use at least 2k of RAM for Library usage and protocol message buffering |
| Stack | 512o | 
| Heap | 0 needed | 
| GPIO | 4 in one wire/ 6 in RS485  |
| Ressources | 1 USART, 1 Timer (option :DMA and CRC)|

## Network consideration
| MCU frequency | Bus frequency |
| :---: | :---: | 
| >=48Mhz | 1Mbs |
| >=32Mhz | 500Kbs |
| >=16Mhz | 250Kbs | 
| >=8Mhz | 100Kbs | 
