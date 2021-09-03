# Minimum requirements
Luos Library uses hardware and software MCU resources. Here are the minimum requirements to make Luos Library work properly:

## MCU consideration
| Hardware | Specification |
| :---: | :---: |
| Flash | Luos Library uses less than 20 kB of Flash memory |
| RAM | Luos Library uses at least 2 kB of RAM for Library usage and protocol message buffering |
| Stack | 512 B |
| Heap | 0 needed |
| GPIO | 4 in one wire with 2 ports / 5 in RS485 with 2 ports |
| Resources | 1 USART, 1 Timer (option: DMA and CRC)|

## Network consideration
| MCU frequency | Bus frequency |
| :---: | :---: |
| >=48 MHz | 1 Mbps |
| >=32 MHz | 500 kbps |
| >=16 MHz | 250 kbps |
| >=8 MHz | 100 kbps |
