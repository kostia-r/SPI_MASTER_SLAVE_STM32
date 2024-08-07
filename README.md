SPI data exchange between two STM.
- On button click Master sends command byte to Slave over the SPI and receives 6-bytes answer from Slave.
- Slave receives 1 command byte from Master over the SPI and responses with 6 bytes back to Master.
- The button on the Master board works via interrupts with debounce (PA0).
- On-board LEDs on the Master and Slave switch with each data exchange.

Master: STM32F103C6T6A, SPI1 (polling mode), 4MHz.
Slave:  STM32F411CEU6,  SPI1 (polling mode), 4MHz.
The CS pin is managed by SW.