BSP for Wyres "W_PROTO" card
----------------------------

Objective : support W_PROTO card on MyNewt 1.7.0
Card hardware:
 - STM32L151CC MCU
 - 256kb flash / 32kb RAM / 8kb PROM : MCU based 
 - UART(1) : MCU based : external grove connector
 - SPI(2) : MCU based : 1 dedicated for radio, 1 on header
 - I2C(1) : MCU based : external grove connector
 - Accelero : XX via I2C
 - Altimeter : XX via I2C
 - MEMS microphone on I2S bus
 - Semtech Lora radio SX1272 on SPI bus
 - 2 LEDs via GPIO on-board and via header
 - 1 'power' PWM GPIO (mosfet switched) on header
 - 2 GPIO on header

Functionality configurable/initialised in BSP

UART : 
 - UART0 : 1 -> creates 'uart0' device : 'real' UART1 of STM32 connected to grove header
 - UARTDBG : 1 -> creates 'uartdbg' s/w bitbang uart on any gpio for debug log output only (19200 baud only)
    - recommended pin : button input header (PB3)

Periphs
 - TIMER1 : 1 -> used as os timer @1MHz frequency
 - I2C1 : 1 -> enables I2C1 in STM32, creates HAL device for 'i2c0', creates devices for accelero and altimeter
 - SPI_0_MASTER : 1 -> creates SPI1 in STM32, creates 'spi0' device for SX1272

Flash layout
 - 16Kb : bootloader (mcuboot recommended)
 - 100Kb x 2 : 2 image slots
