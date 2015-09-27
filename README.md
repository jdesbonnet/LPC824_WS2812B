# LPC824_WS2812B

Sample project to drive WS2812B RGB LED modules using the SPI port of a LPC82x MCU. 

Many code examples of driving WS2812B utilize 'bit banging' in a tight loop, but because of the relatively high data rate this makes performing any other tasks by the MCU difficult. 

This example uses the SPI peripheral and DMA to drastically reduce the MCU load, allowing it to perform other tasks while the LEDs are being programmed.
