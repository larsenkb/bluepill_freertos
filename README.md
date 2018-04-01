## freertos v9.0.0 for stm32f103vct6

This repo contains freertos release 9.0.0 configured for a stm32f103vct6 board.
It should run on any stm32f103 with a few changes.

There are two tasks running. One is blinking two LEDs, the other is sending 
stuff via USART1.

USART1 also is configured to receive in interrupt-mode. it collects the 
received chars and if 'GBL\r' was sent, the controller will jump to it's
bootloader.
See gbl.py for how to send, usart.c for how to receive and jump to bootloader.

