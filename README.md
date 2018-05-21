# nfc05_uart_reader_c

This repo contains the code of the source code of the STSW-ST25R002 of STM (http://www.st.com/content/st_com/en/products/embedded-software/st25-nfc-rfid-software/stsw-st25r002.html) modified to be used by the
STM32L476rg_Nucleo and the X-NUCLEO-NFC05A1. The control interface is not USB as on the original firmware
for the ST25R3911B-DISCO but is redirected over UART2 to the ST-LINK UART.

A accompaning python project implements the host side.