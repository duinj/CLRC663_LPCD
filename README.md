# CLRC663_LPCD on nrf52840

this library is aimed to be used on a nrf52 series chip as the main uC to control the nxp clrc663 nfc frontend

almost all the definitions have been copied from [iwanders](https://github.com/iwanders/MFRC630)

# usage

1. implement your own SPI functionality by using nrf spi drivers
2. add NFC IRQ PIN definition 
3. all the other definitions and references should be given within the code provided

# dont forget

to read the documentation on tag reading, mifare and iso14443

