# CLRC663_LPCD on nrf52840

this library is aimed to be used on an nrf52 series chip as the main uC to control the nxp clrc663 nfc frontend

almost all the definitions have been copied from [iwanders](https://github.com/iwanders/MFRC630)

# usage

1. implement your own SPI functionality by using nrf spi drivers
2. if you want to use LPCD, add NFC IRQ PIN definition and enable lpcd IRQ bit on the nxp chip
3. all the other definitions and references should be given within the code provided

# dont forget

to read the documentation on tag reading, mifare and iso14443.

you can also read through the by NXP provided library to gather information on the LPCD process (calibration and actual measuring)

