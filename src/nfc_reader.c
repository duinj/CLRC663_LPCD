/*
NFC READER LIB MAIN FILE 
31.03.2021
This file includes some basic functions to use an NFC reader
functions implement basic SPI communication between uC and used IC frontend (in this case NXPs CLRC663)

Basically you dont communicate directly with the ports in the IC frontend but write to its specified registers to execute commands
registers can be found in file clrc663_def.h, all the specified registers and values are taken from clrc663_datasheet or NXP application notes
*/

#include "nfc_reader.h"
#include "clrc663.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "string.h"
#define MANUFACTURER_KEY 0xF9
#define LPCD 1

//handle irq from nfc irq pin (the irq sets a flag which is continually checked in main function
static void on_clrc_pin_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  if (pin == NFC_IRQ_PIN) {
    read_LPCDIRQ_FLAG = 1;
  }
}

//lpcd read flag
volatile uint8_t read_LPCDIRQ_FLAG = 0;

void NFCReader_init() {

//NFC IRQ PIN has to be set
#ifdef LPCD
  nrfx_gpiote_in_config_t icm_int2 = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);  
  nrfx_gpiote_in_init(NFC_IRQ_PIN, &icm_int2, on_clrc_pin_event_handler);
  nrfx_gpiote_in_event_enable(NFC_IRQ_PIN, true);
#endif

//pdown and lvl shift enable are necessary for usage on dvk
  nrf_gpio_cfg(CLRC_PDOWN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_pin_clear(CLRC_PDOWN);
  nrf_gpio_cfg(CLRC_LVL_SHIFT_ENABLE, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_pin_clear(CLRC_LVL_SHIFT_ENABLE);
  nrf_delay_ms(100);

  //this is a basic WHOAMI routine. 
  uint8_t version = CLRC_read_reg(CLRC663_REG_VERSION);
  if (version != 0x1a) {
    CLRC663_PRINTF("no clrc chip found");
  } else {
    //setting drivemode and recommended registers.
    //register descriptions can be read in documentation
    //taken from adafruit frontend reader lib
    CLRC_write_multi(CLRC663_REG_DRVMOD, &MILLER_MANCHESTER_MODULATION[0], sizeof(MILLER_MANCHESTER_MODULATION));
    CLRC_write_reg(CLRC663_REG_DRVMOD, 0x8E); /* Driver mode register */
    CLRC_write_reg(CLRC663_REG_TXAMP, 0x14);  /* Transmiter amplifier register */
    CLRC_write_reg(CLRC663_REG_DRVCON, 0x39); /* Driver configuration register */
    CLRC_write_reg(CLRC663_REG_TXL, 0x06);    /* Transmitter register */

    //go into lpcd to spare current
    lpcd_start();
  }
}

void dump_from_mifare() {
  //request answer from potential tag
  uint16_t atqa = CLRC663_iso14443a_REQA();
  if (atqa != 0) { // Are there any cards that answered?
    uint8_t sak;
    uint8_t uid[10] = {0}; // uids are maximum of 10 bytes long.

    uint8_t uid_len = iso14443aSelect(uid, &sak); //mechanism to actually select card since there could be collisions.

    if (uid_len != 0) { // did we get an UID?
      CLRC663_PRINTF("UID of %hhd bytes (SAK:0x%hhX): ", uid_len, sak);
      CLRC663_print_block(uid, uid_len);
      //TODO move keylist to def header
      //keylist
      uint8_t FFkey1[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      uint8_t FFkey2[6] = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7};

      mifareLoadKey(FFkey1);

      //authenticate first sector (contains uid)
      if (mifareAuth(CLRC663_MF_AUTH_KEY_A, 0, uid)) {
        CLRC663_PRINTF("Sector %hhd values at key number", 0);
        radio_mifare_dump_sector(0);
      }

      for (uint8_t s = 1; s < 16; s++) {

//for a card that was in use here the first sector was identifiable with a different key than the following sectors. maybe you have to play around a bit to find the best fit
        mifareLoadKey(FFkey2);

        //authenticate all the other 15 sectors.
        if (mifareAuth(CLRC663_MF_AUTH_KEY_A, s * 4, uid)) {
          CLRC663_PRINTF("Sector %hhd values at key no %hhd", s);
          radio_mifare_dump_sector(s);

        } else {
          CLRC663_PRINTF("wrong key for authentication or time out");
        }
        //after last sector reset chip and restart lpcd mode
        if (s >= 15) {
          read_LPCDIRQ_FLAG = 0;
          nrfx_gpiote_in_event_enable(NFC_IRQ_PIN, true);
          CLRC_write_reg(CLRC663_REG_STATUS, 0);
          lpcd_start();
        }
      }
    } else {
      CLRC663_PRINTF("Could not determine UID");
    }

  } else {
    CLRC663_PRINTF("No card detected");
  }
}

void NFC_LPCD_IRQ_CHECK() {
  if (read_LPCDIRQ_FLAG) {
    nrfx_gpiote_in_event_disable(NFC_IRQ_PIN);  //as long as polling goes on no IRQs from CLRC
    CLRC663_PRINTF("lpcd irq");
    reset_LPCD();

    while (read_LPCDIRQ_FLAG) {
      dump_from_mifare();
    }
  }
}