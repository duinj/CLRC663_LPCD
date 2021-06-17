/*
  The MIT License (MIT)

  Copyright (c) 2021 Julius Duin (duinj)

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  *******************************************************************************

  CLRC663 ready to use software

  nrf52 uC controlling of the clrc frontend 

*/

#include "clrc663.h"
#include "spi.h" //this one is device specific. define spi functions in there also specify spi pins.
#include "nrf_delay.h"
#include "nrf_log.h"
#include "string.h"
//in the whole file the debug statements can be substituted by any logging commands


// ---------------------------------------------------------------------------
// Register interaction functions.
// nrf52840 --> I implemented an individual way to pull the CS pin low and high for every transfer
// also the SPI_SEND function is just based on the hal function nrf_drv_spi_transfer function. basically you tell the clrc chip to select 
// a particular address and then send its command.
// ---------------------------------------------------------------------------
uint8_t CLRC_read_reg(uint8_t reg) {
  uint8_t instruction_tx[2] = {(reg << 1) | 0x01, 0}; //register plus instruction to read
  uint8_t instruction_rx[2] = {0};

  CS_Select(CLRC_CS_PIN);
  ret_code_t err_code;

  err_code = SPI_SEND(&instruction_tx[0], &instruction_rx[0], sizeof(instruction_tx), sizeof(instruction_rx), CLRC_CS_PIN, 0, NRF_SPI_MODE_0, NRF_SPI_FREQ_2M);
  APP_ERROR_CHECK(err_code);

  CS_Unselect();

  return instruction_rx[1]; // register value is 2nd byte
}

void CLRC_write_reg(uint8_t reg, uint8_t value) {
  uint8_t instruction_tx[2] = {(reg << 1) | 0x00, value}; //command at [0]
  uint8_t discard[2] = {0};     //if we use a generic SPI function we will have return values that aren't needed

  CS_Select(CLRC_CS_PIN);
  ret_code_t err_code;

  err_code = SPI_SEND(&instruction_tx[0], &discard[0], sizeof(instruction_tx), sizeof(discard), CLRC_CS_PIN, 0, NRF_SPI_MODE_0, NRF_SPI_FREQ_2M);
  APP_ERROR_CHECK(err_code);

  CS_Unselect();
}

//in CLRC663 writing multiple registers means starting at a given reg like 0x28 (drvMode) and incrementing through the registers above, writing a chain of commands
void CLRC_write_multi(uint8_t reg, const uint8_t *values, uint8_t len) {
  uint8_t instruction_tx[len + 1];
  uint8_t discard[len + 1];
  instruction_tx[0] = (reg << 1) | 0x00;
  uint8_t i;
  for (i = 0; i < len; i++) {
    instruction_tx[i + 1] = values[i];
  }
  CS_Select(CLRC_CS_PIN);
  ret_code_t err_code;

  err_code = SPI_SEND(&instruction_tx[0], &discard[0], sizeof(instruction_tx), sizeof(discard), CLRC_CS_PIN, 0, NRF_SPI_MODE_0, NRF_SPI_FREQ_2M);
  APP_ERROR_CHECK(err_code);

  CS_Unselect();
}

//make data from clrc human readable
static void print_buf_hex(uint8_t *buf, size_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (buf[i] != 0x00) {
      CLRC663_PRINTF("%02x", buf[i]);
    }
  }
}

/**************************************************************************/
//reading and writing fifo is crucial, you do so by reading and writing particular registers and not the fifo itself.
int16_t readFIFO(uint16_t len, uint8_t *buffer) {
  int16_t counter = 0;
  
  if (len > 512) { //overflow == error
    return -1;
  }
  
  for (uint16_t i = 0; i < len; i++) {
    buffer[i] = CLRC_read_reg(CLRC663_REG_FIFODATA);  //fifodata reg contains last bytes from actual fifo 
    counter++;
  }

  return counter;
}

//we always write to FIFO when transceiving commands to internal unit of CLRC
int16_t writeFIFO(uint16_t len, uint8_t *buffer) {
  int counter = 0;

  if (len > 512) {
    return -1;
  }

  for (uint16_t i = 0; i < len; i++) {
    CLRC_write_reg(CLRC663_REG_FIFODATA, buffer[i]); //
    counter++;
  }

  return counter;
}

//only in use if LPCD is used in low power devices
void reset_LPCD(){
  CLRC_write_reg(0x00, 0x00);
  CLRC_write_reg(0x08, 0x00);
  CLRC_write_reg(0x09, 0x00);
  CLRC_write_reg(0x02, 0xb0);
  CLRC_write_reg(0x39, 0x00);
  CLRC_write_reg(0x38, 0x12);
  CLRC_write_reg(0x23, 0x5f); //stop timer
}

// ---------------------------------------------------------------------------
// Command functions.
// ---------------------------------------------------------------------------

//the transceive command is used to tell the clrc chip that whats in the fifo has to go toward the central unit of the chip. f.e. sneding commands
//that initiate reading of an nfc tag
void transceive(uint8_t *data, uint16_t len) {
  CLRC663_cmd_idle();
  CLRC_flushFIFO();
  writeFIFO(len, data);
  CLRC_write_reg(CLRC663_REG_COMMAND, CLRC663_CMD_TRANSCEIVE);
}

void CLRC663_cmd_idle() {
  CLRC_write_reg(CLRC663_REG_COMMAND, CLRC663_CMD_IDLE);
}

// ---------------------------------------------------------------------------
//  Utility functions.
//  these are just for abstraction
// ---------------------------------------------------------------------------
void cancel_and_flush(){
  CLRC663_cmd_idle();
  CLRC_flushFIFO();
}

void CLRC_flushFIFO() {
  CLRC_write_reg(CLRC663_REG_FIFOCONTROL, 1 << 4); //0xB0
}

uint16_t CLRC663_fifo_length() {
  return CLRC_read_reg(CLRC663_REG_FIFOLENGTH); 
}

void CLRC663_clear_irq0() {
  CLRC_write_reg(CLRC663_REG_IRQ0, (uint8_t) ~(1 << 7)); //0b01111111
}
void CLRC663_clear_irq1() {
  CLRC_write_reg(CLRC663_REG_IRQ1, (uint8_t) ~(1 << 7)); //0b01111111
}
uint8_t CLRC663_irq0() {
  return CLRC_read_reg(CLRC663_REG_IRQ0);
}
uint8_t CLRC663_irq1() {
  return CLRC_read_reg(CLRC663_REG_IRQ1);
}


// ---------------------------------------------------------------------------
// Timer functions
// ---------------------------------------------------------------------------
void CLRC663_activate_timer(uint8_t timer, uint8_t active) {
  CLRC_write_reg(CLRC663_REG_TCONTROL, ((active << timer) << 4) | (1 << timer));
}

void CLRC663_timer_set_control(uint8_t timer, uint8_t value) {
  CLRC_write_reg(CLRC663_REG_T0CONTROL + (5 * timer), value);
}
void CLRC663_timer_set_reload(uint8_t timer, uint16_t value) {
  CLRC_write_reg(CLRC663_REG_T0RELOADHI + (5 * timer), value >> 8);
  CLRC_write_reg(CLRC663_REG_T0RELOADLO + (5 * timer), value & 0xFF);
}
void CLRC663_timer_set_value(uint8_t timer, uint16_t value) {
  CLRC_write_reg(CLRC663_REG_T0COUNTERVALHI + (5 * timer), value >> 8);
  CLRC_write_reg(CLRC663_REG_T0COUNTERVALLO + (5 * timer), value & 0xFF);
}
uint16_t CLRC663_timer_get_value(uint8_t timer) {
  uint16_t res = CLRC_read_reg(CLRC663_REG_T0COUNTERVALHI + (5 * timer)) << 8;
  res += CLRC_read_reg(CLRC663_REG_T0COUNTERVALLO + (5 * timer));
  return res;
}

// ---------------------------------------------------------------------------
//  measure I and Q values
//  values can be measured with loaded OR unloaded antenna, this is the easiest way to test if the lpcd setup works in principle -- load or unload antenna and compare i/q values.
//  TODO: this should happend automatically from time to time since antenna circumstances might alter
// ---------------------------------------------------------------------------
void CLRC663_AN11145_start_IQ_measurement() {
  
  CLRC_write_reg(CLRC663_REG_COMMAND, CLRC663_CMD_SOFTRESET);       //reset clrc
  nrf_delay_ms(50);
  CLRC_write_reg(CLRC663_REG_COMMAND, CLRC663_CMD_IDLE);            //idle mode as always before any config
  // disable IRQ0, IRQ1 interrupt sources
  CLRC663_clear_irq0();
  CLRC663_clear_irq1():
  CLRC_write_reg(CLRC663_REG_IRQ0EN, 0x00);
  CLRC_write_reg(CLRC663_REG_IRQ1EN, 0x00);
  CLRC_write_reg(CLRC663_REG_HOSTCTRL, CLRC_FIFO_FLUSH); // Flush FIFO

  //  actual LPCD_config
  //  process from an 11145
  CLRC_write_reg(CLRC663_REG_LPCD_QMIN, QMIN_CALIB_VAL); // Set Qmin register
  CLRC_write_reg(CLRC663_REG_LPCD_QMAX, QMAX_CALIB_VAL); // Set Qmax register
  CLRC_write_reg(CLRC663_REG_LPCD_IMIN, IMIN_CALIB_VAL); // Set Imin register
  CLRC_write_reg(CLRC663_REG_DRVMOD, 0x89);              // set DrvMode register
  // Execute trimming procedure
  CLRC_write_reg(CLRC663_REG_T3RELOADHI, 0x00);    // Write default. T3 reload value Hi
  CLRC_write_reg(CLRC663_REG_T3RELOADLO, 0x10);    // Write default. T3 reload value Lo
  CLRC_write_reg(CLRC663_REG_T4RELOADHI, 0x00);    // Write min. T4 reload value Hi
  CLRC_write_reg(CLRC663_REG_T4RELOADLO, 0x05);    // Write min. T4 reload value Lo
  CLRC_write_reg(CLRC663_REG_T4CONTROL, 0xF8);     // Config T4 for AutoLPCD&AutoRestart.Set AutoTrimm bit.Start T4.
  CLRC_write_reg(CLRC663_REG_LPCD_Q_RESULT, 0x00); // Clear LPCD result; dont put bit 6 since in this case we dont wait for another actual lpcd signal but just the idle measurement
  CLRC_write_reg(CLRC663_REG_RCV, 0x52);           // Set Rx_ADCmode bit
  CLRC_write_reg(CLRC663_REG_RXANA, CLRC_RECEIVER_GAIN_MAX);         // Raise receiver gain to maximum
  CLRC_write_reg(CLRC663_REG_COMMAND, 0x01);       // bit 0 = lpcd start
  nrf_delay_ms(1600);                              // have to delay before going into idle again

  cancel_and_flush();
  CLRC_write_reg(CLRC663_REG_RCV, 0x12);        
//TODO: dynamically change values in use with formula in lpcd_start function (from an11145)
  uint8_t i_result = CLRC_read_reg(CLRC663_REG_LPCD_I_RESULT);
  uint8_t q_result = CLRC_read_reg(CLRC663_REG_LPCD_Q_RESULT);
 // CLRC663_PRINTF("%02x", i_result);
  // CLRC663_PRINTF("%02x", q_result);
  
}
#define CALIBRATION 1
//LPCD setup and go to standby. for nrf52840, the pushpull option is used and an irq has to be send as soon as a tag is detected. enable irq on nrf side if needed/wanted
void lpcd_start() {
  
  //calibration is supposed to happen once in a while but it works as long as the CALI value is set to 1 and lpcd restarts after every measurement
  #if CALIBRATION
    CLRC663_AN11145_start_IQ_measurement();
  #endif
  //calculation from nfc reader lib
  uint8_t q1val = 0x21 | ((0x1e & 0x30) << 2);
  uint8_t q2val = 0x23 | ((0x1e & 0x0c) << 4);
  uint8_t q3val = 0x1c | ((0x1e & 0x03) << 6);
  //write calibration values to respective registers
  CLRC_write_reg(CLRC663_REG_LPCD_QMIN, 0x61);    //values in qmin qmax imin come from calibration process, those are basically the borders withina lpcd burst doesnt detect anything
  CLRC_write_reg(CLRC663_REG_LPCD_QMAX, 0xe3);
  CLRC_write_reg(CLRC663_REG_LPCD_IMIN, 0x9c);
  //set timers (3 and 4) -- start Timer4 with AUTORestart and AutoWakeup; no start of timer3 necessary
  CLRC_write_reg(0x1f, 0x02);
  CLRC_write_reg(0x20, 0xf2);
  CLRC_write_reg(0x24, 0x00);
  CLRC_write_reg(0x25, 0x33);
  CLRC_write_reg(CLRC663_REG_T4CONTROL, 0xdf);        //activate t4 timer, autorestart, 0b11 as frequency (datasheet)
  CLRC_write_reg(CLRC663_REG_LPCD_Q_RESULT, 0x40);    //clear q_result with 0x40, bit 6 tells the chip to wait for the next lpcd signal 
  uint8_t mixadc = CLRC_read_reg(0x38);
  mixadc |= 0x40;                                     //from nfc_reader_lib
  CLRC_write_reg(0x38, mixadc);
  uint8_t rxanabackup = CLRC_read_reg(0x39);
  CLRC_write_reg(0x39, 0x03);
  while (!(CLRC_read_reg(0x23) == 0x9f))              //wait for t4 to start (bit 6 will clear)
    ;
  CLRC_write_reg(0x39, rxanabackup);
  cancel_and_flush();
  CLRC663_clear_irq0();
  CLRC663_clear_irq1();
  CLRC_write_reg(CLRC663_REG_IRQ0EN, CLRC663_IRQ0EN_IDLE_IRQEN);                     //bit 4 = idleirq
  //Enable puhspull option of IRQ Pin, enable Pin itself, let LPCD propagate to global IRQ
  //might have to play with push pull option here. measure with oscillator if necessary
  CLRC_write_reg(CLRC663_REG_IRQ1EN, CLRC663_IRQ1EN_IRQ_PUSHPULL | CLRC663_IRQ1EN_LPCD_IRQEN | CLRC663_IRQ1EN_IRQ_PINEN);   
  //start mode and view short bursts @ antenna per oscillator
  CLRC_write_reg(CLRC663_REG_COMMAND, 0x81);       //bit 7 = standby, bit 0 = lpcd 
}

// ---------------------------------------------------------------------------
// ISO 14443A -- these routines are standardized
// send REQA --> receive ATQA --> select Tag --> Selection over --> MIFARE processes
// ---------------------------------------------------------------------------

//request to respond --> normal activation command 0x26
uint16_t CLRC663_iso14443a_REQA() {
  return CLRC663_iso14443a_WUPA_REQA(CLRC663_ISO14443_CMD_REQA);
}

//request to wake up --> activation command 0x52 when hlta has been send before
uint16_t CLRC663_iso14443a_WUPA() {
  return CLRC663_iso14443a_WUPA_REQA(CLRC663_ISO14443_CMD_WUPA);
}

/*
function to receive answer from any chip near the reader antenna
*/
uint16_t CLRC663_iso14443a_WUPA_REQA(uint8_t instruction) {

  cancel_and_flush();

  // Set register such that we sent 7 bits, set DataEn such that we can send data.
  CLRC_write_reg(CLRC663_REG_TXDATANUM, 7 | CLRC663_TXDATANUM_DATAEN);

  // disable the CRC registers.
  CLRC_write_reg(CLRC663_REG_TXCRCPRESET, CLRC663_RECOM_14443A_CRC | CLRC663_CRC_OFF);
  CLRC_write_reg(CLRC663_REG_RXCRCCON, CLRC663_RECOM_14443A_CRC | CLRC663_CRC_OFF);

  //clearing RX
  CLRC_write_reg(CLRC663_REG_RXBITCTRL, 0);

  // ready the request.
  uint8_t send_req[] = {instruction};

  //clear IRQ flags
  CLRC663_clear_irq0();
  CLRC663_clear_irq1();
  // enable the global IRQ for Rx done and Errors.
  CLRC_write_reg(CLRC663_REG_IRQ0EN, CLRC663_IRQ0EN_RX_IRQEN | CLRC663_IRQ0EN_ERR_IRQEN);
  CLRC_write_reg(CLRC663_REG_IRQ1EN, CLRC663_IRQ1EN_TIMER0_IRQEN); // only trigger on timer for irq1
  uint8_t irq0_value = 0;
  uint8_t irq1_value = 0;

  set_Timer_Register(1100);

  transceive(send_req, 1); //send 0x26 iso reqa

  while (!(irq1_value & CLRC663_IRQ1_TIMER0_IRQ)) { // block until we are done
    irq1_value = CLRC663_irq1();                    //read irq1 reg (has global fired?)
    if (irq1_value & CLRC663_IRQ1_GLOBAL_IRQ) {     // either ERR_IRQ or RX_IRQ
      break;                                        // stop polling irq1 and quit the timeout loop.
    }
  }

  CLRC663_cmd_idle();
  
  uint16_t res;
  // if no Rx IRQ, or if there's an error somehow, return 0
  uint8_t irq0 = CLRC663_irq0();
  if ((!(irq0 & CLRC663_IRQ0_RX_IRQ)) || (irq0 & CLRC663_IRQ0_ERR_IRQ)) {   //this will likely happen when card has been moved too fast or if there is simply no card after sending the command
    CLRC663_PRINTF("No RX, irq1: %hhx irq0: %hhx\n", irq1_value, irq0);
    return 0;
  }
  uint8_t rx_len = CLRC663_fifo_length();             //this will tell if theres an actual atqa in fifo not just a random byte


/*
ISO for atqa
+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----
|  16 |  15 |  14 |  13 |  12 |  11 |  10 |   9 |   8 |   7 |   6 |   5 |   4 |   3 |   2 |   1 |
|          RFU          |     PROPR. CODING     |  UID SIZE | RFU |   BIT FRAME ANTICOLLISION   |
+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----
|                      0x00                     |                      0x04                     |
|  0  |  0  |  0  |  0  |  0  |  0  |  0  |  0  |  0  |  0  |  0  |  0  |  0  |  1  |  0  |  0  |
+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----
*/
  if (rx_len == 2) {  //ATQA is always 2 bytes (04 00) LSB
    uint8_t buff[2] = {};

    int16_t readlen = readFIFO(2, buff); /* Read data back and display it*/
    CLRC663_PRINTF("ATQA answer ");
    print_buf_hex(buff, readlen);
    
    return 1;
  }
  return 0;
}

/*
  here the actual card is selected
*/
uint8_t iso14443aSelect(uint8_t *uid, uint8_t *sak) {

  CLRC663_PRINTF("Selecting an ISO14443A tag ------ collision check & reading UID/SAK");

  CLRC663_cmd_idle(); /* Cancel any current command */
  CLRC_flushFIFO(); 

  CLRC_write_reg(CLRC663_REG_IRQ0EN, CLRC663_IRQ0_RX_IRQ | CLRC663_IRQ0_ERR_IRQ); //rx and err irq to respond
  CLRC_write_reg(CLRC663_REG_IRQ1EN, CLRC663_IRQ1EN_TIMER0_IRQEN);                //irq fire when timer0 underflows (to prevent timeout)

  set_Timer_Register(1100);

  uint8_t cascadelvl;

  //3 SEL bytes
  for (cascadelvl = 1; cascadelvl <= 3; cascadelvl++) {
    uint8_t cmd;
    uint8_t kbits = 0;                        /* Bits known in UID so far. */
    uint8_t send_req[7] = {0};                /* TX buffer */
    uint8_t *uid_this_level = &(send_req[2]); /* UID pointer */
    uint8_t message_length;

    switch (cascadelvl) {
    case 1:
      cmd = ISO14443_CAS_LEVEL_1;
      break;
    case 2:
      cmd = ISO14443_CAS_LEVEL_2;
      break;
    case 3:
      cmd = ISO14443_CAS_LEVEL_3;
      break;
    }

    //CLRC663_PRINTF("a. Disabling CRC checks.");
    CLRC_write_reg(CLRC663_REG_TXCRCPRESET, 0x18);
    CLRC_write_reg(CLRC663_REG_RXCRCCON, 0x18);

    /* As per ISO14443-3, limit coliision checks to 32 attempts. */
    uint8_t cnum;

    //CLRC663_PRINTF("b. Collision detection (max 32 attempts)."); most likely works on one of the early attempts
    for (cnum = 0; cnum < 32; cnum++) {

      print_buf_hex(uid_this_level, (kbits + 8 - 1) / 8);

      /* Clear the interrupts. */
      CLRC_write_reg(CLRC663_REG_IRQ0, 0b01111111);
      CLRC_write_reg(CLRC663_REG_IRQ1, 0b00111111);

      /* Send the current collision level command */
      send_req[0] = cmd;
      send_req[1] = 0x20 + kbits;

      /* Limit MFRC630_REG_TX_DATA_NUM to the correct number of bits. */
      CLRC_write_reg(CLRC663_REG_TXDATANUM, (kbits % 8) | (1 << 3));

      // ValuesAfterColl: If cleared, every received bit after a collision is
      // replaced by a zero. This function is needed for ISO/IEC14443
      // anticollision (0<<7). We want to shift the bits with RxAlign
      uint8_t rxalign = kbits % 8;
      CLRC_write_reg(CLRC663_REG_RXBITCTRL, (0 << 7) | (rxalign << 4));

      /* Determine the message length */
      if ((kbits % 8) == 0) {
        message_length = ((kbits / 8)) + 2;
      } else {
        message_length = ((kbits / 8) + 1) + 2;
      }

      /* Send the command. */
      transceive(send_req, message_length);
      uint8_t sdfa = CLRC663_fifo_length();
      // writeCommand(MFRC630_CMD_TRANSCEIVE, message_length, send_req);

      /* Wait until the command execution is complete. */
      uint8_t irq1_value = 0;
      while (!(irq1_value & CLRC663_IRQ1_TIMER0_IRQ)) {
        irq1_value = CLRC_read_reg(CLRC663_REG_IRQ1);
        /* Check for a global interrrupt, which can only be ERR or RX. */
        if (irq1_value & CLRC663_IRQ1_GLOBAL_IRQ) {
          break;
        }
      }

      /* Cancel any current command */
      CLRC663_cmd_idle();

      /* Parse results */
      uint8_t irq0_value = CLRC_read_reg(CLRC663_REG_IRQ0);
      uint8_t error = CLRC_read_reg(CLRC663_REG_ERROR);
      uint8_t coll = CLRC_read_reg(CLRC663_REG_RXCOLL);
      uint8_t coll_p = 0;

      /* Check if an error occured */
      if (irq0_value & CLRC663_IRQ0_ERR_IRQ) {
        /* Display the error code in human-readable format. */
        if (error & CLRC663_ERROR_COLLDET) {
          /* Collision error, check if the collision position is valid */
          if (coll & (1 << 7)) {      
            /* Valid, so check the collision position (bottom 7 bits). */
            coll_p = coll & (~(1 << 7));

            CLRC663_PRINTF("Bit collision detected at bit ");
            CLRC663_PRINTF("%hhu ", coll_p);

            uint8_t choice_pos = kbits + coll_p;
            uint8_t selection =
                (uid[((choice_pos + (cascadelvl - 1) * 3) / 8)] >>
                    ((choice_pos) % 8)) &
                1;
            uid_this_level[((choice_pos) / 8)] |= selection
                                                  << ((choice_pos) % 8);
            kbits++;

            CLRC663_PRINTF("'uid_this_level' is now ");
            CLRC663_PRINTF("%hhu ", kbits);
            CLRC663_PRINTF(": ");
            print_buf_hex(uid_this_level, 10);
            CLRC663_PRINTF("");
          } else {
            /* Invalid collision position (bit 7 = 0) */

            CLRC663_PRINTF("Bit collision detected, but no valid position.");
            coll_p = 0x20 - kbits;
          } /* End: if (coll & (1 << 7)) */
        } else {

          CLRC663_PRINTF("Unhandled error.");
          coll_p = 0x20 - kbits;
        } /* End: if (error & MFRC630_ERROR_COLLDET) */
      } else if (irq0_value & CLRC663_IRQ0_RX_IRQ) {
        /* We have data and no collision, all is well in the world! */
        coll_p = 0x20 - kbits;

        CLRC663_PRINTF("Received data, no bit collision!");
      } else {
        /* Probably no card */

        CLRC663_PRINTF("No error and no data = No card");
        return 0;
      } /* End: if (irq0_value & (1 << 1)) */

      /* Read the UID so far */
      uint16_t rxlen = CLRC663_fifo_length();
      uint8_t buf[5]; /* UID = 4 bytes + BCC */
      readFIFO(rxlen < 5 ? rxlen : 5, buf);

      /*
       * Move current buffer contents into the UID placeholder, OR'ing the
       * results so that we don't lose the bit we set if you have a collision.
       */
      uint8_t rbx;
      for (rbx = 0; (rbx < rxlen); rbx++) {
        uid_this_level[(kbits / 8) + rbx] |= buf[rbx];
      }
      kbits += coll_p;

      if ((kbits >= 32)) {

        //CLRC663_PRINTF("Leaving collision loop: uid ");
        //CLRC663_PRINTF("%hhu ",kbits);
        //CLRC663_PRINTF(" bits long");

        print_buf_hex(uid_this_level, kbits / 8);
        CLRC663_PRINTF("");
        break; /* Exit the collision loop */
      }
    } /* End: for (cnum = 0; cnum < 32; cnum++) */

    /* Check if the BCC matches ... */

    //CLRC663_PRINTF("C. Checking BCC for data integrity.");
    uint8_t bcc_val = uid_this_level[4];
    uint8_t bcc_calc = uid_this_level[0] ^ uid_this_level[1] ^
                       uid_this_level[2] ^ uid_this_level[3];
    if (bcc_val != bcc_calc) {                      //block check character
      CLRC663_PRINTF("ERROR: BCC mismatch!\n");
      return 0;
    }

    /* Clear the interrupts. */
    CLRC_write_reg(CLRC663_REG_IRQ0, 0b01111111);
    CLRC_write_reg(CLRC663_REG_IRQ1, 0b00111111);

    send_req[0] = cmd;
    send_req[1] = 0x70;
    send_req[6] = bcc_calc;
    message_length = 7;

    /* Re-enable CRCs. */
    CLRC_write_reg(CLRC663_REG_TXCRCPRESET, 0x18 | 1);
    CLRC_write_reg(CLRC663_REG_RXCRCCON, 0x18 | 1);

    /* Reset the TX and RX registers (disable alignment, transmit full bytes) */
    CLRC_write_reg(CLRC663_REG_TXDATANUM, (kbits % 8) | (1 << 3));
    uint8_t rxalign = 0;
    CLRC_write_reg(CLRC663_REG_RXBITCTRL, (0 << 7) | (rxalign << 4));

    /* Send the command. */

    CLRC663_PRINTF("E. Sending collision command");
    transceive(send_req, message_length);
    //writeCommand(MFRC630_CMD_TRANSCEIVE, message_length, send_req);

    /* Wait until the command execution is complete. */
    uint8_t irq1_value = 0;
    while (!(irq1_value & CLRC663_IRQ1_TIMER0_IRQ)) {
      irq1_value = CLRC_read_reg(CLRC663_REG_IRQ1);
      /* Check for a global interrrupt, which can only be ERR or RX. */
      if (irq1_value & CLRC663_IRQ1_GLOBAL_IRQ) {
        break;
      }
    }
    CLRC663_cmd_idle();

    /* Check the source of exiting the loop. */

    uint8_t irq0_value = CLRC_read_reg(CLRC663_REG_IRQ0);
    /* Check the ERROR IRQ */
    if (irq0_value & CLRC663_IRQ0_ERR_IRQ) {
      /* Check what kind of error. */
      uint8_t error = CLRC_read_reg(CLRC663_REG_ERROR);
      if (error & CLRC663_ERROR_COLLDET) {
        /* Collision detecttion. */
        CLRC663_PRINTF("%hhu", error);
        return 0;
      }
    }

    /* Read SAK Select Acknowledge answer from fifo. should be 0x98 or 0x08*/

    //CLRC663_PRINTF("G. Checking SAK in response payload.");
    uint8_t sak_len = CLRC663_fifo_length();
    if (sak_len != 1) {

      //  CLRC663_PRINTF("ERROR: NO SAK in response!\n");
      return 0;
    }
    uint8_t sak_value;
    readFIFO(sak_len, &sak_value);

    /* Check if there is more data to read. */
    if (sak_value & (1 << 2)) {
      /* UID not yet complete, continue to next cascade. */

      CLRC663_PRINTF("UID not complete ... looping to next cascade level.");
      uint8_t UIDn;
      for (UIDn = 0; UIDn < 3; UIDn++) {
        // uid_this_level[UIDn] = uid_this_level[UIDn + 1];
        uid[(cascadelvl - 1) * 3 + UIDn] = uid_this_level[UIDn + 1];
      }
    } else {

      CLRC663_PRINTF("DONE! UID fully parsed, exiting.");
      /* Done! */
      /* Add current bytes at this level to the UID. */
      uint8_t UIDn;
      for (UIDn = 0; UIDn < 4; UIDn++) {
        uid[(cascadelvl - 1) * 3 + UIDn] = uid_this_level[UIDn];
      }

      /* Finally, return the length of the UID that's now at the uid pointer. */
      return cascadelvl * 3 + 1;
    }

  } /* End: for (cascadelvl = 1; cascadelvl <= 3; cascadelvl++) */

  /* Return 0 for UUID length if nothing was found. */
  return 0;
}

/*
to load data from tag you have to provide a valid key; keys are provided in header or in dump_function
*/
void mifareLoadKey(uint8_t *key) {
  CLRC663_cmd_idle();
  CLRC_flushFIFO();
  writeFIFO(6, key);
  CLRC_write_reg(CLRC663_REG_COMMAND, CLRC663_CMD_LOADKEY); //after writing to fifo, this command sends the key towards the crypto unit of the clrc where its going to be used for further authentification
}

//authentification requires the relevant key and the uid, afterwards data will be in exchange between tag and reader
/*
for most parts, manufacturer key will be used and key A is relevant (there are two types of keys, likely one is for reading action, the other for write access [type b])
*/
uint8_t mifareAuth(uint8_t key_type, uint8_t blocknum,
    uint8_t *uid) {

  cancel_and_flush();

  CLRC_write_reg(CLRC663_REG_IRQ0EN, CLRC663_IRQ0_IDLE_IRQ | CLRC663_IRQ0_ERR_IRQ);
  CLRC_write_reg(CLRC663_REG_IRQ1EN, CLRC663_IRQ1_TIMER0_IRQ); // only trigger on timer for irq1

  set_Timer_Register(2000);

  CLRC663_clear_irq0();
  CLRC663_clear_irq1();

  cancel_and_flush();

  //send key type the block to be decrypted and the entire uid to fifo and afterwards use auth command (this one uses the key from above)
  uint8_t params[6] = {key_type, blocknum, uid[0], uid[1], uid[2], uid[3]}; 
  writeFIFO(6, params);
  CLRC_write_reg(CLRC663_REG_COMMAND, CLRC663_CMD_MFAUTHENT);

  uint8_t irq1_value = 0;
  while (!(irq1_value & CLRC663_IRQ1_TIMER0_IRQ)) { //timer0 fire for timeout prevention
    irq1_value = CLRC_read_reg(CLRC663_REG_IRQ1);
    if (irq1_value & CLRC663_IRQ1_GLOBAL_IRQ) {
      break;
    }
  }

  //there will be an error if wrong key is used, look into example dump to change the existing key, there are a few default keys available online
  uint8_t error = CLRC_read_reg(CLRC663_REG_ERROR); 
  if (error) {
    CLRC663_PRINTF("%hhx", error);
    return 0;
  }
  //timeout might also mean that card moved too fast (very unlikely but happens while debugging a lot)
  if (irq1_value & CLRC663_IRQ1_TIMER0_IRQ) { 
    return 0;
  }

  //this one checks if all of the authentification actually worked within clrc663
  uint8_t status = CLRC_read_reg(CLRC663_REG_STATUS);
  return (status & CLRC663_STATUS_CRYPTO1_ON) ? 1 : 0; 
}

//setting timers with one simple function , only t0, might want to implement a more comprehendible version
void set_Timer_Register(uint16_t timerval) {

  CLRC_write_reg(CLRC663_REG_T0CONTROL, 0b10001);

  CLRC_write_reg(CLRC663_REG_T0RELOADLO, 0xFF);

  CLRC_write_reg(CLRC663_REG_T0COUNTERVALLO, 0xff);
  if (timerval == 3000) {
    CLRC_write_reg(CLRC663_REG_T0RELOADHI, 0xff);
    CLRC_write_reg(CLRC663_REG_T0COUNTERVALHI, 0xff);
  } else {
    CLRC_write_reg(CLRC663_REG_T0RELOADHI, timerval >> 8);
    CLRC_write_reg(CLRC663_REG_T0COUNTERVALHI, timerval >> 8);
  }
}

//human readable debug print of tag contents
void parse_ndef_msg(uint8_t *buf, uint16_t len) {
  unsigned char s[16];
  unsigned char r[16];
  memcpy(s, buf, 16);
  for (uint8_t i = 0; i < 16; i++) {

    if (s[i] == 0xD1) {
      memcpy(r, &buf[i], 16 - i);
      r[16 - i] = '\0';
    }
    if (s[i] == 0x68) {
      uint8_t d = 0;
    }
    if (s[i] == 0xfe) {
      s[i] = '\0';
      break;
    }
  }
  if (r[0] | s[0]) {
    CLRC663_PRINTF("%s", r);
    CLRC663_PRINTF("%s", s);
  }
}

//mifare function after auth --> read data that has been received by fifo
uint16_t mifareReadBlock(uint8_t blocknum, uint8_t *buf) {
  CLRC_flushFIFO();

  CLRC_write_reg(CLRC663_REG_TXCRCPRESET, CLRC663_RECOM_14443A_CRC | CLRC663_CRC_ON);
  CLRC_write_reg(CLRC663_REG_RXCRCCON, CLRC663_RECOM_14443A_CRC | CLRC663_CRC_ON);

  CLRC_write_reg(CLRC663_REG_IRQ0EN, CLRC663_IRQ0_IDLE_IRQ | CLRC663_IRQ0_ERR_IRQ);
  CLRC_write_reg(CLRC663_REG_IRQ1EN, CLRC663_IRQ1_TIMER0_IRQ); // only trigger on timer for irq1

  //set register with 3000 which is code for having a very short timer period 
  set_Timer_Register(3000);

  CLRC_write_reg(CLRC663_REG_IRQ0, 0b01111111);
  CLRC_write_reg(CLRC663_REG_IRQ1, 0b00111111);

  uint8_t req[2] = {CLRC663_MF_CMD_READ, blocknum}; //write to fifo and instantly transmit command.
  transceive(req, 2);

  uint8_t irq1_value = 0;
  while (!(irq1_value & CLRC663_IRQ1_TIMER0_IRQ)) {
    irq1_value = CLRC_read_reg(CLRC663_REG_IRQ1);
    if (irq1_value & CLRC663_IRQ1_GLOBAL_IRQ) {
      break;
    }
  }
  CLRC663_cmd_idle();

  if (irq1_value & CLRC663_IRQ1_TIMER0_IRQ) {
    /* Timed out, no auth :( */
    CLRC663_PRINTF("TIMED OUT!"); //even if authenticated you can still get a timeout from moving too fast (or if secotr is corrupted)
    return 0;
  }

  /* Read the size and contents of the FIFO, and return the results. */
  uint16_t buffer_length = CLRC663_fifo_length();
  uint16_t rx_len = (buffer_length <= 16) ? buffer_length : 16;
  readFIFO(rx_len, buf);

  return rx_len;
}

//mifare cards have 16 sectors a 4 block and within each block weve got 16 bytes that are being read in this function.
void radio_mifare_dump_sector(uint8_t sector_num) {
  uint8_t readbuf[16] = {0};
  /* Try to read four blocks inside the sector. */
  for (uint8_t b = 0; b < 4; b++) {
    uint8_t len = 0;
    len = mifareReadBlock(sector_num * 4 + b, readbuf);
    if (len == 0) {
      /* No data returned! */
      CLRC663_PRINTF("no data on block %02x ", b);
      return;
    } else {
      if (sector_num == 0) {
        print_buf_hex(readbuf, len); //in the first sector print actual hex values while within the other sectors print ascii representation
      } else {
        parse_ndef_msg(readbuf, len);
      }
    }
  }
}

