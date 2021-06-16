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
*/

#include <stdint.h>
#include <stddef.h>
#include "clrc663_def.h"
#include <nrfx_gpiote.h>

#define  CLRC663_PRINTF(...) NRF_LOG_INFO(__VA_ARGS__);



//! @}
// ---------------------------------------------------------------------------
// Register interaction functions.
// ---------------------------------------------------------------------------

/*! \defgroup register Register interaction.
    \brief Manipulate the chip's registers.

    These functions use the SPI communication functions to interact with the hardware. All interaction with the chip
    goes through these functions. In case another interface than SPI is to be used, these functions can be adapted.
  @{
*/

/*!
  @brief Reads a register.
  
  \param [in] reg Specifies which register to read.
  \return the value of the register to be read.
 */
uint8_t CLRC_read_reg(uint8_t reg);

/*!
  @brief Write a register.
  
  Sets a single register to the provided value.
  \param [in] reg Specifies which register to write.
  \param [in] value Specifies the value to write to that register.
 */
void CLRC_write_reg(uint8_t reg, uint8_t value);

void reset_LPCD();

uint8_t dump_truth;
/*!
  @brief Write multiple registers.
  
  Sets consecutive registers to the provided values.
  \param [in] reg Specifies at which register writing starts.
  \param [in] values An array of the values to write to the register starting from `reg`.
              The first value (`values[0]`) gets written to `reg`, the second (`values[1]`) to `reg+1`, and so on.
  \param [in] len The number of register to write.
 */
void CLRC_write_multi(uint8_t reg, const uint8_t* values, uint8_t len);


/*!
  @brief sets up and starts LPCD mode.
*/
void lpcd_start();


//selection of tag after receiving atqa
/*!
  @brief select a tag after receiving  atqa+
  @return length of UUID, fill pointers to uid and sak (selection acknowledge)

*/
uint8_t iso14443aSelect(uint8_t *uid, uint8_t *sak);


/*!
  @brief transmit command to fifo which will then be received by internal CLRC unit

  usually commands like REQA, AUTHENTICATION, Load Crypto Key etc. 
*/
void transceive(uint8_t* data, uint16_t len);

//human readable --> debugging
static void print_buf_hex(uint8_t *buf, size_t len);

//middle piece after authentication dump a whole sector 
//function calls mifare_read_block for every block in the sector
//every sector has to be authenticated on its own (which is crucial, since different keys might have to be used)
void radio_mifare_dump_sector(uint8_t sector_num);

/*!
  internal read and write functions for first in first out buffer. 
  like this we can read from FIFO - register in CLRC and also write to fifo to transmit commands to internal unit
  @return length
*/

int16_t readFIFO(uint16_t len, uint8_t *buffer);
int16_t writeFIFO(uint16_t len, uint8_t *buffer);

//adafruit auth slightly altered
/*!
  @brief authentication process of a single sector 
  
  key has already been stored in crypto unit of IC --> use UID and selected key to decrypt data from tag
  @return will be true if authentication process works
*/
uint8_t mifareAuth(uint8_t key_type, uint8_t blocknum,
                                  uint8_t *uid);

/*!
  @brief load key to crypto unit of IC
  key can be altered, factory new chips will have one of the standard keys (in def.h)
*/                                  
void mifareLoadKey(uint8_t *key);


/*!
  @brief timer0 set function
*/
void set_Timer_Register(uint16_t timerval);

//clrc idle mode (set register 0x00 to 0)
void CLRC663_cmd_idle();

/*
 Flush fifo buffer; register 0x02 bit 5
*/
void CLRC_flushFIFO();

/*!
  @brief Get the FIFO length.

  \return  current fifo data length
 */
uint16_t CLRC663_fifo_length();


/*!
  @brief interrupt flags of reg_irq0 to 0x7f (reset) 

 */
void CLRC663_clear_irq0();
/*!
  @brief interrupt flags of reg_irq1 to 0x7f (reset)
 */
void CLRC663_clear_irq1();


/*!
  \return value of irq0 register.
 */
uint8_t CLRC663_irq0();

/*!
  \return value of irq1 register
 */
uint8_t CLRC663_irq1();



// ---------------------------------------------------------------------------
// Timer functions
// ---------------------------------------------------------------------------
/*! \defgroup timer Timer
    \brief Functions for manipulating the timers.

    The CLRC663 has 5 timers, the first four can be treated more or less similarly, the last timer `Timer4` has a
    different control register.

    Timer 0-3 can be treated in a similar way, and as such the functions take an argument that specifies which timer
    to manipulate.

    Timer4 is special, read the datasheet on how to use that timer as it has other clock sources and properties.


  @{
*/

/*!
  @brief Activates a timer.

  This sets the the `CLRC663_REG_TCONTROL` register to enable or disable this timer.

  \note Seems to trigger timer reset?

  \param [in] timer Specifies which timer to use (0, 1, 2 or 3).
  \param [in] active Should be `0` to deactivate the timer, `1` to activate it.
 */
void CLRC663_activate_timer(uint8_t timer, uint8_t active);

// Set the timer control field of the timer.
// value: the value to set the timer's control field to.
/*!
  @brief Sets the timer control register.

  This sets the `T[0-3]Control` register to the provided value. The value speficies the propertief of StopRx, Start
  AutoRestart and Clk for this timer.

  \param [in] timer Specifies which timer to use (0, 1, 2 or 3).
  \param [in] value This can be a combination of the defines associated with the Timer controls.
  \see CLRC663_TCONTROL_STOPRX
  \see CLRC663_TCONTROL_START_NOT, CLRC663_TCONTROL_START_TX_END, CLRC663_TCONTROL_START_LFO_WO,
       CLRC663_TCONTROL_START_LFO_WITH
  \see CLRC663_TCONTROL_CLK_13MHZ, CLRC663_TCONTROL_CLK_211KHZ, CLRC663_TCONTROL_CLK_UF_TA1, CLRC663_TCONTROL_CLK_UF_TA2
 */
void CLRC663_timer_set_control(uint8_t timer, uint8_t value);

/*!
  @brief Sets the reload value of the timer.

  This counter starts counting down from this reload value, an underflow occurs when the timer reaches zero.

  \param [in] timer Specifies which timer to use (0, 1, 2 or 3).
  \param [in] value The value from which to start the counter. 
 */
void CLRC663_timer_set_reload(uint8_t timer, uint16_t value);

/*!
  @brief Sets the current value of this timer..

  Sets the current value of this counter, it counts down from this given value.

  \param [in] timer Specifies which timer to use (0, 1, 2 or 3).
  \param [in] value The value to set the counter to. 
 */
void CLRC663_timer_set_value(uint8_t timer, uint16_t value);


/*!
  @brief Retrieve the current value of a timer.

  Reads the current value of the given timer and returns the result.

  \param [in] timer Specifies which timer to use (0, 1, 2 or 3).
  \return The current value of this timer.
 */
uint16_t CLRC663_timer_get_value(uint8_t timer);
//!  @}

// ---------------------------------------------------------------------------
// From documentation
// ---------------------------------------------------------------------------

// From Application Note 11145:
//      CLRC663, MFRC631, MFRC 630, SLRC610 Low Power Card Detection
//      http://www.nxp.com/documents/application_note/AN11145.pdf

/*! 
    @brief Start IQ Measurement.
    From Application Note 11145, section 3.2.1, it configures the registers to perform an IQ measurement.
    TODO return values for further calculation
*/
void CLRC663_AN11145_start_IQ_measurement();

// ---------------------------------------------------------------------------
// ISO 14443A functions
// ---------------------------------------------------------------------------


/*! @brief Sends an Request Command 0x26

    @return Answer to request A byte (ATQA) --> 2 bytes
*/
uint16_t CLRC663_iso14443a_REQA();

/*!
  @brief Wake up request to tags in sleepmode
*/
uint16_t CLRC663_iso14443a_WUPA();

/*! 
  @brief send reqa to receive atqa
  take instruction, transceive , set timer to check for answers 
  wait for answer, in polling mode poll reqas 
  in LPCD mode this will be called repeatedly after LPCD irq flag has been set.
*/
uint16_t CLRC663_iso14443a_WUPA_REQA(uint8_t instruction);
