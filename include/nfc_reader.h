
#include "clrc663.h"
#include "clrc663_def.h"
#include <nrfx_gpiote.h>
#include <stddef.h>
#include <stdint.h>

/*!
   @brief flag used for efficient processing in the parent uC
*/
volatile uint8_t read_LPCDIRQ_FLAG;


/*!
  @brief nrf gpiote interrupt handler // Set for CLRC_IRQ connected PIN
*/
static void on_clrc_pin_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

/*!
  @brief initialize levelshift, pdown pin, for spi communication;

  /init drvmode registers set for Manchester_Miller Modulation @106 kbit/s
  /start LPCD (this can be commented out if you want to test polling activity in general
*/
void NFCReader_init();

/*!
  @brief all steps from requesting a card until reading its last sector
  
    Request ISO tag with reqa command (0x26, iso standard for tag activation)
    \code
      CLRC663_iso14443a_REQA();
    \endcode
    
    selection process of a tag, we expect the tag to respond with a SAK (Select acknowledge) and a UID (Unique identification) which may be 4 or 7 bytes long
    this function checks for collisions and BBC Mismatches --> ensure Data integrity
    \code
      uint8_t uid_len = iso14443a_select(uid, &sak);
    \endcode

    After Selection and ensuring UID has been fully parsed, load key into CLRC cryptounit
    \code
      mifareLoadKey(FFkey+*Number*); // load into the key buffer
    \endcode

    actual authentification process where all information is processed in the background to access mifare sectors
    \code
      mifareAuth(CLRC663_MF_AUTH_KEY_A, 0, uid)
    \endcode

    if success, read data from respective sector
    \code
      radio_mifare_dump_sector(s);
    \endcode

    this function will be called in a loop, either in constant polling mode or after receiving an LPCD answe
    if function is used after LPCD mode, chip will jump to standby mode either after detectinga card or after 5 seconds.
*/
void dump_from_mifare();

/*!
  @brief in runtime check if flag \read_LPCDIRQ_FLAG has been set by interrupt handler
  
  if flag is set, first reset all the LPCD parameters like stopping timer4
  \code
    reset_LPCD();
  \endcode

  then call dumping function in a loop that runs as long as the flag is still set. 
*/
void NFC_LPCD_IRQ_CHECK();