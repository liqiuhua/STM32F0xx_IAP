
/******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/myliberty
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/
/*! \file
 *
 *  \author 
 *
 *  \brief Platform header file. Defining platform independent functionality.
 *
 */


/*
 *      PROJECT:   
 *      $Revision: $
 *      LANGUAGE:  ISO C99
 */

/*! \file platform.h
 *
 *  \author Gustavo Patricio
 *
 *  \brief Platform specific definition layer  
 *  
 *  This should contain all platform and hardware specifics such as 
 *  GPIO assignment, system resources, locks, IRQs, etc
 *  
 *  Each distinct platform/system/board must provide this definitions 
 *  for all SW layers to use
 *  
 */

#ifndef PLATFORM_H
#define PLATFORM_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

#include "stm32f0xx.h"
#include "st_errno.h"


#include "ST25R3911B.h"
#include "timer.h"
//#include "main.h"
/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/
#define ST25R391X_SS_PIN            SPI1_CS_Pin         /*!< GPIO pin used for ST25R3911 SPI SS                */ 
#define ST25R391X_SS_PORT           SPI1_CS_GPIO_Port   /*!< GPIO port used for ST25R3911 SPI SS port          */ 

#define ST25R391X_INT_PIN           GPIO_Pin_0        /*!< GPIO pin used for ST25R3911 External Interrupt    */
#define ST25R391X_INT_PORT          GPIOA  /*!< GPIO port used for ST25R3911 External Interrupt   */

#ifdef LED_FIELD_Pin
#define PLATFORM_LED_FIELD_PIN      LED_FIELD_Pin       /*!< GPIO pin used as field LED                        */
#endif

#ifdef LED_FIELD_GPIO_Port
#define PLATFORM_LED_FIELD_PORT     LED_FIELD_GPIO_Port /*!< GPIO port used as field LED                       */
#endif


/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/
#define platformIrqST25R3911Disable()               NVIC_DisableIRQ(EXTI0_1_IRQn);              /*!< Disables ST353911 MCU's external interrupt  */
#define platformIrqST25R3911Enable()                NVIC_EnableIRQ(EXTI0_1_IRQn);               /*!< Enables ST353911 MCU's external interrupt   */
#define platformProtectST25R391xIrqStatus()           NVIC_DisableIRQ(EXTI0_1_IRQn);              /*!< Disables ST353911 MCU's external interrupt  */
#define platformUnprotectST25R391xIrqStatus()         NVIC_EnableIRQ(EXTI0_1_IRQn);               /*!< Enables ST353911 MCU's external interrupt   */
#define platformProtectST25R391xComm()                platformProtectST25R391xIrqStatus()
#define platformUnprotectST25R391xComm()              platformUnprotectST25R391xIrqStatus()

#define platformIrqST25R3911SetCallback( cb )          
#define platformIrqST25R3911PinInitialize()                


#define platformLedsInitialize()                                                                    /*!< Initializes the pins used as LEDs to outputs*/

#define platformLedOff( port, pin )                   do{platformGpioClear(port, pin);if(pin==LED_FIELD_Pin)platformGpioClear(port, LED_FIELD_Pin_MB1325B);}while(0)    /*!< Turns the given LED Off                     */ /* Support for old board version MB1325-A */
#define platformLedOn( port, pin )                    do{platformGpioSet(port, pin);if(pin==LED_FIELD_Pin)platformGpioSet(port, LED_FIELD_Pin_MB1325B);}while(0)        /*!< Turns the given LED On                      */ /* Support for old board version MB1325-A */
#define platformLedToogle( port, pin )                do{platformGpioToogle(port, pin);if(pin==LED_FIELD_Pin)platformGpioToogle(port, LED_FIELD_Pin_MB1325B);}while(0)  /*!< Toogle the given LED                        */ /* Support for old board version MB1325-A */

#define platformGpioSet( port, pin )                  GPIO_SetBits(port, pin)    /*!< Turns the given GPIO High                   */
#define platformGpioClear( port, pin )                GPIO_ResetBits(port, pin)  /*!< Turns the given GPIO Low                    */
//#define platformGpioToogle( port, pin )               HAL_GPIO_TogglePin(port, pin)                 /*!< Toogles the given GPIO                      */
#define platformGpioIsHigh( port, pin )               (GPIO_ReadInputDataBit(port, pin)) /*!< Checks if the given LED is High             */
#define platformGpioIsLow( port, pin )                (GPIO_ReadInputDataBit(port, pin))              /*!< Checks if the given LED is Low              */

#define platformTimerCreate( t )                      timerCalculateTimer(t)                        /*!< Create a timer with the given time (ms)     */
#define platformTimerIsExpired( timer )               timerIsExpired(timer)                         /*!< Checks if the given timer is expired        */
#define platformDelay( t )                            Delay10ms( t )                                /*!< Performs a delay for the given time (ms)    */

#define platformGetSysTick()                          GetTick()                                 /*!< Get System Tick ( 1 tick = 1 ms)            */

//#define ST25R3911_COM_SINGLETXRX
#define platformSpiRx( Regaddr, rxBuf, len )          spiRx(Regaddr, rxBuf, len)                    /*!< SPI transceive                              */
#define platformSpiTx( Regaddr, txBuf, len )          spiTx(Regaddr, txBuf, len)

#define platformSpiSelect()                           spiSelect(ST25R391X_SS_PORT, ST25R391X_SS_PIN)
#define platformSpiDeselect()                         spiDeselect(ST25R391X_SS_PORT, ST25R391X_SS_PIN)

#define platformI2CTx( txBuf, len )                                                                 /*!< I2C Transmit                                */
#define platformI2CRx( txBuf, len )                                                                 /*!< I2C Receive                                 */
#define platformI2CStart()                                                                          /*!< I2C Start condition                         */
#define platformI2CStop()                                                                           /*!< I2C Stop condition                          */
#define platformI2CRepeatStart()                                                                    /*!< I2C Repeat Start                            */
#define platformI2CSlaveAddrWR(add)                                                                 /*!< I2C Slave address for Write operation       */
#define platformI2CSlaveAddrRD(add)                                                                 /*!< I2C Slave address for Read operation        */

#define platformLog(...)                                                                            /*!< Log method                                  */


/*
******************************************************************************
* RFAL FEATURES CONFIGURATION
******************************************************************************
*/

#define RFAL_FEATURE_NFCA                      true                   /*!< Enable/Disable RFAL support for NFC-A (ISO14443A)                         */
#define RFAL_FEATURE_NFCB                      true                   /*!< Enable/Disable RFAL support for NFC-B (ISO14443B)                         */
#define RFAL_FEATURE_NFCF                      true                   /*!< Enable/Disable RFAL support for NFC-F (FeliCa)                            */
#define RFAL_FEATURE_NFCV                      true                   /*!< Enable/Disable RFAL support for NFC-V (ISO15693)                          */
#define RFAL_FEATURE_T1T                       true                   /*!< Enable/Disable RFAL support for T1T (Topaz)                               */
#define RFAL_FEATURE_ST25TB                    true                   /*!< Enable/Disable RFAL support for ST25TB                                    */
#define RFAL_FEATURE_DYNAMIC_ANALOG_CONFIG     true                   /*!< Enable/Disable Analog Configs to be dynamically updated (RAM)             */
#define RFAL_FEATURE_DYNAMIC_POWER             false                  /*!< Enable/Disable RFAL dynamic power support                                 */
#define RFAL_FEATURE_ISO_DEP                   true                   /*!< Enable/Disable RFAL support for ISO-DEP (ISO14443-4)                      */
#define RFAL_FEATURE_ISO_DEP_POLL              true                   /*!< Enable/Disable RFAL support for Poller mode (PCD) ISO-DEP (ISO14443-4)    */
#define RFAL_FEATURE_ISO_DEP_LISTEN            false                  /*!< Enable/Disable RFAL support for Listen mode (PICC) ISO-DEP (ISO14443-4)   */
#define RFAL_FEATURE_NFC_DEP                   true                   /*!< Enable/Disable RFAL support for NFC-DEP (NFCIP1/P2P)                      */


#define RFAL_FEATURE_ISO_DEP_IBLOCK_MAX_LEN    256                    /*!< ISO-DEP I-Block max length. Please use values as defined by rfalIsoDepFSx */
#define RFAL_FEATURE_ISO_DEP_APDU_MAX_LEN      1024                   /*!< ISO-DEP APDU max length. Please use multiples of I-Block max length       */

#endif /* PLATFORM_H */


