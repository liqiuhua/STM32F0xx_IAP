
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


/*
 *      PROJECT:   ST25R3911 firmware
 *      $Revision: $
 *      LANGUAGE:  ISO C99
 */

/*! \file
 *
 *  \author Ulrich Herrmann
 *
 *  \brief ST25R3911 Interrupt header file
 *
 *
 * @addtogroup RFAL
 * @{
 *
 * @addtogroup RFAL-HAL
 * @brief RFAL Hardware Abstraction Layer
 * @{
 *
 * @addtogroup ST25R3911
 * @brief RFAL ST25R3911 Driver
 * @{
 * 
 * @addtogroup ST25R3911_Interrupt
 * @brief RFAL ST25R3911 Interrupt
 * @{
 *
 */

#ifndef ST25R3911_INTERRUPT_H
#define ST25R3911_INTERRUPT_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"

/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/

/* Main interrupt register. */
#define ST25R3911_IRQ_MASK_ALL             (uint32_t)(0xFFFFFF) /*!< All ST25R3911 interrupt sources. */
#define ST25R3911_IRQ_MASK_NONE            (uint32_t)(0)        /*!< No ST25R3911 interrupt source. */
#define ST25R3911_IRQ_MASK_OSC             (uint32_t)(0x80)     /*!< ST25R3911 oscillator stable interrupt. */
#define ST25R3911_IRQ_MASK_FWL             (uint32_t)(0x40)     /*!< ST25R3911 FIFO water level interrupt. */
#define ST25R3911_IRQ_MASK_RXS             (uint32_t)(0x20)     /*!< ST25R3911 start of receive interrupt. */
#define ST25R3911_IRQ_MASK_RXE             (uint32_t)(0x10)     /*!< ST25R3911 end of receive interrupt. */
#define ST25R3911_IRQ_MASK_TXE             (uint32_t)(0x08)     /*!< ST25R3911 end of transmission interrupt. */
#define ST25R3911_IRQ_MASK_COL             (uint32_t)(0x04)     /*!< ST25R3911 bit collision interrupt. */

/* Timer and NFC interrupt register. */
#define ST25R3911_IRQ_MASK_DCT             (uint32_t)(0x8000)   /*!< ST25R3911 termination of direct command interrupt. */
#define ST25R3911_IRQ_MASK_NRE             (uint32_t)(0x4000)   /*!< ST25R3911 no-response timer expired interrupt. */
#define ST25R3911_IRQ_MASK_GPE             (uint32_t)(0x2000)   /*!< ST25R3911 general purpose timer expired interrupt. */
#define ST25R3911_IRQ_MASK_EON             (uint32_t)(0x1000)   /*!< ST25R3911 external field on interrupt. */
#define ST25R3911_IRQ_MASK_EOF             (uint32_t)(0x0800)   /*!< ST25R3911 external field off interrupt. */
#define ST25R3911_IRQ_MASK_CAC             (uint32_t)(0x0400)   /*!< ST25R3911 collision during RF collision avoidance interrupt. */
#define ST25R3911_IRQ_MASK_CAT             (uint32_t)(0x0200)   /*!< ST25R3911 minimum guard time expired interrupt. */
#define ST25R3911_IRQ_MASK_NFCT            (uint32_t)(0x0100)   /*!< ST25R3911 initiator bit rate recognized interrupt. */

/* Error and wake-up interrupt register. */
#define ST25R3911_IRQ_MASK_CRC             (uint32_t)(0x800000) /*!< ST25R3911 CRC error interrupt. */
#define ST25R3911_IRQ_MASK_PAR             (uint32_t)(0x400000) /*!< ST25R3911 parity error interrupt. */
#define ST25R3911_IRQ_MASK_ERR2            (uint32_t)(0x200000) /*!< ST25R3911 soft framing error interrupt. */
#define ST25R3911_IRQ_MASK_ERR1            (uint32_t)(0x100000) /*!< ST25R3911 hard framing error interrupt. */
#define ST25R3911_IRQ_MASK_WT              (uint32_t)(0x080000) /*!< ST25R3911 wake-up interrupt. */
#define ST25R3911_IRQ_MASK_WAM             (uint32_t)(0x040000) /*!< ST25R3911 wake-up due to amplitude interrupt. */
#define ST25R3911_IRQ_MASK_WPH             (uint32_t)(0x020000) /*!< ST25R3911 wake-up due to phase interrupt. */
#define ST25R3911_IRQ_MASK_WCAP            (uint32_t)(0x010000) /*!< ST25R3911 wake-up due to capacitance measurement. */


#define ST25R3911_IRQ_MASK_TIM             (0x02)               /*!< additional interrupts in ST25R3911_REG_IRQ_TIMER_NFC */
#define ST25R3911_IRQ_MASK_ERR             (0x01)               /*!< additional interrupts in ST25R3911_REG_IRQ_ERROR_WUP */


/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/


/*! 
 *****************************************************************************
 *  \brief  Wait until an ST25R3911 interrupt occurs
 *
 *  This function is used to access the ST25R3911 interrupt flags. Use this
 *  to wait for max. \a tmo milliseconds for the \b first interrupt indicated
 *  with mask \a mask to occur.
 *
 *  \param[in] mask : mask indicating the interrupts to wait for.
 *  \param[in] tmo : time in milliseconds until timeout occurs. If set to 0
 *                   the functions waits forever.
 *
 *  \return : 0 if timeout occured otherwise a mask indicating the cleared
 *              interrupts.
 *
 *****************************************************************************
 */
extern uint32_t st25r3911WaitForInterruptsTimed(uint32_t mask, uint16_t tmo);

/*! 
 *****************************************************************************
 *  \brief  Get status for the given interrupt
 *
 *  This function is used to check whether the interrupt given by \a mask
 *  has occured. If yes the interrupt gets cleared. This function returns
 *  only status bits which are inside \a mask.
 *
 *  \param[in] mask : mask indicating the interrupt to check for.
 *
 *  \return the mask of the interrupts occurred
 *
 *****************************************************************************
 */
extern uint32_t st25r3911GetInterrupt(uint32_t mask);


/*! 
 *****************************************************************************
 *  \brief  Checks if there is an interrupt
 *
 *  This function is used to check whether the interrupt given by \a mask
 *  has occured. 
 *
 *****************************************************************************
 */
extern void st25r3911CheckForReceivedInterrupts( void );


/*! 
 *****************************************************************************
 *  \brief  Init the 3911 interrupt
 *
 *  This function is used to check whether the interrupt given by \a mask
 *  has occured. 
 *
 *****************************************************************************
 */
extern void st25r3911InitInterrupts( void );


/*! 
 *****************************************************************************
 *  \brief  Modifies the Interrupt
 *
 *  This function modifies the interrupt
 *  
 *  \param[in] clr_mask : bit mask to be cleared on the interrupt mask 
 *  \param[in] set_mask : bit mask to be set on the interrupt mask 
 *****************************************************************************
 */
extern void st25r3911ModifyInterrupts(uint32_t clr_mask, uint32_t set_mask);


/*! 
 *****************************************************************************
 *  \brief Checks received interrupts
 *
 *  Checks received interrupts and saves the result into global params
 *****************************************************************************
 */
extern void st25r3911CheckForReceivedInterrupts( void );


/*! 
 *****************************************************************************
 *  \brief  ISR Service routine
 *
 *  This function modiefies the interupt
 *****************************************************************************
 */
extern void  st25r3911Isr( void );

/*! 
 *****************************************************************************
 *  \brief  Enable a given ST25R3911 Interrupt source
 *
 *  This function enables all interrupts given by \a mask, 
 *  ST25R3911_IRQ_MASK_ALL enables all interrupts.
 *
 *  \param[in] mask: mask indicating the interrupts to be enabled
 *
 *****************************************************************************
 */
extern void st25r3911EnableInterrupts(uint32_t mask);

/*! 
 *****************************************************************************
 *  \brief  Disable one or more a given ST25R3911 Interrupt sources
 *
 *  This function disables all interrupts given by \a mask. 0xff disables all.
 *
 *  \param[in] mask: mask indicating the interrupts to be disabled.
 *
 *****************************************************************************
 */
extern void st25r3911DisableInterrupts(uint32_t mask);

/*! 
 *****************************************************************************
 *  \brief  Clear all st25r3911 irq flags
 *
 *****************************************************************************
 */
extern void st25r3911ClearInterrupts(void);

/*! 
 *****************************************************************************
 *  \brief  Sets IRQ callback for the ST25R3911 interrupt
 *
 *****************************************************************************
 */
extern void st25r3911IRQCallbackSet(void (*cb)());

/*! 
 *****************************************************************************
 *  \brief  Sets IRQ callback for the ST25R3911 interrupt
 *
 *****************************************************************************
 */
extern void st25r3911IRQCallbackRestore(void);

#endif /* ST25R3911_ISR_H */

/**
  * @}
  *
  * @}
  *
  * @}
  * 
  * @}
  */
