
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
 *  \brief Implementation of ST25R3911 communication.
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "st25r3911_com.h"
#include "st25r3911.h"
#include "utils.h"


/*
******************************************************************************
* LOCAL DEFINES
******************************************************************************
*/

#define ST25R3911_WRITE_MODE  (0)                           /*!< ST25R3911 SPI Operation Mode: Write                            */
#define ST25R3911_READ_MODE   (1 << 6)                      /*!< ST25R3911 SPI Operation Mode: Read                             */
#define ST25R3911_FIFO_LOAD   (2 << 6)                      /*!< ST25R3911 SPI Operation Mode: FIFO Load                        */
#define ST25R3911_FIFO_READ   (0xbf)                        /*!< ST25R3911 SPI Operation Mode: FIFO Read                        */
#define ST25R3911_CMD_MODE    (3 << 6)                      /*!< ST25R3911 SPI Operation Mode: Direct Command                   */

#define ST25R3911_CMD_LEN     (1)                           /*!< ST25R3911 CMD length                                           */
#define ST25R3911_BUF_LEN     (ST25R3911_CMD_LEN+ST25R3911_FIFO_DEPTH) /*!< ST25R3911 communication buffer: CMD + FIFO length   */

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/

#ifdef ST25R391X_COM_SINGLETXRX
static uint8_t comBuf[ST25R3911_BUF_LEN];    /*!< ST25R3911 communication buffer            */
#endif /* ST25R391X_COM_SINGLETXRX */

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/

static inline void st25r3911CheckFieldSetLED(uint8_t val)
{
    if (ST25R3911_REG_OP_CONTROL_tx_en & val)
    {
#ifdef PLATFORM_LED_FIELD_PIN
        platformLedOn( PLATFORM_LED_FIELD_PORT, PLATFORM_LED_FIELD_PIN );
    }
    else
    {
        platformLedOff( PLATFORM_LED_FIELD_PORT, PLATFORM_LED_FIELD_PIN );
#endif /* PLATFORM_LED_FIELD_PIN */
    }
}


/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
void st25r3911ReadRegister(uint8_t reg, uint8_t* val)
{ 
#ifdef ST25R391X_COM_SINGLETXRX
    uint8_t* buf = comBuf;
#else  /* ST25R391X_COM_SINGLETXRX */
    uint8_t  buf[2];
#endif  /* ST25R391X_COM_SINGLETXRX */
  
    platformProtectST25R391xComm();
    platformSpiSelect();
  
    buf[0] = (reg | ST25R3911_READ_MODE);
    buf[1] = 0;
  
    platformSpiRx(buf[0], buf, 1);
  
    if(val != NULL)
    {
      *val = buf[0];
    }
    platformSpiDeselect();
    platformUnprotectST25R391xComm();
    return;
}


void st25r3911ReadMultipleRegisters(uint8_t reg, uint8_t* val, uint8_t length)
{
#if !defined(ST25R391X_COM_SINGLETXRX)
    uint8_t cmd = (reg | ST25R3911_READ_MODE);
#endif  /* !ST25R391X_COM_SINGLETXRX */
  
    platformProtectST25R391xComm();
    platformSpiSelect();
  
#ifdef ST25R391X_COM_SINGLETXRX
  
    ST_MEMSET( comBuf, 0x00, (ST25R3911_CMD_LEN + length) );
    comBuf[0] = (reg | ST25R3911_READ_MODE);
    
    platformSpiTxRx(comBuf, comBuf, (ST25R3911_CMD_LEN + length) );           /* Transceive as a single SPI call                        */
    ST_MEMCPY( val, &comBuf[ST25R3911_CMD_LEN], length );                     /* Copy from local buf to output buffer and skip cmd byte */
  
#else  /* ST25R391X_COM_SINGLETXRX */
  
    /* Since the result comes one byte later, let's first transmit the adddress with discarding the result */

//   platformSpiTxRx(&cmd, NULL, ST25R3911_CMD_LEN);
 //   platformSpiTxRx(NULL, val, length);  

    platformSpiRx(cmd , val, length);

    
  
#endif  /* ST25R391X_COM_SINGLETXRX */

    platformSpiDeselect();
    platformUnprotectST25R391xComm();
    return;
}

void st25r3911ReadTestRegister(uint8_t reg, uint8_t* val)
{
  
#ifdef ST25R391X_COM_SINGLETXRX
    uint8_t* buf = comBuf;
#else  /* ST25R391X_COM_SINGLETXRX */
    uint8_t  buf[3];
#endif  /* ST25R391X_COM_SINGLETXRX */

    platformProtectST25R391xComm();
    platformSpiSelect();

    buf[0] = ST25R3911_CMD_TEST_ACCESS;
    buf[1] = (reg | ST25R3911_READ_MODE);
    buf[2] = 0x00;
  
    platformSpiRx(buf[0], buf, 3);
    
    if(val != NULL)
    {
      *val = buf[0];
    }
    
    platformSpiDeselect();
    platformUnprotectST25R391xComm();

    return;
}

void st25r3911WriteTestRegister(uint8_t reg, uint8_t val)
{
#ifdef ST25R391X_COM_SINGLETXRX
    uint8_t* buf = comBuf;
#else  /* ST25R391X_COM_SINGLETXRX */
    uint8_t  buf[3];
#endif  /* ST25R391X_COM_SINGLETXRX */
    
    platformProtectST25R391xComm();
    platformSpiSelect();

    buf[0] = ST25R3911_CMD_TEST_ACCESS;
    buf[1] = (reg | ST25R3911_WRITE_MODE);
    buf[2] = val;
  
    platformSpiTx(buf[0], &buf[1], 2);
  
    platformSpiDeselect();
    platformUnprotectST25R391xComm();

    return;
}

void st25r3911WriteRegister(uint8_t reg, uint8_t val)
{
#ifdef ST25R391X_COM_SINGLETXRX
    uint8_t* buf = comBuf;
#else  /* ST25R391X_COM_SINGLETXRX */
    uint8_t buf[2];
#endif  /* ST25R391X_COM_SINGLETXRX */
  
    if (ST25R3911_REG_OP_CONTROL == reg)
    {
        st25r3911CheckFieldSetLED(val);
    }    
    
    platformProtectST25R391xComm();
    platformSpiSelect();

    buf[0] = reg | ST25R3911_WRITE_MODE;
    buf[1] = val;
    
    platformSpiTx(buf[0],  &buf[1], 1);
    
    platformSpiDeselect();
    platformUnprotectST25R391xComm();

    return;
}

void st25r3911ClrRegisterBits( uint8_t reg, uint8_t clr_mask )
{
    uint8_t tmp;

    st25r3911ReadRegister(reg, &tmp);
    tmp &= ~clr_mask;
    st25r3911WriteRegister(reg, tmp);
    
    return;
}


void st25r3911SetRegisterBits( uint8_t reg, uint8_t set_mask )
{
    uint8_t tmp;

    st25r3911ReadRegister(reg, &tmp);
    tmp |= set_mask;
    st25r3911WriteRegister(reg, tmp);
    
    return;
}

void st25r3911ChangeRegisterBits(uint8_t reg, uint8_t valueMask, uint8_t value)
{
    st25r3911ModifyRegister(reg, valueMask, (valueMask & value) );
}

void st25r3911ModifyRegister(uint8_t reg, uint8_t clr_mask, uint8_t set_mask)
{
    uint8_t tmp;

    st25r3911ReadRegister(reg, &tmp);

    /* mask out the bits we don't want to change */
    tmp &= ~clr_mask;
    /* set the new value */
    tmp |= set_mask;
    st25r3911WriteRegister(reg, tmp);

    return;
}

void st25r3911ChangeTestRegisterBits( uint8_t reg, uint8_t valueMask, uint8_t value )
{
    uint8_t    rdVal;
    uint8_t    wrVal;
    
    /* Read current reg value */
    st25r3911ReadTestRegister(reg, &rdVal);
    
    /* Compute new value */
    wrVal  = (rdVal & ~valueMask);
    wrVal |= (value & valueMask);
    
    /* Write new reg value */
    st25r3911WriteTestRegister(reg, wrVal );
    
    return;
}

void st25r3911WriteMultipleRegisters(uint8_t reg, const uint8_t* values, uint8_t length)
{ 
#if !defined(ST25R391X_COM_SINGLETXRX)
    uint8_t cmd = (reg | ST25R3911_WRITE_MODE);
#endif  /* !ST25R391X_COM_SINGLETXRX */

    if (reg <= ST25R3911_REG_OP_CONTROL && reg+length >= ST25R3911_REG_OP_CONTROL)
    {
        st25r3911CheckFieldSetLED(values[ST25R3911_REG_OP_CONTROL-reg]);
    }
    
    if (length > 0)
    {
        /* make this operation atomic */
        platformProtectST25R391xComm();
        platformSpiSelect();
    
#ifdef ST25R391X_COM_SINGLETXRX
      
        comBuf[0] = (reg | ST25R3911_WRITE_MODE);
        ST_MEMCPY( &comBuf[ST25R3911_CMD_LEN], values, length );

        platformSpiTxRx( comBuf, NULL, (ST25R3911_CMD_LEN + length) );
      
#else  /*ST25R391X_COM_SINGLETXRX*/    
    
       // platformSpiTxRx( &cmd, NULL, ST25R3911_CMD_LEN );
      
        platformSpiTx( cmd, (uint8_t *)values, length );
    
#endif  /*ST25R391X_COM_SINGLETXRX*/    
    
        platformSpiDeselect();
        platformUnprotectST25R391xComm();
    }
    
    return;
}


void st25r3911WriteFifo(const uint8_t* values, uint8_t length)
{
#if !defined(ST25R391X_COM_SINGLETXRX)
    uint8_t cmd = ST25R3911_FIFO_LOAD;
#endif  /* !ST25R391X_COM_SINGLETXRX */

    if (length > 0)
    {  
        platformProtectST25R391xComm();
        platformSpiSelect();
  
#ifdef ST25R391X_COM_SINGLETXRX
  
        comBuf[0] = ST25R3911_FIFO_LOAD;
        ST_MEMCPY( &comBuf[ST25R3911_CMD_LEN], values, length );

//        platformSpiTxRx( comBuf, NULL, (ST25R3911_CMD_LEN + length) );
			  platformSpiTx( comBuf[0],(uint8_t *)&comBuf[1], length );
  
#else  /*ST25R391X_COM_SINGLETXRX*/
  
       // platformSpiTxRx( &cmd, NULL, ST25R3911_CMD_LEN );
			  //platformSpiTxRx( values, NULL, length );
        platformSpiTx( cmd,(uint8_t *)values, length );
  
#endif  /*ST25R391X_COM_SINGLETXRX*/
  
        platformSpiDeselect();
        platformUnprotectST25R391xComm();
    }

    return;
}

void st25r3911ReadFifo(uint8_t* buf, uint8_t length)
{
#if !defined(ST25R391X_COM_SINGLETXRX)
    uint8_t cmd = ST25R3911_FIFO_READ;
#endif  /* !ST25R391X_COM_SINGLETXRX */
    
    if(length > 0)
    {
        platformProtectST25R391xComm();
        platformSpiSelect();

#ifdef ST25R391X_COM_SINGLETXRX
      
        ST_MEMSET( comBuf, 0x00, (ST25R3911_CMD_LEN + length) );
        comBuf[0] = ST25R3911_FIFO_READ;
      
        platformSpiTxRx( comBuf, comBuf, (ST25R3911_CMD_LEN + length) );         /* Transceive as a single SPI call                        */
        ST_MEMCPY( buf, &comBuf[ST25R3911_CMD_LEN], length );                    /* Copy from local buf to output buffer and skip cmd byte */
  
#else  /*ST25R391X_COM_SINGLETXRX*/
  
     //   platformSpiTxRx( &cmd, NULL, ST25R3911_CMD_LEN );
        platformSpiRx( cmd, buf, length );
  
#endif  /*ST25R391X_COM_SINGLETXRX*/
      
        platformSpiDeselect();
        platformUnprotectST25R391xComm();
    }

    return;
}

void st25r3911ExecuteCommand(uint8_t cmd)
{
#ifdef PLATFORM_LED_FIELD_PIN
    if ( cmd >= ST25R3911_CMD_TRANSMIT_WITH_CRC && cmd <= ST25R3911_CMD_RESPONSE_RF_COLLISION_0)
    {
        platformLedOff(PLATFORM_LED_FIELD_PORT, PLATFORM_LED_FIELD_PIN);
    }
#endif /* PLATFORM_LED_FIELD_PIN */
    
    cmd |= ST25R3911_CMD_MODE;

    platformProtectST25R391xComm();
    platformSpiSelect();
   //  USART_SendData(USART1, cmd);
    SPI_Write_CMD( cmd,ST25R3911_CMD_LEN );
    
    platformSpiDeselect();
    platformUnprotectST25R391xComm();

    return;
}


void st25r3911ExecuteCommands(uint8_t *cmds, uint8_t length)
{
    platformProtectST25R391xComm();
    platformSpiSelect();
    
    platformSpiTx(cmds[0],&cmds[1] , length-1 );
    
    platformSpiDeselect();
    platformUnprotectST25R391xComm();

    return;
}

bool st25r3911IsRegValid( uint8_t reg )
{
    if( !(( (int8_t)reg >= ST25R3911_REG_IO_CONF1) && (reg <= ST25R3911_REG_CAPACITANCE_MEASURE_RESULT)) && 
        (reg != ST25R3911_REG_IC_IDENTITY)                                                         )
    {
        return false;
    }
    return true;
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/

