
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
 *  \brief ST25R3911 high level interface
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "st25r3911.h"
#include "st25r3911_com.h"
#include "st25r3911_interrupt.h"
#include "utils.h"

/*
******************************************************************************
* LOCAL DEFINES
******************************************************************************
*/

#define ST25R3911_OSC_STABLE_TIMEOUT           10 /*!< Timeout for Oscillator to get stable, datasheet: 700us, take 5 ms */
#define ST25R3911_CA_TIMEOUT                   10 /*!< Timeout for Collision Avoidance command                           */

/*
******************************************************************************
* LOCAL CONSTANTS
******************************************************************************
*/
/*< ST25R3916  RSSI Display Reg values:      0   1   2   3   4   5   6    7    8   9    a     b    c    d  e  f */
static const uint16_t st25r3911Rssi2mV[] = { 0 ,20 ,27 ,37 ,52 ,72 ,99 ,136 ,190 ,262 ,357 ,500 ,686 ,950, 0, 0 };

/* ST25R3916 2/3 stage gain reduction [dB]          0    0    0    0    0    3    6    9   12   15   18  na na na na na */
static const uint16_t st25r3911Gain2Percent[] = { 100, 100, 100, 100, 100, 141, 200, 281, 398, 562, 794, 1, 1, 1, 1, 1 };

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/
static uint32_t st25r3911NoResponseTime_64fcs;

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/
static ReturnCode st25r3911ExecuteCommandAndGetResult(uint8_t cmd, uint8_t resreg, uint8_t sleeptime, uint8_t* result);

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/

void st25r3911TxRxOn( void )
{
    st25r3911SetRegisterBits(ST25R3911_REG_OP_CONTROL, (ST25R3911_REG_OP_CONTROL_rx_en | ST25R3911_REG_OP_CONTROL_tx_en) );
}

void st25r3911TxRxOff( void )
{
    st25r3911ClrRegisterBits(ST25R3911_REG_OP_CONTROL, (ST25R3911_REG_OP_CONTROL_rx_en | ST25R3911_REG_OP_CONTROL_tx_en) );
}


void st25r3911OscOn( void )
{
    /* Check if oscillator is already turned on and stable                                                */        
    /* Use ST25R3916_REG_OP_CONTROL_en instead of ST25R3916_REG_AUX_DISPLAY_osc_ok to be on the safe side */    
    if( !st25r3911CheckReg( ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_en, ST25R3911_REG_OP_CONTROL_en ) )
    {
        /* Clear any eventual previous oscillator IRQ */
        st25r3911GetInterrupt( ST25R3911_IRQ_MASK_OSC );
      
        /* enable oscillator frequency stable interrupt */
        st25r3911EnableInterrupts(ST25R3911_IRQ_MASK_OSC);

        /* enable oscillator and regulator output */
        st25r3911ModifyRegister(ST25R3911_REG_OP_CONTROL, 0x00, ST25R3911_REG_OP_CONTROL_en);

        /* wait for the oscillator interrupt */
        st25r3911WaitForInterruptsTimed(ST25R3911_IRQ_MASK_OSC, ST25R3911_OSC_STABLE_TIMEOUT);
        st25r3911DisableInterrupts(ST25R3911_IRQ_MASK_OSC);
    }
    
}


void st25r3911Initialize()
{
    uint16_t vdd_mV;

    /* first, reset the st25r3911 */
    st25r3911ExecuteCommand(ST25R3911_CMD_SET_DEFAULT);
//               uint8_t testRead;
//        USART_SendData(USART1, 0x18);
//        st25r3911ReadRegister(0x18, &testRead);
//
//        USART_SendData(USART1, testRead);

        
    /* enable pull downs on miso line */
    st25r3911ModifyRegister(ST25R3911_REG_IO_CONF2, 0, 
            ST25R3911_REG_IO_CONF2_miso_pd1 |
            ST25R3911_REG_IO_CONF2_miso_pd2);

    /* after reset all interrupts are enabled. so disable them at first */
    st25r3911DisableInterrupts(ST25R3911_IRQ_MASK_ALL);
    /* and clear them, just to be sure... */
    st25r3911ClearInterrupts();

    /* trim settings for VHBR board, will anyway changed later on */
    st25r3911WriteRegister(ST25R3911_REG_ANT_CAL_TARGET, 0x80);
    
    st25r3911OscOn();
    
    /* Measure vdd and set sup3V bit accordingly */
    vdd_mV = st25r3911MeasureVoltage(ST25R3911_REG_REGULATOR_CONTROL_mpsv_vdd);

    st25r3911ModifyRegister(ST25R3911_REG_IO_CONF2,
                         ST25R3911_REG_IO_CONF2_sup3V,
                         (vdd_mV < 3600)?ST25R3911_REG_IO_CONF2_sup3V:0);

    /* Make sure Transmitter and Receiver are disabled */
    st25r3911TxRxOff();
    
    return;
}

void st25r3911Deinitialize()
{
    st25r3911DisableInterrupts(ST25R3911_IRQ_MASK_ALL);    

    // Disabe Tx and Rx, Keep OSC
    st25r3911TxRxOff();

    return;
}

ReturnCode st25r3911AdjustRegulators(uint16_t* result_mV)
{
    uint8_t result;
    uint8_t io_conf2;
    ReturnCode err = ERR_NONE;

    /* first check the status of the reg_s bit in ST25R3911_REG_VREG_DEF register.
       if this bit is set adjusting the regulators is not allowed */
    st25r3911ReadRegister(ST25R3911_REG_REGULATOR_CONTROL, &result);

    if (result & ST25R3911_REG_REGULATOR_CONTROL_reg_s)
    {
        return ERR_REQUEST;
    }

    st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_ADJUST_REGULATORS,
                                    ST25R3911_REG_REGULATOR_RESULT,
                                    5,
                                    &result);
    st25r3911ReadRegister(ST25R3911_REG_IO_CONF2, &io_conf2);

    result >>= ST25R3911_REG_REGULATOR_RESULT_shift_reg;
    result -= 5;
    if (result_mV)
    {
        if(io_conf2 & ST25R3911_REG_IO_CONF2_sup3V)
        {
            *result_mV = 2400;
            *result_mV += result * 100;
        }
        else
        {
            *result_mV = 3900;
            *result_mV += result * 120;
        }
    }
    return err;
}

void st25r3911MeasureAmplitude(uint8_t* result)
{
    st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_MEASURE_AMPLITUDE,
                                    ST25R3911_REG_AD_RESULT,
                                    10,
                                    result);
}

void st25r3911MeasurePhase(uint8_t* result)
{
    st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_MEASURE_PHASE,
                                    ST25R3911_REG_AD_RESULT,
                                    10,
                                    result);
}

void st25r3911MeasureCapacitance(uint8_t* result)       //发送直接指令ST25R3911_CMD_MEASURE_CAPACITANCE开始电容测量，并读取AD转换寄存器ST25R3911_REG_AD_RESULT的值赋给result
{
    st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_MEASURE_CAPACITANCE, 
                                    ST25R3911_REG_AD_RESULT,
                                    10,
                                    result);  
}

void st25r3911CalibrateAntenna(uint8_t* result)
{
    st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_CALIBRATE_ANTENNA,
                                    ST25R3911_REG_ANT_CAL_RESULT,
                                    10,
                                    result);
}

void st25r3911CalibrateModulationDepth(uint8_t* result)
{
    st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_CALIBRATE_MODULATION,
                                    ST25R3911_REG_AM_MOD_DEPTH_RESULT,
                                    10,
                                    result);
}


void st25r3911CalibrateCapacitiveSensor(uint8_t* result)
{
  st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_CALIBRATE_C_SENSOR,
                                    ST25R3911_REG_CAP_SENSOR_RESULT,
                                    10,
                                    result);
}


ReturnCode st25r3911SetBitrate(uint8_t txRate, uint8_t rxRate)
{
    uint8_t reg;

    st25r3911ReadRegister(ST25R3911_REG_BIT_RATE, &reg);
    if (rxRate != ST25R3911_BR_DO_NOT_SET)
    {
        if(rxRate > ST25R3911_BR_3390)
        {
            return ERR_PARAM;
        }
        else
        {
            reg &= ~ST25R3911_REG_BIT_RATE_mask_rxrate;
            reg |= rxRate << ST25R3911_REG_BIT_RATE_shift_rxrate;
        }
    }
    if (txRate != ST25R3911_BR_DO_NOT_SET)
    {
        if(txRate > ST25R3911_BR_6780)
        {
            return ERR_PARAM;
        }
        else
        {
            reg &= ~ST25R3911_REG_BIT_RATE_mask_txrate;
            reg |= txRate<<ST25R3911_REG_BIT_RATE_shift_txrate;
        }
    }
    st25r3911WriteRegister(ST25R3911_REG_BIT_RATE, reg);
    
    return ERR_NONE;
}

uint8_t st25r3911MeasurePowerSupply( uint8_t mpsv )
{
    uint8_t result; 
   
    /* Set the source of direct command: Measure Power Supply Voltage */
    st25r3911ChangeRegisterBits( ST25R3911_REG_REGULATOR_CONTROL, ST25R3911_REG_REGULATOR_CONTROL_mask_mpsv, mpsv );

    /* Execute command: Measure Power Supply Voltage */
    st25r3911ExecuteCommandAndGetResult( ST25R3911_CMD_MEASURE_VDD, ST25R3911_REG_AD_RESULT, 10, &result);

    return result;
}

uint16_t st25r3911MeasureVoltage(uint8_t mpsv)
{
    uint8_t result; 
    uint16_t mV;

    result = st25r3911MeasurePowerSupply( mpsv );

    mV = ((uint16_t)result) * 23;
    mV += ((((uint16_t)result) * 438) + 500) / 1000;

    return mV;
}


uint8_t st25r3911GetNumFIFOLastBits( void )
{
    uint8_t  reg;
    
    st25r3911ReadRegister( ST25R3911_REG_FIFO_RX_STATUS2, &reg );
    
    return ((reg & ST25R3911_REG_FIFO_RX_STATUS2_mask_fifo_lb) >> ST25R3911_REG_FIFO_RX_STATUS2_shift_fifo_lb);
}

uint32_t st25r3911GetNoResponseTime_64fcs()
{
    return st25r3911NoResponseTime_64fcs;
}

void st25r3911StartGPTimer_8fcs(uint16_t gpt_8fcs, uint8_t trigger_source)
{
    st25r3911SetGPTime_8fcs(gpt_8fcs);

    st25r3911ModifyRegister(ST25R3911_REG_GPT_CONTROL, 
            ST25R3911_REG_GPT_CONTROL_gptc_mask, 
            trigger_source);
    if (!trigger_source)
        st25r3911ExecuteCommand(ST25R3911_CMD_START_GP_TIMER);

    return;
}

void st25r3911SetGPTime_8fcs(uint16_t gpt_8fcs)
{
    st25r3911WriteRegister(ST25R3911_REG_GPT1, gpt_8fcs >> 8);
    st25r3911WriteRegister(ST25R3911_REG_GPT2, gpt_8fcs & 0xff);

    return;
}

bool st25r3911CheckReg( uint8_t reg, uint8_t mask, uint8_t val )
{
    uint8_t regVal;
    
    regVal = 0;
    st25r3911ReadRegister( reg, &regVal );
    return ((regVal & mask) == val );
}

#include "stm32f0xx_usart.h"

bool st25r3911CheckChipID( uint8_t *rev )
{
    uint8_t ID;
    
    ID = 0;    
    st25r3911ReadRegister( ST25R3911_REG_IC_IDENTITY, &ID );

    /* Check if IC Identity Register contains ST25R3911's IC type code */
    if( (ID & ST25R3911_REG_IC_IDENTITY_mask_ic_type) != ST25R3911_REG_IC_IDENTITY_ic_type )
    {
        return false;
    }
  //  
    if(rev != NULL)
    {
        *rev = (ID & ST25R3911_REG_IC_IDENTITY_mask_ic_rev);
    }
  //  USART_SendData(USART1,ID);
    return true;
}

ReturnCode st25r3911SetNoResponseTime_64fcs(uint32_t nrt_64fcs)
{
    ReturnCode err = ERR_NONE;
    uint8_t nrt_step = 0;

    st25r3911NoResponseTime_64fcs = nrt_64fcs;
    if (nrt_64fcs > USHRT_MAX)
    {
        nrt_step = ST25R3911_REG_GPT_CONTROL_nrt_step;
        nrt_64fcs = (nrt_64fcs + 63) / 64;
        if (nrt_64fcs > USHRT_MAX)
        {
            nrt_64fcs = USHRT_MAX;
            err = ERR_PARAM;
        }
        st25r3911NoResponseTime_64fcs = 64 * nrt_64fcs;
    }

    st25r3911ModifyRegister(ST25R3911_REG_GPT_CONTROL, ST25R3911_REG_GPT_CONTROL_nrt_step, nrt_step);
    st25r3911WriteRegister(ST25R3911_REG_NO_RESPONSE_TIMER1, nrt_64fcs >> 8);
    st25r3911WriteRegister(ST25R3911_REG_NO_RESPONSE_TIMER2, nrt_64fcs & 0xff);

    return err;
}

ReturnCode st25r3911SetStartNoResponseTime_64fcs(uint32_t nrt_64fcs)
{
    ReturnCode err;
    
    err = st25r3911SetNoResponseTime_64fcs( nrt_64fcs );
    if(err == ERR_NONE)
    {
        st25r3911ExecuteCommand(ST25R3911_CMD_START_NO_RESPONSE_TIMER);
    }
    
    return err;
}

ReturnCode st25r3911PerformCollisionAvoidance( uint8_t FieldONCmd, uint8_t pdThreshold, uint8_t caThreshold, uint8_t nTRFW )
{
    uint8_t  treMask;
    uint32_t irqs;
    
    if( (FieldONCmd != ST25R3911_CMD_INITIAL_RF_COLLISION)    && 
        (FieldONCmd != ST25R3911_CMD_RESPONSE_RF_COLLISION_0) && 
        (FieldONCmd != ST25R3911_CMD_RESPONSE_RF_COLLISION_N)   )
    {
        return ERR_PARAM;
    }
    
    /* Check if new thresholds are to be applied */
    if( (pdThreshold != ST25R3911_THRESHOLD_DO_NOT_SET) || (caThreshold != ST25R3911_THRESHOLD_DO_NOT_SET) )
    {
        treMask = 0;
        
        if(pdThreshold != ST25R3911_THRESHOLD_DO_NOT_SET)
        {
            treMask |= ST25R3911_REG_FIELD_THRESHOLD_mask_trg;
        }
        
        if(caThreshold != ST25R3911_THRESHOLD_DO_NOT_SET)
        {
            treMask |= ST25R3911_REG_FIELD_THRESHOLD_mask_rfe;
        }
            
        /* Set Detection Threshold and|or Collision Avoidance Threshold */
        st25r3911ChangeRegisterBits( ST25R3911_REG_FIELD_THRESHOLD, treMask, (pdThreshold & ST25R3911_REG_FIELD_THRESHOLD_mask_trg) | (caThreshold & ST25R3911_REG_FIELD_THRESHOLD_mask_rfe ) );
    }
    
    /* Set n x TRFW */
    st25r3911ModifyRegister(ST25R3911_REG_AUX, ST25R3911_REG_AUX_mask_nfc_n, (nTRFW & ST25R3911_REG_AUX_mask_nfc_n) );
    
    /* Enable and clear CA specific interrupts and execute command */
    st25r3911EnableInterrupts( (ST25R3911_IRQ_MASK_CAC | ST25R3911_IRQ_MASK_CAT) );
    st25r3911GetInterrupt( (ST25R3911_IRQ_MASK_CAC | ST25R3911_IRQ_MASK_CAT) );
    
    st25r3911ExecuteCommand(FieldONCmd);
    
    irqs = st25r3911WaitForInterruptsTimed(ST25R3911_IRQ_MASK_CAC | ST25R3911_IRQ_MASK_CAT, ST25R3911_CA_TIMEOUT );
    
    /* Clear any previous External Field events and disable CA specific interrupts */
    st25r3911GetInterrupt( (ST25R3911_IRQ_MASK_EOF | ST25R3911_IRQ_MASK_EON) );
    st25r3911DisableInterrupts(ST25R3911_IRQ_MASK_CAC | ST25R3911_IRQ_MASK_CAT);
    
    
    if(ST25R3911_IRQ_MASK_CAC & irqs)                             /* Collision occurred */
    {
        return ERR_RF_COLLISION;
    }
    
    if(ST25R3911_IRQ_MASK_CAT & irqs)                             /* No Collision detected, Field On */
    {
        return ERR_NONE;
    }

    /* No interrupt detected */
    return ERR_INTERNAL;
}

ReturnCode st25r3911GetRegsDump(uint8_t* resRegDump, uint8_t* sizeRegDump)
{
    uint8_t regIt;
    uint8_t regDump[ST25R3911_REG_IC_IDENTITY+1];
    
    if(!sizeRegDump || !resRegDump)
    {
        return ERR_PARAM;
    }
    
    for( regIt = ST25R3911_REG_IO_CONF1; regIt < SIZEOF_ARRAY(regDump); regIt++ )
    {
        st25r3911ReadRegister(regIt, &regDump[regIt] );
    }
    
    *sizeRegDump = MIN(*sizeRegDump, regIt);    
    ST_MEMCPY(resRegDump, regDump, *sizeRegDump );

    return ERR_NONE;
}


void st25r3911SetNumTxBits( uint32_t nBits )
{
    st25r3911WriteRegister(ST25R3911_REG_NUM_TX_BYTES2, (uint8_t)((nBits >> 0) & 0xff)); 
    st25r3911WriteRegister(ST25R3911_REG_NUM_TX_BYTES1, (uint8_t)((nBits >> 8) & 0xff));    
}


bool st25r3911IsCmdValid( uint8_t cmd )
{
    if( !((cmd >= ST25R3911_CMD_SET_DEFAULT)       && (cmd <= ST25R3911_CMD_ANALOG_PRESET))           && 
        !((cmd >= ST25R3911_CMD_MASK_RECEIVE_DATA) && (cmd <= ST25R3911_CMD_CLEAR_RSSI))              &&
        !((cmd >= ST25R3911_CMD_TRANSPARENT_MODE)  && (cmd <= ST25R3911_CMD_START_NO_RESPONSE_TIMER)) &&
        !((cmd >= ST25R3911_CMD_TEST_CLEARA)       && (cmd <= ST25R3911_CMD_FUSE_PPROM))               )        
    {
        return false;
    }
    return true;
}

ReturnCode st25r3911StreamConfigure(const struct st25r3911StreamConfig *config)
{
    uint8_t smd = 0;
    uint8_t mode;

    if (config->useBPSK)
    {
        mode = ST25R3911_REG_MODE_om_bpsk_stream;
        if (config->din<2 || config->din>4) /* not in fc/4 .. fc/16 */
        {
            return ERR_PARAM;
        }
        smd |= (4 - config->din) << ST25R3911_REG_STREAM_MODE_shift_scf;

    }
    else
    {
        mode = ST25R3911_REG_MODE_om_subcarrier_stream;
        if (config->din<3 || config->din>6) /* not in fc/8 .. fc/64 */
        {
            return ERR_PARAM;
        }
        smd |= (6 - config->din) << ST25R3911_REG_STREAM_MODE_shift_scf;
        if (config->report_period_length == 0) 
        {
            return ERR_PARAM;
        }
    }

    if (config->dout<1 || config->dout>7) /* not in fc/2 .. fc/128 */
    {
        return ERR_PARAM;
    }
    smd |= (7 - config->dout) << ST25R3911_REG_STREAM_MODE_shift_stx;

    if (config->report_period_length > 3) 
    {
        return ERR_PARAM;
    }
    smd |= config->report_period_length << ST25R3911_REG_STREAM_MODE_shift_scp;

    st25r3911WriteRegister(ST25R3911_REG_STREAM_MODE, smd);
    st25r3911ChangeRegisterBits(ST25R3911_REG_MODE, ST25R3911_REG_MODE_mask_om, mode);

    return ERR_NONE;
}

/*******************************************************************************/
ReturnCode st25r3911GetRSSI( uint16_t *amRssi, uint16_t *pmRssi )
{
    uint8_t  rssi;
    uint8_t  gainRed;
    
    st25r3911ReadRegister( ST25R3911_REG_RSSI_RESULT, &rssi );
    st25r3911ReadRegister( ST25R3911_REG_GAIN_RED_STATE, &gainRed );
    
    if( amRssi != NULL )
    {
        *amRssi = (uint16_t) ( (uint32_t)( st25r3911Rssi2mV[ (rssi >> ST25R3911_REG_RSSI_RESULT_shift_rssi_am) ] * st25r3911Gain2Percent[ (gainRed >> ST25R3911_REG_GAIN_RED_STATE_shift_gs_am) ] ) / 100 );
    }
    
    if( pmRssi != NULL )
    {
        *pmRssi = (uint16_t) ( (uint32_t)( st25r3911Rssi2mV[ (rssi & ST25R3911_REG_RSSI_RESULT_mask_rssi_pm) ] * st25r3911Gain2Percent[ (gainRed & ST25R3911_REG_GAIN_RED_STATE_mask_gs_pm) ] ) / 100 );
    }
    
    return ERR_NONE;
}

bool st25r3911IrqIsWakeUpCap( void )
{
  return ( st25r3911GetInterrupt(ST25R3911_IRQ_MASK_WCAP) ? true : false );
}


bool st25r3911IrqIsWakeUpPhase( void )
{
  return ( st25r3911GetInterrupt(ST25R3911_IRQ_MASK_WPH) ? true : false );
}


bool st25r3911IrqIsWakeUpAmplitude( void )
{
  return ( st25r3911GetInterrupt(ST25R3911_IRQ_MASK_WAM) ? true : false );
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
/*! 
 *****************************************************************************
 *  \brief  Executes a direct command and returns the result
 *
 *  This function executes the direct command given by \a cmd waits for
 *  \a sleeptime and returns the result read from register \a resreg.
 *
 *  \param[in] cmd: direct command to execute.
 *  \param[in] resreg: Address of the register containing the result.
 *  \param[in] sleeptime: time in milliseconds to wait before reading the result.
 *  \param[out] result: 8 bit long result
 *
 *****************************************************************************
 */
static ReturnCode st25r3911ExecuteCommandAndGetResult(uint8_t cmd, uint8_t resreg, uint8_t sleeptime, uint8_t* result)
{

    if (   (cmd >= ST25R3911_CMD_INITIAL_RF_COLLISION && cmd <= ST25R3911_CMD_RESPONSE_RF_COLLISION_0)
            || (cmd == ST25R3911_CMD_MEASURE_AMPLITUDE)
            || (cmd >= ST25R3911_CMD_ADJUST_REGULATORS && cmd <= ST25R3911_CMD_MEASURE_PHASE)
            || (cmd >= ST25R3911_CMD_CALIBRATE_C_SENSOR && cmd <= ST25R3911_CMD_MEASURE_VDD)
            || (cmd >= 0xFD && cmd <= 0xFE )
       )
    {
        st25r3911EnableInterrupts(ST25R3911_IRQ_MASK_DCT);

        st25r3911GetInterrupt(ST25R3911_IRQ_MASK_DCT);

        st25r3911ExecuteCommand(cmd);

        st25r3911WaitForInterruptsTimed(ST25R3911_IRQ_MASK_DCT, sleeptime);
        st25r3911DisableInterrupts(ST25R3911_IRQ_MASK_DCT);
    }
    else
    {
        return ERR_PARAM;
    }

    /* read out the result if the pointer is not NULL */
    if (result)
		{
			  
			  st25r3911ReadRegister(resreg, result);
		}
        

    return ERR_NONE;

}
