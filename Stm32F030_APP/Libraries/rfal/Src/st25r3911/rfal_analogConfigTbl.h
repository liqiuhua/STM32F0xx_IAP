
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
 *      PROJECT:   ST25R391x firmware
 *      $Revision: $
 *      LANGUAGE:  ISO C99
 */

/*! \file rfal_analogConfig.h
 *
 *  \author bkam
 *
 *  \brief ST25R3911 Analog Configuration Settings
 *  
 */

#ifndef ST25R3911_ANALOGCONFIG_H
#define ST25R3911_ANALOGCONFIG_H

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "rfal_analogConfig.h"
#include "st25r3911_com.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL MACROS
 ******************************************************************************
 */

/*! Macro for Configuration Setting with only one register-mask-value set: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1] */
#define MODE_ENTRY_1_REG(MODE, R0, M0, V0)              \
    (MODE >> 8), (MODE & 0xFF), 1, (R0 >> 8), (R0 & 0xFF), (M0), (V0)

/*! Macro for Configuration Setting with only two register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1] */
#define MODE_ENTRY_2_REG(MODE, R0, M0, V0, R1, M1, V1)  \
    (MODE >> 8), (MODE & 0xFF), 2, (R0 >> 8), (R0 & 0xFF), (M0), (V0) \
                                 , (R1 >> 8), (R1 & 0xFF), (M1), (V1)

/*! Macro for Configuration Setting with only three register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_3_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2)  \
    (MODE >> 8), (MODE & 0xFF), 3, (R0 >> 8), (R0 & 0xFF), (M0), (V0) \
                                 , (R1 >> 8), (R1 & 0xFF), (M1), (V1) \
                                 , (R2 >> 8), (R2 & 0xFF), (M2), (V2) \

/*! Macro for Configuration Setting with only four register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_4_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3)  \
    (MODE >> 8), (MODE & 0xFF), 4, (R0 >> 8), (R0 & 0xFF), (M0), (V0) \
                                 , (R1 >> 8), (R1 & 0xFF), (M1), (V1) \
                                 , (R2 >> 8), (R2 & 0xFF), (M2), (V2) \
                                 , (R3 >> 8), (R3 & 0xFF), (M3), (V3) \

/*! Macro for Configuration Setting with only five register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_5_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4)  \
    (MODE >> 8), (MODE & 0xFF), 5, (R0 >> 8), (R0 & 0xFF), (M0), (V0) \
                                 , (R1 >> 8), (R1 & 0xFF), (M1), (V1) \
                                 , (R2 >> 8), (R2 & 0xFF), (M2), (V2) \
                                 , (R3 >> 8), (R3 & 0xFF), (M3), (V3) \
                                 , (R4 >> 8), (R4 & 0xFF), (M4), (V4) \

/*! Macro for Configuration Setting with only six register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_6_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4, R5, M5, V5)  \
    (MODE >> 8), (MODE & 0xFF), 6, (R0 >> 8), (R0 & 0xFF), (M0), (V0) \
                                 , (R1 >> 8), (R1 & 0xFF), (M1), (V1) \
                                 , (R2 >> 8), (R2 & 0xFF), (M2), (V2) \
                                 , (R3 >> 8), (R3 & 0xFF), (M3), (V3) \
                                 , (R4 >> 8), (R4 & 0xFF), (M4), (V4) \
                                 , (R5 >> 8), (R5 & 0xFF), (M5), (V5) \
                                 
/*! Macro for Configuration Setting with only seven register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_7_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4, R5, M5, V5, R6, M6, V6)  \
    (MODE >> 8), (MODE & 0xFF), 7, (R0 >> 8), (R0 & 0xFF), (M0), (V0) \
                                 , (R1 >> 8), (R1 & 0xFF), (M1), (V1) \
                                 , (R2 >> 8), (R2 & 0xFF), (M2), (V2) \
                                 , (R3 >> 8), (R3 & 0xFF), (M3), (V3) \
                                 , (R4 >> 8), (R4 & 0xFF), (M4), (V4) \
                                 , (R5 >> 8), (R5 & 0xFF), (M5), (V5) \
                                 , (R6 >> 8), (R6 & 0xFF), (M6), (V6) \

/*! Macro for Configuration Setting with only eight register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_8_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4, R5, M5, V5, R6, M6, V6, R7, M7, V7)  \
    (MODE >> 8), (MODE & 0xFF), 8, (R0 >> 8), (R0 & 0xFF), (M0), (V0) \
                                 , (R1 >> 8), (R1 & 0xFF), (M1), (V1) \
                                 , (R2 >> 8), (R2 & 0xFF), (M2), (V2) \
                                 , (R3 >> 8), (R3 & 0xFF), (M3), (V3) \
                                 , (R4 >> 8), (R4 & 0xFF), (M4), (V4) \
                                 , (R5 >> 8), (R5 & 0xFF), (M5), (V5) \
                                 , (R6 >> 8), (R6 & 0xFF), (M6), (V6) \
                                 , (R7 >> 8), (R7 & 0xFF), (M7), (V7) \

/*! Macro for Configuration Setting with only nine register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_9_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4, R5, M5, V5, R6, M6, V6, R7, M7, V7, R8, M8, V8)  \
    (MODE >> 8), (MODE & 0xFF), 9, (R0 >> 8), (R0 & 0xFF), (M0), (V0) \
                                 , (R1 >> 8), (R1 & 0xFF), (M1), (V1) \
                                 , (R2 >> 8), (R2 & 0xFF), (M2), (V2) \
                                 , (R3 >> 8), (R3 & 0xFF), (M3), (V3) \
                                 , (R4 >> 8), (R4 & 0xFF), (M4), (V4) \
                                 , (R5 >> 8), (R5 & 0xFF), (M5), (V5) \
                                 , (R6 >> 8), (R6 & 0xFF), (M6), (V6) \
                                 , (R7 >> 8), (R7 & 0xFF), (M7), (V7) \
                                 , (R8 >> 8), (R8 & 0xFF), (M8), (V8) \

/*! Macro for Configuration Setting with only ten register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_10_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4, R5, M5, V5, R6, M6, V6, R7, M7, V7, R8, M8, V8, R9, M9, V9)  \
    (MODE >> 8), (MODE & 0xFF),10, (R0 >> 8), (R0 & 0xFF), (M0), (V0) \
                                 , (R1 >> 8), (R1 & 0xFF), (M1), (V1) \
                                 , (R2 >> 8), (R2 & 0xFF), (M2), (V2) \
                                 , (R3 >> 8), (R3 & 0xFF), (M3), (V3) \
                                 , (R4 >> 8), (R4 & 0xFF), (M4), (V4) \
                                 , (R5 >> 8), (R5 & 0xFF), (M5), (V5) \
                                 , (R6 >> 8), (R6 & 0xFF), (M6), (V6) \
                                 , (R7 >> 8), (R7 & 0xFF), (M7), (V7) \
                                 , (R8 >> 8), (R8 & 0xFF), (M8), (V8) \
                                 , (R9 >> 8), (R9 & 0xFF), (M9), (V9) \

/*! Macro for Configuration Setting with eleven register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_11_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4, R5, M5, V5, R6, M6, V6, R7, M7, V7, R8, M8, V8, R9, M9, V9, R10, M10, V10)  \
    (MODE >> 8), (MODE & 0xFF),11, (R0 >> 8), (R0 & 0xFF), (M0), (V0) \
                                 , (R1 >> 8), (R1 & 0xFF), (M1), (V1) \
                                 , (R2 >> 8), (R2 & 0xFF), (M2), (V2) \
                                 , (R3 >> 8), (R3 & 0xFF), (M3), (V3) \
                                 , (R4 >> 8), (R4 & 0xFF), (M4), (V4) \
                                 , (R5 >> 8), (R5 & 0xFF), (M5), (V5) \
                                 , (R6 >> 8), (R6 & 0xFF), (M6), (V6) \
                                 , (R7 >> 8), (R7 & 0xFF), (M7), (V7) \
                                 , (R8 >> 8), (R8 & 0xFF), (M8), (V8) \
                                 , (R9 >> 8), (R9 & 0xFF), (M9), (V9) \
                                 , (R10 >> 8), (R10 & 0xFF), (M10), (V10) \

/*! Macro for Configuration Setting with twelve register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_12_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4, R5, M5, V5, R6, M6, V6, R7, M7, V7, R8, M8, V8, R9, M9, V9, R10, M10, V10, R11, M11, V11)  \
    (MODE >> 8), (MODE & 0xFF),12, (R0 >> 8), (R0 & 0xFF), (M0), (V0) \
                                 , (R1 >> 8), (R1 & 0xFF), (M1), (V1) \
                                 , (R2 >> 8), (R2 & 0xFF), (M2), (V2) \
                                 , (R3 >> 8), (R3 & 0xFF), (M3), (V3) \
                                 , (R4 >> 8), (R4 & 0xFF), (M4), (V4) \
                                 , (R5 >> 8), (R5 & 0xFF), (M5), (V5) \
                                 , (R6 >> 8), (R6 & 0xFF), (M6), (V6) \
                                 , (R7 >> 8), (R7 & 0xFF), (M7), (V7) \
                                 , (R8 >> 8), (R8 & 0xFF), (M8), (V8) \
                                 , (R9 >> 8), (R9 & 0xFF), (M9), (V9) \
                                 , (R10 >> 8), (R10 & 0xFF), (M10), (V10) \
                                 , (R11 >> 8), (R11 & 0xFF), (M11), (V11) \

/*! Macro for Configuration Setting with thirteen register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_13_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4, R5, M5, V5, R6, M6, V6, R7, M7, V7, R8, M8, V8, R9, M9, V9, R10, M10, V10, R11, M11, V11, R12, M12, V12)  \
    (MODE >> 8), (MODE & 0xFF),13, (R0 >> 8), (R0 & 0xFF), (M0), (V0) \
                                 , (R1 >> 8), (R1 & 0xFF), (M1), (V1) \
                                 , (R2 >> 8), (R2 & 0xFF), (M2), (V2) \
                                 , (R3 >> 8), (R3 & 0xFF), (M3), (V3) \
                                 , (R4 >> 8), (R4 & 0xFF), (M4), (V4) \
                                 , (R5 >> 8), (R5 & 0xFF), (M5), (V5) \
                                 , (R6 >> 8), (R6 & 0xFF), (M6), (V6) \
                                 , (R7 >> 8), (R7 & 0xFF), (M7), (V7) \
                                 , (R8 >> 8), (R8 & 0xFF), (M8), (V8) \
                                 , (R9 >> 8), (R9 & 0xFF), (M9), (V9) \
                                 , (R10 >> 8), (R10 & 0xFF), (M10), (V10) \
                                 , (R11 >> 8), (R11 & 0xFF), (M11), (V11) \
                                 , (R12 >> 8), (R12 & 0xFF), (M12), (V12) \
/* Setting for approximately 14%: */
#define AM_MOD_DRIVER_LEVEL_DEFAULT 0xb9 
/*
 ******************************************************************************
 * GLOBAL DATA TYPES
 ******************************************************************************
 */
const uint8_t rfalAnalogConfigDefaultSettings[] = {
      //****** Default Analog Configuration for Chip-Specific. ******/
      MODE_ENTRY_10_REG( RFAL_ANALOG_CONFIG_TECH_CHIP
                      , ST25R3911_REG_OP_CONTROL,             0x30, 0x10 /* default to AM */
                      , ST25R3911_REG_IO_CONF1,               0x06, 0x06 /* MCUCLK: HF clk off */
                      , ST25R3911_REG_IO_CONF1,               (ST25R3911_REG_IO_CONF1_mask_out_cl | ST25R3911_REG_IO_CONF1_lf_clk_off), 0x07 /* MCUCLK: LF clk off */
                      , ST25R3911_REG_IO_CONF2,               0x18, 0x18 /* pull downs */
                      , ST25R3911_REG_RX_CONF4,               ST25R3911_REG_RX_CONF4_mask_rg2_pm, 0x1<<ST25R3911_REG_RX_CONF4_shift_rg2_pm /* increase digitizer window for PM */
                      , ST25R3911_REG_ANT_CAL_TARGET,         0xFF, 0x80 /* 90degrees */
                      , ST25R3911_REG_ANT_CAL_CONTROL,        0xF8, 0x00 /* trim value from calibrate antenna */
                      , ST25R3911_REG_AM_MOD_DEPTH_CONTROL,   ST25R3911_REG_AM_MOD_DEPTH_CONTROL_am_s, ST25R3911_REG_AM_MOD_DEPTH_CONTROL_am_s /* AM modulated level is defined by RFO AM Modulated Level Def Reg, fixed setting, no automatic adjustment */
                      , ST25R3911_REG_FIELD_THRESHOLD,        ST25R3911_REG_FIELD_THRESHOLD_mask_trg, ST25R3911_REG_FIELD_THRESHOLD_trg_75mV
                      , ST25R3911_REG_FIELD_THRESHOLD,        ST25R3911_REG_FIELD_THRESHOLD_mask_rfe, ST25R3911_REG_FIELD_THRESHOLD_rfe_75mV
                      )
                      
      //****** Default Analog Configuration for Poll NFC-A Tx. ******/
    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_RFO_AM_ON_LEVEL, 0xff, 0xf0 /* Used for 848 TX: very high AM to keep wave shapes */
                      )
                      
    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_106 | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am,        0x00        /* OOK */
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_212 | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am,        0x00        /* OOK */
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_424 | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am,        0x00        /* OOK */
                      )

    , MODE_ENTRY_2_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_848 | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,  ST25R3911_REG_AUX_tr_am,  ST25R3911_REG_AUX_tr_am       /* AM! */
                      , ST25R3911_REG_RFO_AM_ON_LEVEL, 0xff, 0xf0 /* Used for 848 TX: very high AM to keep wave shapes */
                      )

      //****** Default Analog Configuration for Poll NFC-A Rx. ******/
    , MODE_ENTRY_3_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF3, 0xff, 0x18
                      , ST25R3911_REG_RX_CONF4,               ST25R3911_REG_RX_CONF4_mask_rg2_am, 0x2<<ST25R3911_REG_RX_CONF4_shift_rg2_am /* increase digitizer window for AM */
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_rx_tol, 0x00                                               /* rx_tol Off */
                      )
                                                                                  
    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_106 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1, 0x7f, 0x00
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_212 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1, 0x7f, 0x04
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_424 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1, 0x7f, 0x22
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_848 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1, 0x7f, 0x22
                      )

      //****** Default Analog Configuration for Poll NFC-B Tx. ******/
    , MODE_ENTRY_2_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am, ST25R3911_REG_AUX_tr_am   /* AM */
                      , ST25R3911_REG_RFO_AM_ON_LEVEL,        0xff, AM_MOD_DRIVER_LEVEL_DEFAULT                  /* Fixed driver for AM level: ~14% */
                      )

      //****** Default Analog Configuration for Poll NFC-B Rx. ******/
    , MODE_ENTRY_3_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF3, 0xff, 0x18
                      , ST25R3911_REG_RX_CONF4,               ST25R3911_REG_RX_CONF4_mask_rg2_am, 0x1<<ST25R3911_REG_RX_CONF4_shift_rg2_am /* increase digitizer window for AM */
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_rx_tol, ST25R3911_REG_AUX_rx_tol                           /* rx_tol On as default */
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_106 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x04
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_212 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x04
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_424 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x22
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_848 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x22
                      )
                      
    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_1695 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x6c
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_3390 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x6c
                      )
                      
      //****** Default Analog Configuration for Poll NFC-F Tx. ******/
    , MODE_ENTRY_2_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am, ST25R3911_REG_AUX_tr_am /* AM */
                      , ST25R3911_REG_RFO_AM_ON_LEVEL,        0xff, AM_MOD_DRIVER_LEVEL_DEFAULT /* Fixed driver for AM level: ~14% */
                      )

      //****** Default Analog Configuration for Poll NFC-F Common bitrate Rx. ******/
    , MODE_ENTRY_3_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF3, 0xff, 0x18
                      , ST25R3911_REG_RX_CONF4,               ST25R3911_REG_RX_CONF4_mask_rg2_am, 0x1<<ST25R3911_REG_RX_CONF4_shift_rg2_am /* increase digitizer window for AM */
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_rx_tol, ST25R3911_REG_AUX_rx_tol                           /* rx_tol On as default */
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_212 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x13   /* dev. from data sheet: lp 300kKz */
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_424 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x0b   /* dev. from data sheet: lp 600kHz */
                      )
                      
    //****** Default Analog Configuration for Poll NFC-V Common bitrate Tx ******/
    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX, ST25R3911_REG_AUX_tr_am, 0x00
                      )

    //****** Default Analog Configuration for Poll NFC-V Common bitrate Rx ******/
    , MODE_ENTRY_4_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1, 0x7f, 0x0c /* use filter settings from table 9: "Recommended for 424/484 kHz sub-carrier" */
                      , ST25R3911_REG_RX_CONF3, 0xff, 0x18
                      , ST25R3911_REG_RX_CONF4,               ST25R3911_REG_RX_CONF4_mask_rg2_am, 0x1<<ST25R3911_REG_RX_CONF4_shift_rg2_am /* increase digitizer window for AM */
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_rx_tol, ST25R3911_REG_AUX_rx_tol                           /* rx_tol On as default */
                      )

      //****** Default Analog Configuration for Poll AP2P Common bitrate Tx. ******/
    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_RFO_AM_ON_LEVEL,        0xff, AM_MOD_DRIVER_LEVEL_DEFAULT /* Fixed driver for AM level: ~14% */
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_106 | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am, 0x00 /* OOK */
                      )

    , MODE_ENTRY_2_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_212 | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am, ST25R3911_REG_AUX_tr_am /* AM */
                      , ST25R3911_REG_RFO_AM_ON_LEVEL,        0xff, AM_MOD_DRIVER_LEVEL_DEFAULT /* Fixed driver for AM level: ~14% */
                      )

    , MODE_ENTRY_2_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_424 | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am, ST25R3911_REG_AUX_tr_am /* AM */
                      , ST25R3911_REG_RFO_AM_ON_LEVEL,        0xff, AM_MOD_DRIVER_LEVEL_DEFAULT /* Fixed driver for AM level: ~14% */
                      )

      //****** Default Analog Configuration for Poll AP2P Common bitrate Rx. ******/
    , MODE_ENTRY_4_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x45
                      , ST25R3911_REG_RX_CONF3,              (ST25R3911_REG_RX_CONF3_lim | ST25R3911_REG_RX_CONF3_rg_nfc), (ST25R3911_REG_RX_CONF3_lim | ST25R3911_REG_RX_CONF3_rg_nfc)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_rx_tol, ST25R3911_REG_AUX_rx_tol                           /* rx_tol On as default */
                      , ST25R3911_REG_RX_CONF4,               ST25R3911_REG_RX_CONF4_mask_rg2_am, 0x1<<ST25R3911_REG_RX_CONF4_shift_rg2_am /* increase digitizer window for AM */
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_106 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF3,               ST25R3911_REG_RX_CONF3_mask_rg1_am, 0xc0
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_212 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF3,               ST25R3911_REG_RX_CONF3_mask_rg1_am, 0x00
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_424 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF3,               ST25R3911_REG_RX_CONF3_mask_rg1_am, 0x00
                      )

      //****** Default Analog Configuration for Listen AP2P Common bitrate Tx. ******/
      , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX)
                        , ST25R3911_REG_RFO_AM_ON_LEVEL,        0xff, AM_MOD_DRIVER_LEVEL_DEFAULT /* Fixed driver for AM level: ~14% */
                        )

      , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_106 | RFAL_ANALOG_CONFIG_TX)
                        , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am, 0x00 /* OOK */
                        )

      , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_212 | RFAL_ANALOG_CONFIG_TX)
                        , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am, ST25R3911_REG_AUX_tr_am /* AM */
                        )

      , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_424 | RFAL_ANALOG_CONFIG_TX)
                        , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am, ST25R3911_REG_AUX_tr_am /* AM */
                        )

        //****** Default Analog Configuration for Listen AP2P Common bitrate Rx. ******/
      , MODE_ENTRY_3_REG( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX)
                        , ST25R3911_REG_RX_CONF1,               0x7f, 0x45
                        , ST25R3911_REG_RX_CONF3,              (ST25R3911_REG_RX_CONF3_lim | ST25R3911_REG_RX_CONF3_rg_nfc), (ST25R3911_REG_RX_CONF3_lim | ST25R3911_REG_RX_CONF3_rg_nfc)
                        , ST25R3911_REG_RX_CONF4,               ST25R3911_REG_RX_CONF4_mask_rg2_am, 0x1<<ST25R3911_REG_RX_CONF4_shift_rg2_am /* increase digitizer window for AM */
                        )

      , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_106 | RFAL_ANALOG_CONFIG_RX)
                        , ST25R3911_REG_RX_CONF3,               ST25R3911_REG_RX_CONF3_mask_rg1_am, 0xc0
                        )

      , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_212 | RFAL_ANALOG_CONFIG_RX)
                        , ST25R3911_REG_RX_CONF3,               ST25R3911_REG_RX_CONF3_mask_rg1_am, 0x00
                        )

      , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_424 | RFAL_ANALOG_CONFIG_RX)
                        , ST25R3911_REG_RX_CONF3,               ST25R3911_REG_RX_CONF3_mask_rg1_am, 0x00
                        )
};

#endif /* ST25R3911_ANALOGCONFIG_H */
