/**
  ******************************************************************************
  * @file    STM32F0xx_IAP/src/stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    29-May-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"

/** @addtogroup STM32F0xx_IAP
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
  	
extern __weak void HAL_IncTick(void);

/*

__weak void HAL_IncTick(void)
{
  uwTick++;
}
*/

uint32_t TickCount=0;
void SysTick_Handler(void)
{
    //HAL_IncTick();

    TickCount++;


}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval None
  */
	

void  TIM1_BRK_UP_TRG_COM_IRQHandler (void)
{
	if ( TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET ) 
	{		
		TIM_ClearITPendingBit(TIM1 , TIM_FLAG_Update);  		 
	}		 	
}

void TIM3_IRQHandler(void)
{
	 if ( TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET ) 
	{		
		TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);  		 
	}	
}

void EXTI0_1_IRQHandler(void)
{
    if((EXTI->PR & EXTI_Line0) != (uint32_t)RESET)
    {

        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
