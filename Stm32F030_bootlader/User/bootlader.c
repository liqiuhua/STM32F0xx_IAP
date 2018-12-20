#include "stm32f0xx.h"

#include <stdio.h>
#define BOOTLADER_SIZE ((uint32_t)1024 * 12)

#define BOOT_START_ADDRESS ((uint32_t)0x08000000)
#define BOOT_FLASH_END_ADDRESS ((uint32_t)BOOT_START_ADDRESS+BOOTLADER_SIZE-1)
#define BOOT_FLASH_SIZE (BOOT_FLASH_END_ADDRESS-BOOT_START_ADDRESS+1)

#define APP_START_ADDRESS ((uint32_t)(BOOT_FLASH_END_ADDRESS+1))

uint8_t BootFlag;
void Usart1Init(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

    //AF config
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_UP;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1,&USART_InitStructure);
    USART_Cmd(USART1,ENABLE);
    
    USART_ClearFlag(USART1,USART_FLAG_TC);

    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 3 ;
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

int USART1_Printf(int8_t *data,int length)
{
  for(int i = 0;i<length;i++)
  {
      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, *data);
    data++;
  }

}

void USART1_IRQHandler(void)
{
  uint8_t RcvData;
  	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  
	{
		RcvData =USART_ReceiveData(USART1);
        if(RcvData=='U')
        {
            BootFlag=1;
        }
		USART_SendData(USART1,RcvData);
  } 
}
static void ShowVersion(void)
{
    printf("************** Welcome bootlader 1.0 ***************\n");
    printf("****************************************************\n");
}

void HardwareInit(void)
{
    /* close all interrupt*/
    __disable_irq();

   // RCC_DeInit();//为了从IAP程序中跳出运行APP程序的时候正常开始，初始化时要RCC为复位状态，恢复NVIC

    SystemInit();
    extern uint32_t SystemCoreClock;
    SysTick_Config(SystemCoreClock/600);
    Usart1Init(115200);

    ShowVersion();

    __enable_irq();

}
typedef void (*pFunction)(void);
__IO pFunction JumpToAplication;
void JumpToApp(void)
{
   // printf("Run Application %x \n",(*(__IO uint32_t*)APP_START_ADDRESS));
    if(((*(__IO uint32_t*)APP_START_ADDRESS) & 0x2ffe0000)==0x20000000)
    {
            
        printf("Go to application and runnig\n");

        __disable_irq();
        JumpToAplication=(pFunction)(*(__IO uint32_t*)(APP_START_ADDRESS+4));
        __set_MSP(*(__IO uint32_t*)APP_START_ADDRESS);
        
        JumpToAplication();

    }
}
