#include "stm32f0xx.h"
#include "bootlader.h"

#include <string.h>
#define APPLICATION_ADDRESS (uint32_t)0x8003000
#if   (defined ( __CC_ARM ))
  __IO uint32_t VectorTable[48] __attribute__((at(0x20000000)));
#elif (defined (__ICCARM__))
#pragma location = 0x20000000
  __no_init __IO uint32_t VectorTable[48];
#elif defined   (  __GNUC__  )
  __IO uint32_t VectorTable[48] __attribute__((section(".RAMVectorTable")));
#elif defined ( __TASKING__ )
  __IO uint32_t VectorTable[48] __at(0x20000000);
#endif

void IAPAPP_Init()
{
	for(uint8_t i = 0; i < 48; i++)
  {
    VectorTable[i] = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
  }
  /* Enable the SYSCFG peripheral clock*/
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_SYSCFG, ENABLE); 
  /* Remap SRAM at 0x00000000 */
  SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SRAM);
}
int main(void)
{
	//SCB->VTOR =((uint32_t)0x08003000);
	IAPAPP_Init();
	
	HardwareInit();
	printf("Hello, welcome to application\n");
	uint16_t TimeCount=0;
	while(1)
	{
		if((TickCount>100)&&(TimeCount<1000))
		{
			TickCount=0;
			
			printf("wait %d second\n",TimeCount);
			TimeCount++;
			
		}

	}
	return 1;
}
