#include "stm32f0xx.h"
#include "bootlader.h"

#include <string.h>

int main(void)
{
	HardwareInit();
	printf("Hello, welcome to bootlader\n");
	uint16_t TimeCount=0;
	while(1)
	{
		if((TickCount>100)&&(TimeCount<6))
		{
			TickCount=0;
			
			printf("wait %d second\n",TimeCount);
			TimeCount++;
			
		}
		if((TimeCount>5)&&(BootFlag==0))
		{
			
			JumpToApp();
		}
	}
	return 1;
}
