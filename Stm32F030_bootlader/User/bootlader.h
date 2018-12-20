#ifndef __BOOTLADER_H_
#define __BOOTLADER_H_

#include <stdio.h>




void HardwareInit(void);
int USART1_Printf(int8_t *data,int length);
void JumpToApp(void);

extern uint8_t BootFlag;
extern uint32_t TickCount;











//#pragma import(__use_no_semihosting)                              
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;     

//_sys_exit(int x) 
//{ 
//	x = x; 
//} 



int fputc(int ch, FILE *f)
{
  	USART_SendData(USART1, (uint8_t) ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)    //μè′y·￠?ííê±?
	{
	
	}
  	return ch;
}

#endif
