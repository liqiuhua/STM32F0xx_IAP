这是STM32F0xx系列的Flash bootlader和Application例程，
与STM32F1和F4系列还是有一定的区别的。

STM32F0系列需要在Application程序前面加上以下代码
main（）
{
	memcpy((uint32_t*)0x20000000, (uint32_t*)0x08004000, VECTOR_SIZE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//开时钟，不开有可能不能成功运行中断
	SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SRAM); 
}

此工程还需要在Keil里进行相应的设置。