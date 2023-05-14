#include "stm32f4xx.h"                  // Device header

#define PLL_M 8
#define PLL_N 336
#define PLL_P 4
#define PLL_Q 4
// HSE = 8 000 000
// SYSClock = HSE/M*N/P = 84 000 000
void my_SystemInit(void)
{
	uint32_t StartUpCounter = 0, HSEStatus = 0;
	
	/* FPU settings ------------------------------------------------------------*/
	//#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    //SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
	//#endif
	
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
	
	//#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
	//SystemInit_ExtMemCtl(); 
	//#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

	// Enable HSE
	RCC->CR |= 0x00010000;		//RCC->CR |= ((uint32_t)RCC_CR_HSEON);
	
	/* Wait till HSE is ready and if Time out is reached exit */
	do
	{
		HSEStatus = RCC->CR & RCC_CR_HSERDY;
		StartUpCounter++;
	} while((HSEStatus == 0) && (StartUpCounter != 0x05000));
	
	// HCLK = SYSCLK / 1
    RCC->CFGR |= 0x00000000;	//RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	
	// PCLK2 = HCLK / 1
    RCC->CFGR |= 0x00000000;	//RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
    
    // PCLK1 = HCLK / 2 
    RCC->CFGR |= 0x00001000;	//RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
	
	// Configure the main PLL
	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
                   (0x00400000) | (PLL_Q << 24);
    /*RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
                   (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);		//RCC_PLLCFGR_PLLSRC_HSE = 0x00400000*/

    // Enable the main PLL
    RCC->CR |= 0x01000000;	//RCC->CR |= RCC_CR_PLLON;
	
	/* Wait till the main PLL is ready */
//    while((RCC->CR & RCC_CR_PLLRDY) == 0)
//    {
//    }
	while((RCC->CR & 0x02000000) == 0)
    {
    }
	/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
    //FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_3WS;
	FLASH->ACR = 0x00000100 | 0x00000200 | 0x00000400 | 0x00000003; // 3 waitng periods
	
	/* Select the main PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(0x00000003));	//RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));	/*!< SW[1:0] bits (System clock Switch) */
    RCC->CFGR |= 0x00000002;							//RCC->CFGR |= RCC_CFGR_SW_PLL;	/*PLL selected as system clock */
	
	/* Wait till the main PLL is used as system clock source */
//    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL) 
//    {	/*!< SWS[1:0] bits (System Clock Switch Status) 	PLL/PLLP used as system clock 	*/
//    }
	while ((RCC->CFGR & (uint32_t)0x0000000C ) != 0x00000008)
    {
    }
	
	/* Configure the Vector Table location -------------------------------------*/
	//#if defined(USER_VECT_TAB_ADDRESS)
	//SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
	//#endif /* USER_VECT_TAB_ADDRESS */
	
	// select boot from sram
	SCB->VTOR = 0x08000000;
}

/*
	RCC->PLLCFGR |= 0x00000008; //M=8
	RCC->PLLCFGR |= 0x00005400; //N=336
	RCC->PLLCFGR |= 0x00010000; //p=4
*/

//void SystemInit(void)
//{
//  /* FPU settings ------------------------------------------------------------*/
//  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
//    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
//  #endif
//  /* Reset the RCC clock configuration to the default reset state ------------*/
//  /* Set HSION bit */
//  RCC->CR |= (uint32_t)0x00000001;

//  /* Reset CFGR register */
//  RCC->CFGR = 0x00000000;

//  /* Reset HSEON, CSSON and PLLON bits */
//  RCC->CR &= (uint32_t)0xFEF6FFFF;

//  /* Reset PLLCFGR register */
//  RCC->PLLCFGR = 0x24003010;

//  /* Reset HSEBYP bit */
//  RCC->CR &= (uint32_t)0xFFFBFFFF;

//  /* Disable all interrupts */
//  RCC->CIR = 0x00000000;

//#if defined(DATA_IN_ExtSRAM) || defined(DATA_IN_ExtSDRAM)
//  SystemInit_ExtMemCtl(); 
//#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */
//         
//  /* Configure the System clock source, PLL Multiplier and Divider factors, 
//     AHB/APBx prescalers and Flash settings ----------------------------------*/
//  SetSysClock();

//  /* Configure the Vector Table location add offset address ------------------*/
//#ifdef VECT_TAB_SRAM
//  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
//#else
//  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
//#endif
//}
