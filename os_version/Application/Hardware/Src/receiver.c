#include "receiver.h"
#include "motor.h"
#include "bluetooth.h"
#include "os_cpu.h"
#include "ucos_ii.h"
#include <sys/_stdint.h>

uint32_t PulseWidth, Period, DutyCycle;
uint32_t ppm_val[10], ppm_cnt = 0;
BOOLEAN ppm_started, ppm_error = 0;

void MY_NVIC_PriorityGroupConfig(uint8_t NVIC_Group)
{
	uint32_t temp,temp1;
	temp1 = (~(NVIC_Group)&0x07);
	temp1<<=8;
	temp = SCB->AIRCR;
	temp &= 0x0000F8FF;
	temp |= 0x05FA0000;
	temp |= temp1;
	SCB->AIRCR = temp;
}

void TIM1_Init(void)
{
	// PA8 : TIM1CH1
	RCC->AHB1ENR |= 1 << 0;
	RCC->APB2ENR |= 1 << 0;
	GPIOA->AFR[1] |= 1 << 0;
	GPIOA->MODER &= ~(3 << (2*8));
	GPIOA->MODER |= 2 << (2*8);//锟斤拷锟�?��拷锟斤拷
	GPIOA->OTYPER &=~ (1 << 8);//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�?
	GPIOA->OSPEEDR |= 2 << (2*8);//50Mhz
	GPIOA->PUPDR &= ~(3 << (2*8));//no pull and down
	// 时锟斤拷锟斤拷元
	TIM1->ARR = 0;
	TIM1->CR1 &=~ (1 << 4);//锟斤拷锟斤拷锟斤拷锟斤拷
	TIM1->PSC |= 83;
	TIM1->ARR |= 19999;
	TIM1->EGR |= 1 << 1;//锟斤拷锟斤拷EGR锟侥达拷锟斤拷锟斤拷CC1G位锟斤拷使锟�??拷锟今到憋拷锟斤拷锟脚号猴拷筒锟斤拷锟揭伙拷锟斤拷锟斤拷锟斤拷录锟�
	TIM1->CR1 |= 1 << 7;//ARR使锟斤拷
	// 锟斤拷锟斤拷通锟斤拷1
	TIM1->CCMR1 |= 1 << (2*0);//通锟斤拷1锟侥诧拷锟斤拷锟脚猴拷IC1锟斤拷映锟戒到锟斤拷锟斤拷锟斤拷TI1锟斤�?
	TIM1->CCMR1 &=~ (3 << (2*1));//锟斤拷锟皆憋拷锟斤拷锟脚号斤拷锟�?��拷�?锟斤拷锟斤拷
	TIM1->CCMR1 &=~ (15 << (4*1));//锟剿诧拷锟斤拷为锟斤�?
	TIM1->CCER |= 1 << 0;//CC1使锟斤拷
	TIM1->CCER &=~ (1 << 1);//锟斤拷路锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�?��拷锟斤拷锟斤拷锟斤拷
	TIM1->CCER &=~ (1 << 3);
	//锟斤拷锟斤拷通锟斤拷2
	// TIM1->CCMR1 |= 2 << (2*4);//通锟斤拷2锟侥诧拷锟斤拷锟脚猴拷IC2锟斤拷映锟戒到锟斤拷锟斤拷锟斤拷TI1锟斤�? 
	// TIM1->CCMR1 &=~ (3 << (2*5));//锟斤拷锟皆憋拷锟斤拷锟脚号斤拷锟�?��拷�?锟斤拷锟斤拷
	// TIM1->CCMR1 &=~ (15 << (4*3));//锟剿诧拷锟斤拷为锟斤�?
	// TIM1->CCER |= 1 << 4;//CC2使锟斤拷
	// TIM1->CCER |= 1 << 5;//锟斤拷路锟斤拷锟铰斤拷锟斤拷锟斤拷锟�?��拷锟斤拷锟斤拷锟斤拷
	// TIM1->CCER |= 1 << 7;	
	//SMCR锟侥达拷锟斤拷锟斤拷锟斤�?
	TIM1->SMCR &= ~(5 << 1);
	TIM1->SMCR |= 4 << 0;//锟斤拷锟斤拷为锟斤拷位模�?
	TIM1->SMCR &= ~(7 << 4);
	TIM1->SMCR |= 5 << 4;//锟剿诧拷锟斤拷亩锟绞憋拷锟斤拷锟斤拷锟絋I1FP1
	TIM1->SMCR |= 1 << 7;//锟斤拷锟斤拷为锟斤拷模式
	
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM1,TIM_IT_CC1,ENABLE);
	
	//使锟斤拷锟叫讹拷
	TIM1->DIER |= 1 << 1;//使锟斤拷CC1锟叫讹拷
	TIM1->CR1 |= 1 << 0;
}

void Receiver_Init(void)
{
	TIM1_Init();
}

uint32_t Receiver_CalcDutyCycle(uint8_t i)
{
	return 1000+(ppm_val[i]-PPM_MIN_VAL)*1000/(PPM_MAX_VAL-PPM_MIN_VAL);
}

void TIM1_CC_IRQHandler(void)
{
	static uint8_t valid_cnt = 0; // represent if the ppm signal is valid

	if(TIM_GetITStatus(TIM1,TIM_IT_CC1) == SET)
	{
		uint32_t val = TIM_GetCapture1(TIM1);
		if(val > PPM_MAX_VAL)
		{
			if(val > 2000 && val <= 12000) // ppm signal start symbol
			{
				ppm_started = 1;
				ppm_error = 0;
			}
			else
			{
				ppm_error = 1;
				signal_blocked = 1;
			}
		} 
		if(ppm_started == 1)
		{
			ppm_val[ppm_cnt] = val;
			if(ppm_cnt!=0 && ppm_val[ppm_cnt] <= (PPM_MAX_VAL) && ppm_val[ppm_cnt] >= (PPM_MIN_VAL))
			{
				valid_cnt ++;
			}
			ppm_cnt++;
			if(ppm_cnt > 8)
			{
				ppm_cnt = 0;
				ppm_started = 0;
				if(valid_cnt == 8)
				{
					SignalBlockDetect();
					if(signal_blocked == 0)
					{
						MotorArmDetect();
						// ESCUnlockfDetect(); // no need to execute this :)
					}
				}
				valid_cnt = 0;
			}
		}

		TIM_ClearITPendingBit(TIM1,TIM_IT_CC1);
	}
}

