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
	GPIOA->MODER |= 2 << (2*8);//éæ¤æ·éç?¸æ·éæ¤æ·
	GPIOA->OTYPER &=~ (1 << 8);//éæ¤æ·éæ¤æ·éæ¤æ·éæ¤æ·éæ¤æ·éï¿?
	GPIOA->OSPEEDR |= 2 << (2*8);//50Mhz
	GPIOA->PUPDR &= ~(3 << (2*8));//no pull and down
	// æ¶éæ¤æ·éæ¤æ·å
	TIM1->ARR = 0;
	TIM1->CR1 &=~ (1 << 4);//éæ¤æ·éæ¤æ·éæ¤æ·éæ¤æ·
	TIM1->PSC |= 83;
	TIM1->ARR |= 19999;
	TIM1->EGR |= 1 << 1;//éæ¤æ·éæ¤æ·EGRéä¾¥è¾¾æ·éæ¤æ·éæ¤æ·CC1Gä½éæ¤æ·ä½¿éç??æ·éä»å°ææ·éæ¤æ·éèå·ç´æ·ç­éæ¤æ·éæ­ä¼æ·éæ¤æ·éæ¤æ·éæ¤æ·å½éï¿½
	TIM1->CR1 |= 1 << 7;//ARRä½¿éæ¤æ·
	// éæ¤æ·éæ¤æ·ééæ¤æ·1
	TIM1->CCMR1 |= 1 << (2*0);//ééæ¤æ·1éä¾¥è¯§æ·éæ¤æ·éèç´æ·IC1éæ¤æ·æ éæå°éæ¤æ·éæ¤æ·éæ¤æ·TI1éæ¤æ?
	TIM1->CCMR1 &=~ (3 << (2*1));//éæ¤æ·éçææ·éæ¤æ·éèå·æ¤æ·éå?¤æ·é?éæ¤æ·éæ¤æ·
	TIM1->CCMR1 &=~ (15 << (4*1));//éå¿è¯§æ·éæ¤æ·ä¸ºéæ¤æ?
	TIM1->CCER |= 1 << 0;//CC1ä½¿éæ¤æ·
	TIM1->CCER &=~ (1 << 1);//éæ¤æ·è·¯éæ¤æ·éæ¤æ·éæ¤æ·éæ¤æ·éæ¤æ·éå?½æ·éæ¤æ·éæ¤æ·éæ¤æ·
	TIM1->CCER &=~ (1 << 3);
	//éæ¤æ·éæ¤æ·ééæ¤æ·2
	// TIM1->CCMR1 |= 2 << (2*4);//ééæ¤æ·2éä¾¥è¯§æ·éæ¤æ·éèç´æ·IC2éæ¤æ·æ éæå°éæ¤æ·éæ¤æ·éæ¤æ·TI1éæ¤æ? 
	// TIM1->CCMR1 &=~ (3 << (2*5));//éæ¤æ·éçææ·éæ¤æ·éèå·æ¤æ·éå?¤æ·é?éæ¤æ·éæ¤æ·
	// TIM1->CCMR1 &=~ (15 << (4*3));//éå¿è¯§æ·éæ¤æ·ä¸ºéæ¤æ?
	// TIM1->CCER |= 1 << 4;//CC2ä½¿éæ¤æ·
	// TIM1->CCER |= 1 << 5;//éæ¤æ·è·¯éæ¤æ·éé°æ¤æ·éæ¤æ·éæ¤æ·éå?½æ·éæ¤æ·éæ¤æ·éæ¤æ·
	// TIM1->CCER |= 1 << 7;	
	//SMCRéä¾¥è¾¾æ·éæ¤æ·éæ¤æ·éæ¤æ?
	TIM1->SMCR &= ~(5 << 1);
	TIM1->SMCR |= 4 << 0;//éæ¤æ·éæ¤æ·ä¸ºéæ¤æ·ä½æ¨¡å¼?
	TIM1->SMCR &= ~(7 << 4);
	TIM1->SMCR |= 5 << 4;//éå¿è¯§æ·éæ¤æ·äº©éç»ææ·éæ¤æ·éæ¤æ·éçµI1FP1
	TIM1->SMCR |= 1 << 7;//éæ¤æ·éæ¤æ·ä¸ºéæ¤æ·æ¨¡å¼
	
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM1,TIM_IT_CC1,ENABLE);
	
	//ä½¿éæ¤æ·éå«è®¹æ·
	TIM1->DIER |= 1 << 1;//ä½¿éæ¤æ·CC1éå«è®¹æ·
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

