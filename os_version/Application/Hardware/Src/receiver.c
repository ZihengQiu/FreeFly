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
	GPIOA->MODER |= 2 << (2*8);//閿熸枻鎷烽敓鐭櫢鎷烽敓鏂ゆ嫹
	GPIOA->OTYPER &=~ (1 << 8);//閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓锟�
	GPIOA->OSPEEDR |= 2 << (2*8);//50Mhz
	GPIOA->PUPDR &= ~(3 << (2*8));//no pull and down
	// 鏃堕敓鏂ゆ嫹閿熸枻鎷峰厓
	TIM1->ARR = 0;
	TIM1->CR1 &=~ (1 << 4);//閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
	TIM1->PSC |= 83;
	TIM1->ARR |= 19999;
	TIM1->EGR |= 1 << 1;//閿熸枻鎷烽敓鏂ゆ嫹EGR閿熶茎杈炬嫹閿熸枻鎷烽敓鏂ゆ嫹CC1G浣嶉敓鏂ゆ嫹浣块敓鐭鎷烽敓浠婂埌鎲嬫嫹閿熸枻鎷烽敓鑴氬彿鐚存嫹绛掗敓鏂ゆ嫹閿熸彮浼欐嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷峰綍閿燂拷
	TIM1->CR1 |= 1 << 7;//ARR浣块敓鏂ゆ嫹
	// 閿熸枻鎷烽敓鏂ゆ嫹閫氶敓鏂ゆ嫹1
	TIM1->CCMR1 |= 1 << (2*0);//閫氶敓鏂ゆ嫹1閿熶茎璇ф嫹閿熸枻鎷烽敓鑴氱尨鎷稩C1閿熸枻鎷锋槧閿熸垝鍒伴敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹TI1閿熸枻鎷�
	TIM1->CCMR1 &=~ (3 << (2*1));//閿熸枻鎷烽敓鐨嗘唻鎷烽敓鏂ゆ嫹閿熻剼鍙锋枻鎷烽敓鍙嚖鎷烽閿熸枻鎷烽敓鏂ゆ嫹
	TIM1->CCMR1 &=~ (15 << (4*1));//閿熷壙璇ф嫹閿熸枻鎷蜂负閿熸枻鎷�
	TIM1->CCER |= 1 << 0;//CC1浣块敓鏂ゆ嫹
	TIM1->CCER &=~ (1 << 1);//閿熸枻鎷疯矾閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鍙綇鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
	TIM1->CCER &=~ (1 << 3);
	//閿熸枻鎷烽敓鏂ゆ嫹閫氶敓鏂ゆ嫹2
	// TIM1->CCMR1 |= 2 << (2*4);//閫氶敓鏂ゆ嫹2閿熶茎璇ф嫹閿熸枻鎷烽敓鑴氱尨鎷稩C2閿熸枻鎷锋槧閿熸垝鍒伴敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹TI1閿熸枻鎷� 
	// TIM1->CCMR1 &=~ (3 << (2*5));//閿熸枻鎷烽敓鐨嗘唻鎷烽敓鏂ゆ嫹閿熻剼鍙锋枻鎷烽敓鍙嚖鎷烽閿熸枻鎷烽敓鏂ゆ嫹
	// TIM1->CCMR1 &=~ (15 << (4*3));//閿熷壙璇ф嫹閿熸枻鎷蜂负閿熸枻鎷�
	// TIM1->CCER |= 1 << 4;//CC2浣块敓鏂ゆ嫹
	// TIM1->CCER |= 1 << 5;//閿熸枻鎷疯矾閿熸枻鎷烽敓閾版枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鍙綇鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
	// TIM1->CCER |= 1 << 7;	
	//SMCR閿熶茎杈炬嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷�
	TIM1->SMCR &= ~(5 << 1);
	TIM1->SMCR |= 4 << 0;//閿熸枻鎷烽敓鏂ゆ嫹涓洪敓鏂ゆ嫹浣嶆ā寮�
	TIM1->SMCR &= ~(7 << 4);
	TIM1->SMCR |= 5 << 4;//閿熷壙璇ф嫹閿熸枻鎷蜂憨閿熺粸鎲嬫嫹閿熸枻鎷烽敓鏂ゆ嫹閿熺祴I1FP1
	TIM1->SMCR |= 1 << 7;//閿熸枻鎷烽敓鏂ゆ嫹涓洪敓鏂ゆ嫹妯″紡
	
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM1,TIM_IT_CC1,ENABLE);
	
	//浣块敓鏂ゆ嫹閿熷彨璁规嫹
	TIM1->DIER |= 1 << 1;//浣块敓鏂ゆ嫹CC1閿熷彨璁规嫹
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
	static uint8_t valid_cnt = 0; // check if the ppm signal is valid

	if(TIM_GetITStatus(TIM1,TIM_IT_CC1) == SET)
	{
		uint32_t val = TIM_GetCapture1(TIM1);
		if(val > PPM_MAX_VAL)
		{
			if(val > 2000 && val <= 12000) // ppm signal start
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
						// ESCUnlockfDetect();
					}
				}
				valid_cnt = 0;
			}
		}

		TIM_ClearITPendingBit(TIM1,TIM_IT_CC1);
	}
}

