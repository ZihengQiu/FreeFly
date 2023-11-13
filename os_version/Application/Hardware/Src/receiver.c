#include "receiver.h"
#include "motor.h"
#include "bluetooth.h"
#include "os_cpu.h"
#include "ucos_ii.h"
#include <sys/_stdint.h>

uint32_t PulseWidth, Period, DutyCycle;
uint32_t ppm_val[10], ppm_cnt = 0;
BOOLEAN ppm_started;

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
	GPIOA->MODER |= 2 << (2*8);//���ù���
	GPIOA->OTYPER &=~ (1 << 8);//�����������
	GPIOA->OSPEEDR |= 2 << (2*8);//50Mhz
	GPIOA->PUPDR &= ~(3 << (2*8));//no pull and down
	// ʱ����Ԫ
	TIM1->ARR = 0;
	TIM1->CR1 &=~ (1 << 4);//��������
	TIM1->PSC |= 83;
	TIM1->ARR |= 19999;
	TIM1->EGR |= 1 << 1;//����EGR�Ĵ�����CC1Gλ��ʹ�ò��񵽱����źź�Ͳ���һ�������¼�
	TIM1->CR1 |= 1 << 7;//ARRʹ��
	// ����ͨ��1
	TIM1->CCMR1 |= 1 << (2*0);//ͨ��1�Ĳ����ź�IC1��ӳ�䵽������TI1��
	TIM1->CCMR1 &=~ (3 << (2*1));//���Ա����źŽ��з�Ƶ����
	TIM1->CCMR1 &=~ (15 << (4*1));//�˲���Ϊ��
	TIM1->CCER |= 1 << 0;//CC1ʹ��
	TIM1->CCER &=~ (1 << 1);//��·�����������У�������
	TIM1->CCER &=~ (1 << 3);
	//����ͨ��2
	// TIM1->CCMR1 |= 2 << (2*4);//ͨ��2�Ĳ����ź�IC2��ӳ�䵽������TI1�� 
	// TIM1->CCMR1 &=~ (3 << (2*5));//���Ա����źŽ��з�Ƶ����
	// TIM1->CCMR1 &=~ (15 << (4*3));//�˲���Ϊ��
	// TIM1->CCER |= 1 << 4;//CC2ʹ��
	// TIM1->CCER |= 1 << 5;//��·���½������У�������
	// TIM1->CCER |= 1 << 7;	
	//SMCR�Ĵ�������
	TIM1->SMCR &= ~(5 << 1);
	TIM1->SMCR |= 4 << 0;//����Ϊ��λģʽ
	TIM1->SMCR &= ~(7 << 4);
	TIM1->SMCR |= 5 << 4;//�˲���Ķ�ʱ������TI1FP1
	TIM1->SMCR |= 1 << 7;//����Ϊ��ģʽ
	
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM1,TIM_IT_CC1,ENABLE);
	
	//ʹ���ж�
	TIM1->DIER |= 1 << 1;//ʹ��CC1�ж�
	TIM1->CR1 |= 1 << 0;
}

void Receiver_Init(void)
{
	TIM1_Init();
}

//void TIM1_CC_IRQHandler(void)//TIM1_GetDutyCycle PWM version
//{
//	if(((TIM1->SR & 0x2) == 2)&&((TIM1->SR & 0x4) == 4))//����Ƿ�׽�������غ��½��� 
//		{
//			PulseWidth = TIM1->CCR2;
//			Period = TIM1->CCR1;
//			TIM1->CNT = 0;
//			//DutyCycle = PulseWidth*10/19;
//    }
//    TIM1->SR &=~ (1 << 1);
//	DutyCycle = PulseWidth*10/19;
//	//if(ret-DutyCycle > 3 || ret-DutyCycle < -3) return DutyCycle;
//	if(DutyCycle < 10) DutyCycle = 10;
//		else if(DutyCycle > 20) DutyCycle = 20;
//}

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
		if(val >= 4000) // ppm signal start
		{
			ppm_started = 1;
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
					MotorArmDetect();
					ESCUnlockDetect();
				}
				valid_cnt = 0;
			}
		}

		TIM_ClearITPendingBit(TIM1,TIM_IT_CC1);
	}
}

