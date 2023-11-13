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

//NVIC_PreemptionPriority:抢占优先级
//NVIC_SubPriority：响应优先级
//NVIC_Channel：中断编号
//NVIC_Group：中断分组
void MY_NVIC_Init(uint8_t NVIC_PreemptionPriority,uint8_t NVIC_SubPriority,uint8_t NVIC_Channel,uint8_t NVIC_Group)
{
	uint32_t temp;
	MY_NVIC_PriorityGroupConfig(NVIC_Group);//设置分组
	temp = NVIC_PreemptionPriority<<(4 - NVIC_Group);
	temp |= NVIC_SubPriority&(0X0f>>NVIC_Group);
	temp &= 0Xf;                                    //取低四位
	NVIC->ISER[NVIC_Channel/32] |= 1 << NVIC_Channel % 32;//使能中断位
	NVIC->IP[NVIC_Channel] |= temp << 4;//设置响应优先级和抢占优先级
}

void TIM1_Init(void)
{
	// PA8 : TIM1CH1
	RCC->AHB1ENR |= 1 << 0;
	RCC->APB2ENR |= 1 << 0;
	GPIOA->AFR[1] |= 1 << 0;
	GPIOA->MODER &= ~(3 << (2*8));
	GPIOA->MODER |= 2 << (2*8);//复用功能
	GPIOA->OTYPER &=~ (1 << 8);//复用推挽输出
	GPIOA->OSPEEDR |= 2 << (2*8);//50Mhz
	GPIOA->PUPDR &= ~(3 << (2*8));//no pull and down
	// 时机单元
	TIM1->ARR = 0;
	TIM1->CR1 &=~ (1 << 4);//递增计数
	TIM1->PSC |= 83;
	TIM1->ARR |= 19999;
	TIM1->EGR |= 1 << 1;//配置EGR寄存器的CC1G位，使得捕获到边沿信号后就产生一个捕获事件
	TIM1->CR1 |= 1 << 7;//ARR使能
	// 输入通道1
	TIM1->CCMR1 |= 1 << (2*0);//通道1的捕获信号IC1被映射到了引脚TI1上
	TIM1->CCMR1 &=~ (3 << (2*1));//不对边沿信号进行分频处理
	TIM1->CCMR1 &=~ (15 << (4*1));//滤波器为零
	TIM1->CCER |= 1 << 0;//CC1使能
	TIM1->CCER &=~ (1 << 1);//电路对上升沿敏感（即捕获）
	TIM1->CCER &=~ (1 << 3);
	//输入通道2
	// TIM1->CCMR1 |= 2 << (2*4);//通道2的捕获信号IC2被映射到了引脚TI1上 
	// TIM1->CCMR1 &=~ (3 << (2*5));//不对边沿信号进行分频处理
	// TIM1->CCMR1 &=~ (15 << (4*3));//滤波器为零
	// TIM1->CCER |= 1 << 4;//CC2使能
	// TIM1->CCER |= 1 << 5;//电路对下降沿敏感（即捕获）
	// TIM1->CCER |= 1 << 7;	
	//SMCR寄存器设置
	TIM1->SMCR &= ~(5 << 1);
	TIM1->SMCR |= 4 << 0;//设置为复位模式
	TIM1->SMCR &= ~(7 << 4);
	TIM1->SMCR |= 5 << 4;//滤波后的定时器输入TI1FP1
	TIM1->SMCR |= 1 << 7;//设置为从模式
	
	
	TIM_ITConfig(TIM1,TIM_IT_CC1,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);
	
//	NVIC->IP[TIM1_CC_IRQn] = 0 << 4;
//	NVIC->IP[TIM1_UP_TIM10_IRQn] = 1 << 4;
//	NVIC->ISER[TIM1_CC_IRQn/32] |= 1 << TIM1_CC_IRQn % 32;
//	NVIC->ISER[TIM1_UP_TIM10_IRQn/32] |= 1 << (uint32_t)(TIM1_UP_TIM10_IRQn % 32);
	
	//使能中断
	TIM1->DIER |= 1 << 1;//使能CC1中断
	TIM1->CR1 |= 1 << 0;
}

void Receiver_Init(void)
{
	TIM1_Init();
}

//void TIM1_CC_IRQHandler(void)//TIM1_GetDutyCycle PWM version
//{
//	if(((TIM1->SR & 0x2) == 2)&&((TIM1->SR & 0x4) == 4))//检测是否捕捉到上升沿和下降沿 
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
		if(val >= 4000)
		{
			ppm_started = 1;
		} 
		if(ppm_started == 1)
		{
			ppm_val[ppm_cnt] = TIM_GetCapture1(TIM1);
			if(ppm_cnt!=0 && ppm_val[ppm_cnt] <= (PPM_MAX_VAL+10) && ppm_val[ppm_cnt] >= (PPM_MIN_VAL-10))
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

