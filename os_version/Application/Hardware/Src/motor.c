#include "stm32f4xx_gpio.h"

#include <stdio.h>
#include <string.h>
#include <sys/_stdint.h>

#include "ucos_ii.h"

#include "bluetooth.h"
#include "motor.h"
#include "receiver.h"

void Motor_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;   //声明一个结构体变量，用来初始化GPIO

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//声明一个结构体变量，用来初始化定时器

	TIM_OCInitTypeDef TIM_OCInitStructure;//根据TIM_OCInitStruct中指定的参数初始化外设TIMx

	/* 开启时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	/*  配置GPIO的模式和IO口 */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 |GPIO_Pin_9;// PC6 7 8 9
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;//复用推挽输出
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIOC->AFR[0] |= 2 << 24; 
	
	//TIM3定时器初始化
	//PWM 频率 50Hz = 84 000 000/(83+1)/(19999+1)
	TIM_TimeBaseInitStructure.TIM_Period = 19999; //自动重装载寄存器周期ARR 
	TIM_TimeBaseInitStructure.TIM_Prescaler = 83;//TIM3计数器时钟频率预分频值PSC
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_TimeBaseInit(TIM3, & TIM_TimeBaseInitStructure);

	//PWM初始化	  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//PWM输出使能
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;

	TIM_OC1Init(TIM3,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);//使能TIMx在CCR1上的预装载寄存器
	TIM_Cmd(TIM3,ENABLE);//使能TIMx外设
	TIM3->CR1|=(1<<7);
}

BOOLEAN ESC_Unlock(void)
{
	memset(ppm_val, 0, sizeof(ppm_val));
	
	uint32_t fails_cnt = 0;
	while(1)
	{
		if(fails_cnt > 1000)
		{
			// return 0;
		}
 
		BOOLEAN hold_flag = 1;
		Bluetooth_SendString("waiting for 2000\n");
		while(ppm_val[THR] <= PPM_MAX_VAL)
		{
			printf("%d\n", ppm_val[THR]);
		}

		Bluetooth_SendString("Reach 2000\n");

		for(uint16_t i=0; i<300; i++)
		{
			if(ppm_val[THR] <= PPM_MAX_VAL)
			{
				hold_flag = 0;
				break;
			}		
			OSTimeDly(10);	// delay 3S in total
			if(i % 100 == 0)
				printf("Still for %d\n", i);
		}
		if(hold_flag == 0)
		{
			fails_cnt++;
			continue;
		}

		Bluetooth_SendString("2000 verified!\n");

		while(ppm_val[THR] >= PPM_MIN_VAL)
		{
			printf("%d\n", ppm_val[THR]);
		}

		Bluetooth_SendString("Reach 1000\n");

		for(uint16_t i=0; i<300; i++)
		{
			if(ppm_val[THR] >= PPM_MIN_VAL)
			{
				hold_flag = 0;
				break;
			}		
			OSTimeDly(10);	// delay 3S in total
		}
		if(hold_flag == 0)
		{
			fails_cnt++;
			continue;
		}
		
		Bluetooth_SendString("1000 verified!\n");

		break;
	}
	TIM_SetCompare1(TIM3, 2000);
	OSTimeDly(4000);
	TIM_SetCompare1(TIM3, 1000);
	OSTimeDly(4000);  
	
	return 1;
}

void Motor_SetDutyCycle(uint32_t Compare1)
{
	TIM3->CCR1 = Compare1;
}

