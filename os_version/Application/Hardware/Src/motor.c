#include "stm32f4xx.h"                  // Device header
#include "motor.h"

void Motor_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;   //声明一个结构体变量，用来初始化GPIO

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//声明一个结构体变量，用来初始化定时器

	TIM_OCInitTypeDef TIM_OCInitStructure;//根据TIM_OCInitStruct中指定的参数初始化外设TIMx

	/* 开启时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	/*  配置GPIO的模式和IO口 */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;// PC6
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

void Motor_SetDutyCycle(uint32_t Compare1)
{
	TIM3->CCR1 = Compare1;
}

