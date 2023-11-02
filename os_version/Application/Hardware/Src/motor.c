#include "stm32f4xx_gpio.h"

#include <stdio.h>
#include <string.h>
#include <sys/_stdint.h>

#include "ucos_ii.h"

#include "bluetooth.h"
#include "motor.h"
#include "receiver.h"

BOOLEAN motor_armed = 1;

OS_TMR  *tmr_arm, *tmr_disarm;

void Motor_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;   //����һ���ṹ�������������ʼ��GPIO

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//����һ���ṹ�������������ʼ����ʱ��

	TIM_OCInitTypeDef TIM_OCInitStructure;//����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx

	/* ����ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	/*  ����GPIO��ģʽ��IO�� */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 |GPIO_Pin_9;// PC6 7 8 9
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;//�����������
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIOC->AFR[0] |= 2 << 24; 
	
	//TIM3��ʱ����ʼ��
	//PWM Ƶ�� 50Hz = 84 000 000/(83+1)/(19999+1)
	TIM_TimeBaseInitStructure.TIM_Period = 19999; //�Զ���װ�ؼĴ�������ARR 
	TIM_TimeBaseInitStructure.TIM_Prescaler = 83;//TIM3������ʱ��Ƶ��Ԥ��ƵֵPSC
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, & TIM_TimeBaseInitStructure);

	//PWM��ʼ��	  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//PWM���ʹ��
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;

	TIM_OC1Init(TIM3,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);//ʹ��TIMx��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_Cmd(TIM3,ENABLE);//ʹ��TIMx����
	TIM3->CR1|=(1<<7);
}

OS_TMR_CALLBACK ArmTmrCallback(OS_TMR *ptmr, void *p_arg)
{
	motor_armed = 1;
	return NULL;
}

OS_TMR_CALLBACK DisarmTmrCallback(OS_TMR *ptmr, void *p_arg)
{
	motor_armed = 0;
	return NULL;
}

void Motor_Tmr_Init(void)
{
	uint8_t err1, err2;
	tmr_arm = OSTmrCreate(10, 0, OS_TMR_OPT_ONE_SHOT,
				(OS_TMR_CALLBACK)ArmTmrCallback,0 , (INT8U *)"ARM TIMER", &err1);
	tmr_disarm = OSTmrCreate(10, 0, OS_TMR_OPT_ONE_SHOT,
				(OS_TMR_CALLBACK)DisarmTmrCallback,0 , (INT8U *)"DISARM TIMER", &err2);
}

void Motor_Init(void)
{
	Motor_GPIO_Init();
	Motor_Tmr_Init();
}

void MotorArmDetect(void)
{
	// arm detect : throttle minimum, yaw maximum for 1 seconds
	if(ppm_val[THR] < PPM_MIN_VAL && ppm_val[RUD] > PPM_MAX_VAL && OSTmrStateGet(tmr_arm, 0) == 0)
	{
		OSTmrStart(tmr_arm, 0);
		return;
	}

	// disarm detect : throttle minimum, yaw minimum for 1 seconds
	if(ppm_val[THR] < PPM_MIN_VAL && ppm_val[RUD] < PPM_MIN_VAL && OSTmrStateGet(tmr_disarm, 0) == 0)
	{
		OSTmrStart(tmr_disarm, 0);
		return;
	}
}

void Motor_SetDutyCycle(uint32_t Compare1)
{
	TIM3->CCR1 = Compare1;
}

