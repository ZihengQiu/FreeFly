#include "os_cpu.h"
#include "stm32f4xx_gpio.h"

#include <stdio.h>
#include <string.h>
#include <sys/_stdint.h>

#include "ucos_ii.h"

#include "bluetooth.h"
#include "motor.h"
#include "receiver.h"


BOOLEAN motor_armed = 0,	// armed : motor can be controlled by the remote controller
		signal_blocked = 1, // signal blocked : cut off board's pwm signal to motor, used in emergency
		ESC_unlock_need_execute = 0,
		ESC_unlock_executed = 0;

OS_TMR  *tmr_arm, *tmr_disarm; // timer for arm and disarm detection

uint32_t motor_compare[4]; // compare value of each motor to be set

void Motor_GPIO_Init(void)
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
	GPIOC->AFR[0] |= 2 << 28;
	GPIOC->AFR[1] |= 2 << 0;
	GPIOC->AFR[1] |= 2 << 4;
	
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
	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	TIM_OC3Init(TIM3,&TIM_OCInitStructure);
	TIM_OC4Init(TIM3,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);//使能TIMx在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);//使能TIMx在CCR2上的预装载寄存器
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);//使能TIMx在CCR3上的预装载寄存器
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);//使能TIMx在CCR4上的预装载寄存器
	
	TIM3->CCR1 = MOTOR_COMPARE_MIN_VAL; // warning:CCR's default value 0 can make motor spin
	TIM3->CCR2 = MOTOR_COMPARE_MIN_VAL;
	TIM3->CCR3 = MOTOR_COMPARE_MIN_VAL;
	TIM3->CCR4 = MOTOR_COMPARE_MIN_VAL;

	TIM_Cmd(TIM3,ENABLE);//使能TIMx外设
	TIM3->CR1|=(1<<7);
}

OS_TMR_CALLBACK ArmTmrCallback(OS_TMR *ptmr, void *parg)
{
	motor_armed = 1;
	Bluetooth_SendString("motor armed!\r\n");
	return NULL;
}

OS_TMR_CALLBACK DisarmTmrCallback(OS_TMR *ptmr, void *parg)
{
	motor_armed = 0;
	Bluetooth_SendString("motor disarmed!\r\n");
	return NULL;
}

void Motor_Tmr_Init(void)
{
	uint8_t err1, err2;
	tmr_arm = OSTmrCreate(5, 0, OS_TMR_OPT_ONE_SHOT,
				(OS_TMR_CALLBACK)ArmTmrCallback,0 , (INT8U *)"ARM TIMER", &err1);
	tmr_disarm = OSTmrCreate(5, 0, OS_TMR_OPT_ONE_SHOT,
				(OS_TMR_CALLBACK)DisarmTmrCallback,0 , (INT8U *)"DISARM TIMER", &err2);
}

void Motor_Init(void)
{
	Motor_GPIO_Init();
	Motor_Tmr_Init();
}

void SignalBlockDetect(void)
{
	if(ppm_val[7] >= PPM_MAX_VAL-15)
	{
		signal_blocked = 1;
	}
	else
	{
		signal_blocked = 0;
	}
}

void MotorArmDetect(void)
{
	// arm detect : throttle minimum, yaw maximum for 1 seconds
	if(ppm_val[THR] < PPM_MIN_VAL+15 && ppm_val[RUD] > PPM_MAX_VAL-15)
	{
		if(OSTmrStateGet(tmr_arm, 0)!=3)
		{
			OSTmrStart(tmr_arm, 0);
		}
	}
	else
	{
		OSTmrStop(tmr_arm, OS_TMR_OPT_NONE, NULL, OS_ERR_NONE);
	}

	// disarm detect : throttle minimum, yaw minimum for 1 seconds
	if(ppm_val[THR] < PPM_MIN_VAL+15 && ppm_val[RUD] < PPM_MIN_VAL+15)
	{
		if(OSTmrStateGet(tmr_disarm, 0) != 3)
		{
			OSTmrStart(tmr_disarm, 0);
		}
	}
	else 
	{
		OSTmrStop(tmr_disarm, OS_TMR_OPT_NONE, NULL, OS_ERR_NONE);
	}
}


BOOLEAN ESCUnlock(void)	// ret 0 : unlock failed, ret 1 : unlock success
{
	Bluetooth_SendString("ESC will be unlocked in 3s...\r\n");
	Bluetooth_SendString("Turn CH5or7 to High to stop unlock.\r\n");

	OSTimeDly(2000);
	if(ppm_val[5] > PPM_MIN_VAL || ppm_val[6] > PPM_MIN_VAL)
	{
		Bluetooth_SendString("ESC unlocked procedure stopped.\r\n");
		return 0;
	}

	Bluetooth_SendString("ESC unlock procedure starts!\r\n");

	motor_compare[0] = MOTOR_COMPARE_MAX_VAL;
	motor_compare[1] = MOTOR_COMPARE_MAX_VAL;
	motor_compare[2] = MOTOR_COMPARE_MAX_VAL;
	motor_compare[3] = MOTOR_COMPARE_MAX_VAL;
	MotorSetSpeed();

	OSTimeDly(4000);

	motor_compare[0] = MOTOR_COMPARE_MIN_VAL;
	motor_compare[1] = MOTOR_COMPARE_MIN_VAL;
	motor_compare[2] = MOTOR_COMPARE_MIN_VAL;
	motor_compare[3] = MOTOR_COMPARE_MIN_VAL;
	MotorSetSpeed();

	TIM_SetCompare1(TIM3, MOTOR_COMPARE_MIN_VAL);
	OSTimeDly(4000);

	return 1;
}

void ESCUnlockDetect(void)
{
	if(ppm_val[5] < PPM_MIN_VAL && ppm_val[6] < PPM_MIN_VAL && ESC_unlock_executed == 0)
	{
		ESC_unlock_need_execute = 1;
	}
}

uint32_t MotorSpeedLimit(uint32_t compare)
{
	if(compare > MOTOR_COMPARE_MAX_VAL)
	{
		return MOTOR_COMPARE_MAX_VAL;
	}
	else if(compare < MOTOR_COMPARE_MIN_VAL)
	{
		return MOTOR_COMPARE_MIN_VAL;
	}
	else
	{
		return compare;
	}
}

void MotorSetSpeed(void)
{
	TIM3->CCR3 = MotorSpeedLimit(motor_compare[0]); // PB8
	TIM3->CCR1 = MotorSpeedLimit(motor_compare[1]); // PB6
	TIM3->CCR2 = MotorSpeedLimit(motor_compare[2]); // PB7
	TIM3->CCR4 = MotorSpeedLimit(motor_compare[3]);	// PB9
}
