#include <includes.h>


void OSInitHookBegin(void)
{}
void OSInitHookEnd(void)
{}
void OSTCBInitHook(OS_TCB *ptcb)
{}
void OSTaskCreateHook(OS_TCB *ptcb)
{}
void OSTaskIdleHook(void)
{}
void OSTaskStatHook(void)
{}
void OSTimeTickHook(void)
{}
void OSTaskDelHook(OS_TCB *ptcb)
{}
void OSTaskReturnHook(OS_TCB *ptcb)
{}
OS_STK *OSTaskStkInit(void (*task)(void *p_arg), void *p_arg,OS_STK *ptos,INT16U opt)
{
	OS_STK *stk;
	opt = opt;
	stk = (OS_STK *)((OS_STK)(ptos)&0xFFFFFFF8u); // to ensure 2 words alignment of AAPCS
	
	*(--stk) = (OS_STK)0x01000000uL; 	// xPSR
	*(--stk) = (OS_STK)task;			// PC
	*(--stk) = (OS_STK)0xFFFFFFFEL;		// LR
	*(--stk) = (OS_STK)0x12;			// R12
	*(--stk) = (OS_STK)0x3;				// R3
	*(--stk) = (OS_STK)0x2;				// R2
	*(--stk) = (OS_STK)0x1;				// R1
	*(--stk) = (OS_STK)p_arg;			// R0
	*(--stk) = (OS_STK)0xFFFFFFFD;		// EXEC_RETURN
	*(--stk) = (OS_STK)0x11;			// R11
	*(--stk) = (OS_STK)0x10;			// R10
	*(--stk) = (OS_STK)0x9;				// R9
	*(--stk) = (OS_STK)0x8;				// R8
	*(--stk) = (OS_STK)0x7;				// R7
	*(--stk) = (OS_STK)0x6;				// R6
	*(--stk) = (OS_STK)0x5;				// R5
	*(--stk) = (OS_STK)0x4;				// R4
	
	return stk;
}
extern INT32U times;
// void SysTick_Handler(void)
// {
// 	#if OS_CRITICAL_METHOD == 3u
// 		OS_CPU_SR  cpu_sr = 0u;
// 	#endif
// 	OS_ENTER_CRITICAL();
// 	OSIntEnter();
// 	OS_EXIT_CRITICAL();
// 	OSTimeTick();
// 	OSIntExit();

// }
