	EXTERN OSRunning   
	EXTERN OSPrioCur
	EXTERN OSPrioHighRdy
	EXTERN OSTCBHighRdy
	EXTERN OSTCBCur
	EXTERN OSTimeTick
	EXTERN OSIntExit
	EXTERN OSIntEnter
	EXTERN OSIntNesting
	EXTERN SYSVIEW_TaskSwitchedIn

	EXPORT OSStartHighRdy
	EXPORT OSCtxSw
	EXPORT OSIntCtxSw
	EXPORT OS_SaveSR
	EXPORT OS_RestoreSR
	EXPORT SysTick_Handler
	EXPORT PendSV_Handler

SCB_PRI_REG_PENDSV  EQU 	0xE000ED22		; interrupt priority register
NVIC_ICS_REG		EQU 	0xE000ED04		; interrupt control state register

NVIC_PENDSV_PRI		EQU 	0x000000FF		; PendSV priority
NVIC_PENDSV_SETPEND	EQU		0x10000000		; PendSV set to pend status

	AREA |.text|, CODE, READONLY, ALIGN=2
	THUMB
    REQUIRE8
    PRESERVE8

OSStartHighRdy
	CPSID I

	; set PendSV priority to the lowest
	ldr r0, =SCB_PRI_REG_PENDSV
	ldr r1, =NVIC_PENDSV_PRI
	str r1, [r0]

	; Set the PSP to 0
	MOVS    R0, #0     
    MSR     PSP, R0

	; OSRunning = TRUE
	ldr r0, =OSRunning
	mov r1, #1
	str r1, [r0]

	; change msp to psp
	mrs r1, CONTROL
	; mov r1,#0
	orr r1, r1, #2
	msr CONTROL, r1
	ISB

	; psp = OSTCBHighRdy->OSTCBStkPtr
	ldr r0, =OSTCBCur
	ldr r0, [r0]
	ldr r0, [r0]	; !!!
	msr psp, r0

	; restore context
	ldmfd sp!, {r4-r11}
	ldmfd sp!, {lr}		; exc_return is useless
	ldmfd sp!, {r0-r3}
	ldmfd sp!, {r12, lr}
	ldmfd sp!, {r1, r0} ; r0(xpsr) is useless

	CPSIE I

	orr r0,r0,#0x01
	BX r0

OSCtxSw
OSIntCtxSw
	ldr r0, =NVIC_ICS_REG
	ldr r1, =NVIC_PENDSV_SETPEND
	ldr r2, [r0]
	orr r2, r1, r2
	str r2, [r0]
	bx lr

PendSV_Handler
	
	mrs r0, psp	; old task's psp
	; save old task's remaining registers
	; lr is automatically updated to EXEC_RETURN
	stmfd r0!, {r4-r11, lr}

	; OSTCBCur->OSTCBStkPtr = psp
	ldr r1, =OSTCBCur
	ldr r2, [r1]
	str r0, [r2]

	mov r4, lr	; save lr, in case of executing OSTaskSwHook

	;BL      OSTaskSwHook 
	
	; Return with Process Stack by setting EXEC_RETURN
	orr lr, r4, #0x04

	; OSPrioCur = OSPrioHighRdy;
	ldr     r0, =OSPrioCur                                      
    ldr     r1, =OSPrioHighRdy
    ldrb    r2, [r1]
    ldrb    r2, [r0]

	; OSTCBCur = OSTCBHighRdy
	ldr r0, =OSTCBHighRdy
	ldr r1, =OSTCBCur
	ldr r0, [r0]
	str r0, [r1]
	
	; SYSVIEW 
	push {r0, r1}
	mov r0, r1
	bl SYSVIEW_TaskSwitchedIn
	pop {r0, r1}
	
	; psp = OSTCBCur->OSTCBStkPtr
	ldr r1, [r0]	; r1 = sp
	
	;restore new task's registers
	ldmfd r1!, {r4-r11, lr}
	msr psp, r1

	bx lr
	

OS_SaveSR
	cpsid i
	mrs r0, BASEPRI	; save previous BASEPRI
	mov r1, #0		; set new BASEPRI
	msr BASEPRI, r1
	dsb
	isb
	cpsie i
	bx lr

OS_RestoreSR
	cpsid i
	msr BASEPRI, r0
	dsb
	isb
	cpsie i
	bx lr
	
SysTick_Handler

	; OS_SaveSR
	cpsid i
	mrs r0, BASEPRI	; save previous BASEPRI
	mov r1, #0		; set new BASEPRI
	msr BASEPRI, r1
	dsb
	isb
	cpsie i
	
	; OSIntEnter
	push {lr}
 	bl OSIntEnter
	pop {lr}

	; OS_RestoreSR
	cpsid i
	msr BASEPRI, r0
	dsb
	isb
	cpsie i

	; if OSIntNesting == 1
	ldr r1, =OSIntNesting
	ldrb r2, [r1]	;OSIntNesting : int8u
	cmp r2, #1
	beq OSTickISR_FromTask
	bne OSTickISR_FromInt

OSTickISR_FromTask
	; save remained registers & save psp
 	mrs r0, psp
 	stmfd r0!, {r4-r11, lr}
 	msr psp, r0
	ldr r2, =OSTCBCur
	ldr r3, [r2]
	str r0, [r3]
 	bl OSTimeTick
	bl OSIntExit
	; restore remained registers
 	mrs r0, psp
 	ldmfd r0!, {r4-r11, lr}
 	msr psp, r0
 	bx lr

OSTickISR_FromInt	
	; save remained registers
	mrs r0, msp
 	stmfd r0!, {r4-r11, lr}
 	msr msp, r0
 	bl OSTimeTick
 	bl OSIntExit
 	; restore remained registers
 	mrs r0, msp
 	ldmfd r0!, {r4-r11, lr}
 	msr msp, r0
 	bx lr

 	ALIGN
 	END

