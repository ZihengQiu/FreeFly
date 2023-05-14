#ifndef __OS_CPU_H
#define __OS_CPU_H

#include <ucos_ii.h>

/*
******************************************************************************
*                                  DATA TYPES
*                             (Compiler Specific)
******************************************************************************
*/
  
typedef unsigned char  BOOLEAN;                                               
typedef unsigned char  INT8U;       /* Unsigned  8 bit quantity           */
typedef signed   char  INT8S;       /* Signed    8 bit quantity           */
typedef unsigned short INT16U;      /* Unsigned 16 bit quantity           */
typedef signed   short INT16S;      /* Signed   16 bit quantity           */
typedef unsigned int   INT32U;      /* Unsigned 32 bit quantity           */
typedef signed   int   INT32S;      /* Signed   32 bit quantity           */
typedef float          FP32;        /* Single precision floating point    */  
typedef double         FP64;        /* Double precision floating point    */
  
typedef unsigned int   OS_STK;      /* Each stack entry is 32-bit wide    */  
typedef unsigned int OS_CPU_SR;   /* Define size of CPU status register ;save cpsr*/ 
  
/*
******************************************************************************
*                             Processor Specifics
******************************************************************************
*/
#define  OS_CRITICAL_METHOD   3                                              
  
#if      OS_CRITICAL_METHOD == 1
#define  OS_ENTER_CRITICAL()  ????                                            
#define  OS_EXIT_CRITICAL()   ????
#endif
  
#if      OS_CRITICAL_METHOD == 2
#define  OS_ENTER_CRITICAL()  ????                                            
#define  OS_EXIT_CRITICAL()   ????
#endif
  
#if      OS_CRITICAL_METHOD == 3

extern OS_CPU_SR OS_SaveSR(void);
extern void OS_RestoreSR(OS_CPU_SR cpu_sr);

#define  OS_ENTER_CRITICAL()  do{cpu_sr = OS_SaveSR();}	while(0)                                   
#define  OS_EXIT_CRITICAL()   do{OS_RestoreSR(cpu_sr);}	while(0)   

#endif
  
  
#define  OS_STK_GROWTH        1		/* Stack growth (0=Up, 1=Down) */    
  
#define  OS_TASK_SW()         OSCtxSw()                                           

#endif
