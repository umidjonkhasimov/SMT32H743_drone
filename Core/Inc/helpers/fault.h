#ifndef FAULT_H
#define FAULT_H

#include "main.h"
#include <stdint.h>

typedef struct
{
    uint32_t magic;     /* FAULT_MAGIC once something has actually faulted */
    uint32_t source;    /* 1 = HardFault, 2 = MemManage, 3 = Bus, 4 = Usage */

    uint32_t cfsr;      /* Configurable Fault Status - says what kind */
    uint32_t hfsr;      /* HardFault Status - bit 30 = escalated, see cfsr */
    uint32_t bfar;      /* address of a bus fault, if BFARVALID */
    uint32_t mmfar;     /* address of a memory-management fault */
    uint32_t abfsr;     /* Cortex-M7 auxiliary bus fault (AXI/TCM/EPPB) */
    uint32_t icsr;

    uint32_t msp;
    uint32_t psp;

    /* Raw top of stack. The 8-word exception frame - R0 R1 R2 R3 R12 LR PC
       xPSR - is in here somewhere, past whatever the handler prologue
       pushed. xPSR is the recognisable one: it has bit 24 set. */
    uint32_t stack[20];
} FaultInfo_t;

/* Lives in the no-init DMA section, so it survives a reset and can be read
   back after the board has been power-cycled and reflashed. */
extern volatile FaultInfo_t g_fault;

#define FAULT_MAGIC 0xFA017EDU

void Fault_Capture(uint32_t source);

#endif /* FAULT_H */
