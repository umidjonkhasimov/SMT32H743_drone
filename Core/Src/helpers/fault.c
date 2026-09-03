#include "helpers/fault.h"

__attribute__((section(".dma_buffer")))
volatile FaultInfo_t g_fault;

void Fault_Capture(uint32_t source)
{
    /* These are memory-mapped registers latched by the core at fault time,
       so they are valid regardless of what the handler prologue did to the
       stack. This is the reliable part of the capture. */
    g_fault.cfsr  = SCB->CFSR;
    g_fault.hfsr  = SCB->HFSR;
    g_fault.bfar  = SCB->BFAR;
    g_fault.mmfar = SCB->MMFAR;
    g_fault.abfsr = SCB->ABFSR;
    g_fault.icsr  = SCB->ICSR;

    g_fault.msp = __get_MSP();
    g_fault.psp = __get_PSP();

    const uint32_t *sp = (const uint32_t *)g_fault.msp;

    for (int i = 0; i < 20; i++)
    {
        g_fault.stack[i] = sp[i];
    }

    g_fault.source = source;
    g_fault.magic  = FAULT_MAGIC;
}
