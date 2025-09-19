#include "sys.h"


void WFI_SET(void)
{
    __asm__ volatile ("wfi");
}

void INTX_DISABLE(void)
{
    __asm__ volatile (
        "cpsid i\n"
        "bx lr\n"
    );
}

void INTX_ENABLE(void)
{
    __asm__ volatile (
        "cpsie i\n"
        "bx lr\n"
    );
}

void MSR_MSP(uint32_t addr)
{
    __asm__ volatile (
        "msr msp, r0\n"
        "bx lr\n"
    );
}
