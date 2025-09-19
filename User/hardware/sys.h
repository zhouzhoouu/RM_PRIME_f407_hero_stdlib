#ifndef SYS_H
#define SYS_H
#include "main.h"

void WFI_SET(void) __attribute__((naked));
void INTX_DISABLE(void) __attribute__((naked));
void INTX_ENABLE(void) __attribute__((naked));
void MSR_MSP(uint32_t addr) __attribute__((naked));

#endif
