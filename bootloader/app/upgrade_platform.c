#include "upgrade_platform.h"
#include "upgrade.h"
#include "main.h"

#if defined(__CC_ARM)
__asm void upgrade_jump(uint32_t stack, uint32_t entry)
{
  MSR MSP, r0
  MOVS r2, #0
  MSR CONTROL, r2
  MSR BASEPRI, r2
  MSR FAULTMASK, r2
  ISB
  LDR r3, =0xFFFFFFFF
  MOV lr, r3
  CPSIE i
  BX r1
}
#else
__attribute__((naked, noreturn)) static void upgrade_jump(
    uint32_t stack __attribute__((unused)), uint32_t entry __attribute__((unused)))
{
  __asm volatile(
      "msr msp, r0\n"
      "movs r2, #0\n"
      "msr control, r2\n"
      "msr basepri, r2\n"
      "msr faultmask, r2\n"
      "isb\n"
      "ldr r3, =0xFFFFFFFF\n"
      "mov lr, r3\n"
      "cpsie i\n"
      "bx r1\n");
}
#endif

void Upgrade_JumpToApp(uint32_t stack, uint32_t entry)
{
  uint32_t groups, i;
  __disable_irq();
  SysTick->CTRL = 0u;
  SysTick->LOAD = 0u;
  SysTick->VAL = 0u;
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
  groups = (SCnSCB->ICTR & SCnSCB_ICTR_INTLINESNUM_Msk) + 1u;
  if(groups > 8u)
    groups = 8u;
  for(i = 0u; i < groups; ++i)
  {
    NVIC->ICER[i] = 0xffffffffu;
    NVIC->ICPR[i] = 0xffffffffu;
  }
  SCB->VTOR = MSP1_ADDR;
  __DSB();
  __ISB();
  upgrade_jump(stack, entry);
}
