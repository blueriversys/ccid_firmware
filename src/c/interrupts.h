#ifndef __INTERRUPTS_H__
#  define __INTERRUPTS_H__

int interrupts_get_and_disable(void);
void interrupts_enable(void);

extern void AT91F_enable_interrupt(void);
extern void AT91F_disable_interrupt(void);

#endif
