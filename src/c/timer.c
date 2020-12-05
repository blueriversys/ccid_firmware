#include "timer.h"

// This is not working correctly. Maybe the compiler is making an optimization here...???
void systick_wait_ms(int unit)
{
	int i;
	for (i=0; i<unit*500000; i++);
}

