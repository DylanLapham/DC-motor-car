#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"
#include "motors.h"
#include "interrupt.h"
#include "init.h"

int main(void)
{
	int i;

	initializePins();
	setup_SW1_interrupt();
	setup_PORTA_interrupt();

	motorDirection('F');	// forward
	timeDelay(100);			// 100ms

	for(;;)
	{
		i++;
	}

	return 0;
}
