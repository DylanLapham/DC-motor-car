#ifndef INIT_H_
#define INIT_H_

void initializePins(void);
void timeDelay(unsigned short t);

void timeDelay(unsigned short t)
{
	// enable clock gating and OSCERCLK.
	SIM->SCGC6 |= (1 << 24);
	SIM->SOPT2 |= (0x2 << 24);

	TPM0->CONF |= (0x1 << 17);

	// overflow flag check
	TPM0->SC = (0x1 << 7) | (0x07);

	// MOD value is the result of equation. You get roughly 61.5
	TPM0->MOD = t * 61 + t/2;

	// start clock!
	TPM0->SC |= 0x01 << 3;

	// ticks away until no longer time.
	while(!(TPM0->SC & 0x80)){}
	return;
}

void initializePins(void)
{
	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	// enable clock gating for A, B, C
	SIM->SCGC5 |= (1 << 10) | (1 << 11);

	// ports setup
	PORTB->PCR[0] &= ~0x700;
	PORTB->PCR[1] &= ~0x700;
	PORTB->PCR[2] &= ~0x700;
	PORTB->PCR[3] &= ~0x700;
	PORTB->PCR[0] |= 0x700 & (1 << 8);
	PORTB->PCR[1] |= 0x700 & (1 << 8);

	// driven with source clock not GPIO
	PORTB->PCR[2] |= 0x300;
	PORTB->PCR[3] |= 0x300;

	PORTC->PCR[1] &= ~0x700;
	PORTC->PCR[2] &= ~0x700;
	PORTC->PCR[1] |= 0x700 & (1 << 8);
	PORTC->PCR[2] |= 0x700 & (1 << 8);

	GPIOB->PDDR |= (1 << 0);
	GPIOB->PDDR |= (1 << 1);
	GPIOB->PDDR |= (1 << 2);
	GPIOB->PDDR |= (1 << 3);

	GPIOC->PDDR |= (1 << 1);
	GPIOC->PDDR |= (1 << 2);

	// TPM2
	SIM->SCGC6 |= (1 << 26);
	SIM->SOPT2 |= (0x2 << 24);

	TPM2->CONTROLS[0].CnSC |= (0x2 << 2) | (0x2 << 4);
	TPM2->CONTROLS[1].CnSC |= (0x2 << 2) | (0x2 << 4);

	// TPM1
	SIM->SCGC6 |= (1 << 25);
	SIM->SOPT2 |= (0x1 << 25);

	TPM2->MOD = 7999;

	TPM2->SC = 0x01 << 3;
}

#endif
