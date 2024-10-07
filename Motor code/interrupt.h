#ifndef INTERRUPT_H_
#define INTERRUPT_H_

void setup_SW1_interrupt();
void setup_PORTA_interrupt();
void PORTC_PORTD_IRQHandler(void);

void PORTC_PORTD_IRQHandler(void)
{
	// clear overflow
	PORTC->PCR[3] |= (1 << 24);
	timeDelay(1000);
	motor_drive();
}

void setup_SW1_interrupt()
{
	SIM->SCGC5 |= (1 << 11);
	PORTC->PCR[3] &= ~0xF0703;
	PORTC->PCR[3] |= 0xF0703 & ((0xA << 16) | (1 << 8) | 0x3);
	GPIOC->PDDR &= ~(1 << 3);

	NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

void PORTA_IRQHandler(void)
{
	GPIOD->PTOR |= (1 << 5);
	encoderCount++;
	PORTA->PCR[6] |= (1 << 24);
	PORTA->PCR[7] |= (1 << 24);
	PORTA->PCR[14] |= (1 << 24);
	PORTA->PCR[15] |= (1 << 24);
}

void setup_PORTA_interrupt()
{
	// enable clock gating for A
	SIM->SCGC5 |= (1 << 9);
	PORTA->PCR[6] &= ~(0xF0703);
	PORTA->PCR[7] &= ~(0xF0703);
	PORTA->PCR[14] &= ~(0xF0703);
	PORTA->PCR[15] &= ~(0xF0703);

	PORTA->PCR[6] |= 0xF0703 & ((0xA << 17) | (1 << 8) | 0b1001);
	PORTA->PCR[7] |= 0xF0703 & ((0xA << 17) | (1 << 8) | 0b1001);
	PORTA->PCR[14] |= 0xF0703 & ((0xA << 17) | (1 << 8) | 0b1001);
	PORTA->PCR[15] |= 0xF0703 & ((0xA << 17) | (1 << 8) | 0b1001);

	GPIOA->PDDR &= ~(1 << 6);
	GPIOA->PDDR &= ~(1 << 7);
	GPIOA->PDDR &= ~(1 << 14);
	GPIOA->PDDR &= ~(1 << 15);

	// LED test
	SIM->SCGC5 |= (1 << 12);
	PORTD->PCR[5] &= ~0x700;
	PORTD->PCR[5] |= 0x700 & (1 << 8);
	GPIOD->PDDR |= (1 << 5);

	NVIC_EnableIRQ(30);
}

#endif
