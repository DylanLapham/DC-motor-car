#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"
#include "PID.h"

int main(void)
{
	int i;

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

	TPM2->MOD = 7999;

	TPM2->SC = 0x01 << 3;

	setup_SW1_interrupt();
	setup_PORTA_interrupt();

	for(;;)
	{
		i++;
	}

	return 0;
}

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

void PORTC_PORTD_IRQHandler(void)
{
	// clear overflow
	PORTC->PCR[3] |= (1 << 24);
	timeDelay(1000);
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
	encoderCount++;
	PORTA->PCR[6] |= (1 << 24);
	PORTA->PCR[7] |= (1 << 24);
	PORTA->PCR[14] |= (1 << 24);
	PORTA->PCR[15] |= (1 << 24);
	GPIOD->PTOR |= (1 << 5);
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

	SIM->SCGC5 |= (1 << 12);
	PORTD->PCR[5] &= ~0x700;
	PORTD->PCR[5] |= 0x700 & (1 << 8);
	GPIOD->PDDR |= (1 << 5);

	NVIC_EnableIRQ(30);
}

void motorDirection(int direction)
{
	if(direction == 1)
	{
		GPIOB->PDOR &= ~(1 << 0);
		GPIOB->PDOR |= (1 << 1);

		GPIOC->PDOR &= ~(1 << 1);
		GPIOC->PDOR |= (1 << 2);

		GPIOB->PDOR |= (1 << 3);
		GPIOB->PDOR |= (1 << 2);
	}

	else
	{
		GPIOB->PDOR &= ~(1 << 0);
		GPIOB->PDOR &= ~(1 << 1);

		GPIOC->PDOR &= ~(1 << 1);
		GPIOC->PDOR &= ~(1 << 2);

		GPIOB->PDOR |= (1 << 2);
		GPIOB->PDOR |= (1 << 3);
	}
}

float adjust_pid(float expectedOutput, float actualOutput)
{
	error = expectedOutput - actualOutput;
	integral = integral + error;
	derivative = error - previousError;

	output = Kp * error + Ki * integral + Kd * derivative;

	previousError = error;
	return output;
}

void motor_drive(void)
{
	motorDirection(1);
	while(t != 24)
	{
		t++;
		timeDelay(250);
		encoderCount = 0;
		float control = adjust_pid(expected_output, encoderCount);
		currentSpeed += control;

		// eliminating bias
		TPM2->CONTROLS[0].CnV = currentSpeed;
		TPM2->CONTROLS[1].CnV = currentSpeed;
	}

	motorDirection(6);
}
