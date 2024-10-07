#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"
#include "I2C.h"

void timeDelay(unsigned short t)
{
	SIM->SCGC6 |= (1 << 24);
	SIM->SOPT2 |= (0x2 << 24);
	TPM0->CONF |= (0x1 << 17);
	TPM0->SC = (1 << 7) | (0x7);

	TPM0->MOD = t * 62 + t/2;

	TPM0->SC |= 0x01 << 3;

	while(!(TPM0->SC & 0x80)){}
	return;
}

void motor_control(char motor, char direction, int speed)
{

	if(motor == 0)
	{
		TPM2->CONTROLS[0].CnV = speed;
		if(direction == 0)
		{
			GPIOB->PDOR &= ~(1 << 1);
			GPIOB->PDOR |= (1 << 0);
		}

		if(direction == 1)
		{
			GPIOB->PDOR &= ~0x1;
			GPIOB->PDOR |= 0x2;
		}
	}

	if(motor == 1)
	{
		TPM2->CONTROLS[1].CnV = speed;
		if(direction == 0)
		{
			GPIOC->PDOR &= ~0x4;
			GPIOC->PDOR |= 0x2;
		}

		if(direction == 1)
		{
			GPIOC->PDOR &= ~0x2;
			GPIOC->PDOR |= 0x4;
		}
	}
}

void motor_break(char motor, char stp_brk)
{
	if((motor == 0) & (stp_brk == 0))
	{
		GPIOB->PDOR |= 0x3;
	}

	if((motor == 1) & (stp_brk == 0))
	{
		GPIOC->PDOR |= 0x6;
	}

	if((motor == 0) & (stp_brk == 1))
	{
		GPIOB->PDOR &= ~0x3;
	}

	if((motor == 1) & (stp_brk == 1))
	{
		GPIOC->PDOR &= ~0x6;
	}
}

void trace_line()
{
	ADC0->SC1[0] = 0x05;
	while(!(ADC0->SC1[0] & 0x80)){}
	leftSensor = ADC0->R[0];

	if(leftSensor > 174)
	{
		motor_control(1, 1, 400);
	}

	else
	{
		motor_control(1, 1, 700);
	}

	ADC0->SC1[0] = 0x01;
	while(!(ADC0->SC1[0] & 0x80)){}
	rightSensor = ADC0->R[0];
	if(rightSensor > 174)
	{
		motor_control(0, 0, 400);
	}

	else
	{
		motor_control(0, 0, 700);
	}
}

int calulateColor()
{
    C = color[1] << 8 | color[0];
    R = color[3] << 8 | color[2];
    G = color[5] << 8 | color[4];
    B = color[7] << 8 | color[6];

    C = C/10;
    R = R/10;
    G = G/10;
    B = B/10;

    if (14 <= C && C <= 16)
    {
    	 return 1;
    }

    if (24 <= C && C <= 28)
    {
        return 2;
    }

    if (31 < C && C < 50)
    {
        return 3;
    }

    if (B <= G && 14 <= C && C <= 16)
    {
        return 4;
    }

    return -1;
}

int getColor()
{
    uint8_t cmdcode = (1 << 7) | (1 << 5) | (0x14);

    I2C_read(cmdcode, color, 8);
    i = calulateColor();
    return i;
}

void setup_SW1_interrupt()
{
	SIM->SCGC5 |= (1 << 11);
	PORTC->PCR[3] &= ~0xF0703;
	PORTC->PCR[3] |= 0xF0703 & ((0xA << 16) | (1 << 8) | 0x3 );
	GPIOC->PDDR &= ~(1 << 3);
	NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

void PORTC_PORTD_IRQHandler(void)
{
	timeDelay(2000);
	init_I2C();
	I2C_write(0x80, 0x03);
	timeDelay(3);
	I2C_write( (0x80 | 0x01), 0xFF);
	timeDelay(3);
	start = getColor();

	if(start == 3)
	{
		stopVar = 1;
	}

	motor_control(0, 0, 700);
	motor_control(1, 1, 700);

	while(1)
	{
		int color = getColor();
		while(color == 3)
		{
			color = getColor();
			motor_control(1, 1, 700);
			motor_control(0, 0, 300);

			if(color == stopVar)
			{
				motor_break(0, 1);
				motor_break(1, 1);
			}
		}
		trace_line();
	}
}


int main(void)
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole();
#endif

    SIM->SCGC5 |= (1 << 10 | 1 << 11 | 1 << 12);

    	SIM->SCGC6 |= (1 << 26);
    	SIM->SOPT2 |= (0x2 << 24);
    	TPM2->CONTROLS[0].CnV = 600;
    	TPM2->CONTROLS[1].CnV = 600;
    	TPM2->MOD = 1000;

    	PORTB->PCR[0] &= ~0x703;
    	PORTB->PCR[1] &= ~0x703;
    	PORTB->PCR[2] &= ~0x703;
    	PORTB->PCR[3] &= ~0x703;
    	PORTC->PCR[1] &= ~0x703;
    	PORTC->PCR[2] &= ~0x703;

    	TPM2->CONTROLS[0].CnSC |= (1 << 3) | (1 << 5);
    	TPM2->CONTROLS[1].CnSC |= (1 << 3) | (1 << 5);

    	TPM2->SC |= 1 << 3;

    	PORTB->PCR[0] |= 1 << 8;
    	PORTB->PCR[1] |= 1 << 8;
    	PORTC->PCR[1] |= 1 << 8;
    	PORTC->PCR[2] |= 1 << 8;

    	PORTB->PCR[2] |= 3 << 8;
    	PORTB->PCR[3] |= 3 << 8;

    	GPIOB->PDDR |= 0xF;
    	GPIOC->PDDR |= 0x6;

    unsigned short cal_v = 0;

    	//line sensor setup
    	SIM->SCGC5 |= (1 << 13);
    	SIM->SCGC6 |= (1 << 27);
    	// setup ADC Clock ( < 4 MHz)
    	ADC0->CFG1 = 0; // default everything.
    	// analog Calibrate
    	ADC0->SC3 = 0x07; // enable Maximum Hardware Averaging
    	ADC0->SC3 |= 0x80; // start Calibration
    	// wait for calibration to complete

    	while(!(ADC0->SC1[0] & 0x80)){ }
    	// calibration Complete, time to write calibration registers.
    	cal_v = ADC0->CLP0 + ADC0->CLP1 + ADC0->CLP2 + ADC0->CLP3 + ADC0->CLP4 + ADC0->CLPS;
    	cal_v = cal_v >> 1 | 0x8000;
    	ADC0->PG = cal_v;
    	cal_v = 0;
    	cal_v = ADC0->CLM0 + ADC0->CLM1 + ADC0->CLM2 + ADC0->CLM3 + ADC0->CLM4 + ADC0->CLMS;
    	cal_v = cal_v >> 1 | 0x8000;
    	ADC0->MG = cal_v;
    	ADC0->SC3 = 0; // turn off Hardware Averaging

    // in theory, allows a switch press via interrupt to begin
    setup_SW1_interrupt();
    int i = 0;

    PORTD->PCR[5] &= ~(0x700);
    PORTD->PCR[5] |= 0x700;
    GPIOD->PDDR |= (1 << 5);

    while(1)
    {
    	i++;
    	GPIOD->PTOR |= (1 << 5);
    }

    return 0 ;
}
