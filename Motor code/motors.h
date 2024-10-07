#ifndef MOTORS_H_
#define MOTORS_H_

void motorDirection(char direction);
void motor_drive(void);
void timeDelay(unsigned short t);

void motorDirection(char direction)
{
	if(direction == 'F')
	{
		GPIOB->PDOR &= ~(1 << 0);
		GPIOB->PDOR |= (1 << 1);

		GPIOC->PDOR &= ~(1 << 1);
		GPIOC->PDOR |= (1 << 2);

		GPIOB->PDOR |= (1 << 3);
		GPIOB->PDOR |= (1 << 2);
	}

	else if(direction == 'B')
	{
		GPIOB->PDOR |= (1 << 0);
		GPIOB->PDOR &= ~(1 << 1);

		GPIOC->PDOR |= (1 << 1);
		GPIOC->PDOR &= ~(1 << 2);

		GPIOB->PDOR |= (1 << 2);
		GPIOB->PDOR |= (1 << 3);
	}

	else if(direction == 'R')
	{
		GPIOB->PDOR &= ~(1 << 0);
		GPIOB->PDOR |= (1 << 1);

		GPIOC->PDOR |= (1 << 1);
		GPIOC->PDOR &= ~(1 << 2);

		GPIOB->PDOR |= (1 << 2);
		GPIOB->PDOR |= (1 << 3);
	}

	else if(direction == 'L')
	{
		GPIOB->PDOR |= (1 << 0);
		GPIOB->PDOR &= ~(1 << 1);

		GPIOC->PDOR &= ~(1 << 1);
		GPIOC->PDOR |= (1 << 2);

		GPIOB->PDOR |= (1 << 2);
		GPIOB->PDOR |= (1 << 3);
	}

	else if(direction == 'SB')
	{
		GPIOB->PDOR |= (1 << 0);
		GPIOB->PDOR |= (1 << 1);

		GPIOC->PDOR &= ~(1 << 1);
		GPIOC->PDOR &= ~(1 << 2);

		GPIOB->PDOR |= (1 << 2);
		GPIOB->PDOR &= ~(1 << 3);
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

#endif
