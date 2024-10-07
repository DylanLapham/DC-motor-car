uint8_t color[8];
int i;
unsigned int leftSensor = 0;
unsigned int rightSensor = 0;
uint16_t C, R, G, B;
int stopVar;
int start;

void clear()
{
	I2C0->FLT |= (1 << 6);
	I2C0->FLT &= ~(1 << 4);
	I2C0->S |= (1 << 4);
	I2C0->S |= (1 << 1);
}

void TCF()
{
	// wait for TCF
	while(!(I2C0->S | 1 << 7)){}
}

void IICIF()
{
	// wait for IICIF
	while(!(I2C0->S & 1 << 1)){}
}

void begin()
{
    // sets master and Tx
	I2C0->C1 |= (1<<5) | (1<<4);
}

void restart()
{
    // sets master, Tx, and restart
	I2C0->C1 |= (1 << 5) | (1 << 4) | (1 << 2);
	int i;
	for (i = 0; i < 10; i++)
	{
		__asm volatile ("nop");		// do nothing 10 times
	}
}

void stop()
{
	I2C0->C1 &= ~((1 << 5) | (1 << 4) | (1 << 3));
	while((I2C0->S & (1 << 5))){}
}

void clearIICIF()
{
	// clear IICIF
	I2C0->S |= (1 << 1);
}

int ack()
{
	// check if no or yes
	if(!(I2C0->S) & (0x1))
	{
		return 0;
	}
	return 1;
}

void I2C_write(uint8_t reg_address, uint8_t Data)
{
	clear();
	TCF();
	begin();
	I2C0->D = (0x29 << 1); // color sensor address
	IICIF();

	if (!ack())
	{
		stop();
		return;
	}

	clearIICIF();
	I2C0->D = (reg_address);
	IICIF();

	if (!ack())
	{
		stop();
		return;
	}

	TCF();
	clearIICIF();
	I2C0->D = (Data);
	IICIF();

	if(!ack())
	{
		printf("data NACK");
	}

	clearIICIF();
	stop();
}

void I2C_read(uint8_t reg_address, uint8_t* data_array, int Length)
{
	unsigned char temp = 0;
	clear();
	TCF();
	begin();
	temp++;
	I2C0->D = (0x29 << 1);
	IICIF();

	if (!ack())
	{
		stop();
	}

	clearIICIF();
	I2C0->D = (reg_address);
	IICIF();

	if (!ack())
	{
		stop();
	}

	clearIICIF();
	restart();
	I2C0->D = (0x29 << 1) | 0x1;
	IICIF();

	if (!ack())
	{
		stop();
	}

	TCF();
	clearIICIF();
	I2C0->C1 &= ~((1 << 3) | (1 << 4));

	if(Length == 1)
	{
		I2C0->C1 |= (1 << 3);		// nack
	}

   temp = I2C0->D;
   for(int index = 0; index < Length; index++)
   {
		IICIF();
		clearIICIF();

		if((Length - index) == 2)
		{
			I2C0->C1 |= (0x8);	// stop
		}

		if((Length - index) == 1)
		{
			stop();
		}

		data_array[index] = I2C0->D;
	}
}

void init_I2C()
{
    // enable Clock Gating for I2C
	SIM->SCGC4 |= (1 << 6);
	PORTC->PCR[8] &= ~(0x700);
	PORTC->PCR[9] &= ~(0x700);
	PORTC->PCR[8] |= (0x700) & (0x200); // SCL = ALT2
	PORTC->PCR[9] |= (0x700) & (0x200); // SDA = ALT2

    // clear all I2C registers
	I2C0->A1 = 0;
	I2C0->F = 0;
	I2C0->C1 = 0;
	I2C0->S = 0;
	I2C0->D = 0;
	I2C0->C2 = 0;
	I2C0->FLT = 0;
	I2C0->A2 = 0;
	I2C0->RA = 0;
	I2C0->SMB = 0;
	I2C0->SLTH = 0;
	I2C0->SLTL = 0;


	I2C0->FLT |= (0x50); // set filter
	clear();

	I2C0->F &= (0x23); // set I2C Divider Register
	I2C0->C1 |= (1 << 7); // enable I2C
}

