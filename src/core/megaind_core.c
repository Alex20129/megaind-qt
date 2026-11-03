/*
 * megaind_core.c - Core functions extracted from megaind.c without CLI
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "megaind.h"
#include "comm.h"

int doBoardInit(int stack)
{
	int dev = 0;
	int add = 0;
	uint8_t buff[8];

	if ( (stack < 0) || (stack > 7))
	{
		return ERROR;
	}
	add = stack + SLAVE_OWN_ADDRESS_BASE;
	dev = i2cSetup(add);
	if (dev == -1)
	{
		return ERROR;
	}
	if (ERROR == i2cMem8Read(dev, I2C_MEM_REVISION_MAJOR, buff, 1))
	{
		// Board not detected
		return ERROR;
	}
	return dev;
}

int boardCheck(int stack)
{
	int dev = 0;
	int add = 0;
	uint8_t buff[8];

	if ( (stack < 0) || (stack > 7))
	{
		return ERROR;
	}
	add = stack + SLAVE_OWN_ADDRESS_BASE;
	dev = i2cSetup(add);
	if (dev == -1)
	{
		return ERROR;
	}
	if (ERROR == i2cMem8Read(dev, I2C_MEM_REVISION_MAJOR, buff, 1))
	{
		return ERROR;
	}
	return OK;
}