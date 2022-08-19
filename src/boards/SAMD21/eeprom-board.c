/*!
 * \file      eeprom-board.c
 *
 * \brief     Target board EEPROM driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include "utilities.h"
#include "eeprom-board.h"

#include <hal_gpio.h>
#include <spi_lite.h>

#define	EEPROM_PAGE_SIZE	4096
#define	EEPROM_BUFFER_SIZE	128

// XXX: find this a better home
#define EEPROM_CS                                   GPIO( GPIO_PORTA, 6)

static uint8_t eepromPage[EEPROM_PAGE_SIZE];
static bool eepromPageValid = false;

static void
readEepromPage()
{
	uint8_t cmd[4] = { 0x03 };
	uint32_t addr;

	for (addr = 0; addr < EEPROM_PAGE_SIZE; addr += EEPROM_BUFFER_SIZE) {
		cmd[1] = addr >> 16;
		cmd[2] = addr >> 8;
		cmd[3] = addr;

		gpio_set_pin_level(EEPROM_CS, false);
		SPI_1_exchange_block((void *)cmd, sizeof (cmd));
		SPI_1_read_block((void *)(eepromPage + addr), EEPROM_BUFFER_SIZE);
		gpio_set_pin_level(EEPROM_CS, true);
	}

	eepromPageValid = true;
}

static void
writeEepromPage()
{


}

uint8_t EepromMcuWriteBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
	if (eepromPageValid) {
		readEepromPage();
	}

	memcpy1(eepromPage + addr, buffer, size);
	writeEepromPage();

    return SUCCESS;
}

uint8_t EepromMcuReadBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
	if (eepromPageValid) {
		readEepromPage();
	}
    
	memcpy1(buffer, eepromPage + addr, size);
	return SUCCESS;
}

void EepromMcuSetDeviceAddr( uint8_t addr )
{
    while( 1 )
    {
    }
}

uint8_t EepromMcuGetDeviceAddr( void )
{
    while( 1 )
    {
    }
}
