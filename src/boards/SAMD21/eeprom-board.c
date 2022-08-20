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
selectEeprom()
{
	gpio_set_pin_level(EEPROM_CS, false);
}

static void
deselectEeprom()
{
	gpio_set_pin_level(EEPROM_CS, true);	
}

uint8_t
readStatus(uint8_t regOp)
{
	uint8_t cmd[2] = { regOp };

	selectEeprom();
	SPI_1_exchange_block(cmd, sizeof (cmd));
	deselectEeprom();

	return (cmd[1]);
}

uint8_t
readStatus1(void)
{
	return (readStatus(0x05));
}

uint8_t
readStatus2(void)
{
	return (readStatus(0x35));
}

void
enableWrite()
{
	selectEeprom();
	SPI_1_exchange_data(0x06);
	deselectEeprom();
}

void
eraseChip()
{
	selectEeprom();
	SPI_1_exchange_data(0x60);
	deselectEeprom();
}

uint32_t count;

static void
erase4kBlock(uint32_t addr)
{
	uint8_t cmd[4];
	
	cmd[0] = 0x20;
	cmd[1] = addr >> 16;
	cmd[2] = addr >> 8;
	cmd[3] = addr;
	
	selectEeprom();
	SPI_1_exchange_block((void *)cmd, sizeof (cmd));
	deselectEeprom();
	
	count = 0;
	while (readStatus1() & 1) {
		count++;
	}
}

static void
readBlock(uint32_t addr, uint8_t *buffer, size_t len) {
	uint8_t cmd[4];
	
	cmd[0] = 0x03;
	cmd[1] = addr >> 16;
	cmd[2] = addr >> 8;
	cmd[3] = addr;
	
	selectEeprom();
	SPI_1_exchange_block((void *)cmd, sizeof (cmd));
	SPI_1_read_block((void *)buffer, len);
	deselectEeprom();
}

static void
writeBlock(uint32_t addr, uint8_t *buffer, size_t len) {
	uint8_t cmd[4];
	
	cmd[0] = 0x02;
	cmd[1] = addr >> 16;
	cmd[2] = addr >> 8;
	cmd[3] = addr;

	selectEeprom();
	SPI_1_exchange_block((void *)cmd, sizeof (cmd));
	SPI_1_write_block((void *)buffer, len);
	deselectEeprom();

	count = 0;
	while (readStatus1() & 1) {
		count++;
	}
}

static void
readEepromPage()
{
	uint32_t addr;

	for (addr = 0; addr < EEPROM_PAGE_SIZE; addr += EEPROM_BUFFER_SIZE) {
		readBlock(addr, eepromPage + addr, EEPROM_BUFFER_SIZE);
	}

	eepromPageValid = true;
}

static void
writeEepromPage()
{
	uint32_t addr;
	
	// enable write
	enableWrite();
	erase4kBlock(0x0);

	// program the page
	for (addr = 0; addr < EEPROM_PAGE_SIZE; addr += EEPROM_BUFFER_SIZE) {
		enableWrite();
		writeBlock(addr, eepromPage + addr, EEPROM_BUFFER_SIZE);
	}
}

LmnStatus_t EepromMcuWriteBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
	if (!eepromPageValid) {
		readEepromPage();
	}

	memcpy1(eepromPage + addr, buffer, size);
	writeEepromPage();

    return LMN_STATUS_OK;
}

LmnStatus_t EepromMcuReadBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
	if (!eepromPageValid) {
		readEepromPage();
	}
    
	memcpy1(buffer, eepromPage + addr, size);
	return LMN_STATUS_OK;
}

void EepromMcuSetDeviceAddr( uint8_t addr )
{
    while( 1 )
    {
    }
}

LmnStatus_t EepromMcuGetDeviceAddr( void )
{
    while( 1 )
    {
    }
//    return 0;
}
