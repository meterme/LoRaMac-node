/*!
 * \file      gps-board.c
 *
 * \brief     Target board GPS driver implementation
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
#include "board-config.h"
#include "board.h"
#include "gpio.h"
#include "gps.h"
#include "uart.h"
#include "lpm-board.h"
#include "rtc-board.h"
#include "gps-board.h"

/*!
 * \brief Maximum number of data byte that we will accept from the GPS
 */
static volatile uint8_t NmeaStringSize = 0;

Gpio_t GpsPowerEn;
Gpio_t GpsPps;

extern Uart_t GpsUart;

PpsTrigger_t PpsTrigger;

void GpsMcuOnPpsSignal( void* context )
{

}

void GpsMcuInvertPpsTrigger( void )
{

}

void GpsMcuInit( void )
{

}

void GpsMcuStart( void )
{
}

void GpsMcuStop( void )
{
}

void GpsMcuProcess( void )
{

}

void GpsMcuIrqNotify( UartNotifyId_t id )
{
    uint8_t data;
    if( id == UART_NOTIFY_RX )
    {
        if( UartGetChar( &GpsUart, &data ) == 0 ) {
            processGpsChar(data);
        }
    }
}
