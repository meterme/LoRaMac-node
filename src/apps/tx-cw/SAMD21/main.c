/*!
 * \file      main.c
 *
 * \brief     Tx Continuous Wave implementation
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
 *
 * \author    Marten Lootsma(TWTG) on behalf of Microchip/Atmel (c)2017
 */
#include "board.h"
#include "gpio.h"
#include "timer.h"
#include "radio.h"

#include "delay.h"

#include <math.h>

extern uint16_t readAdc(void);


#if defined( REGION_AS923 )

#define RF_FREQUENCY                                923000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm

#elif defined( REGION_AU915 )

#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm

#elif defined( REGION_CN470 )

#define RF_FREQUENCY                                470000000 // Hz
#define TX_OUTPUT_POWER                             20        // 20 dBm

#elif defined( REGION_CN779 )

#define RF_FREQUENCY                                779000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm

#elif defined( REGION_EU433 )

#define RF_FREQUENCY                                433000000 // Hz
#define TX_OUTPUT_POWER                             20        // 20 dBm

#elif defined( REGION_EU868 )

#define RF_FREQUENCY                                868000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm

#elif defined( REGION_KR920 )

#define RF_FREQUENCY                                920000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm

#elif defined( REGION_IN865 )

#define RF_FREQUENCY                                865000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm

#elif defined( REGION_US915 )

#define RF_FREQUENCY                                910000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm

#elif defined( REGION_RU864 )

#define RF_FREQUENCY                                864000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm

#else

    #error "Please define a frequency band in the compiler options."

#endif
#define TX_TIMEOUT                                  1     // seconds (MAX value)

int8_t txPower;
int16_t txVal;
int16_t aVal;
float floorPower;
float fVal[22];
float pVal[22];

volatile bool txDone = false;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnRadioTxTimeout( void )
{
    // Restarts continuous wave transmission when timeout expires

    txDone = true;
#if 0
    Delay(2.0);
    Radio.SetTxContinuousWave( RF_FREQUENCY, txPower, TX_TIMEOUT );
    Delay(0.1);
    aVal = readAdc();
    fVal = (aVal / 32768.0f);
#endif
}

float
convertdBm(float mVal)
{
    float adjV, p, dBm;

    adjV = mVal / 1.9f;
    p = (adjV * adjV) / 50.0f;
    dBm = 10.0f * log10f(p * 1000.0f);
    return (dBm + 29.5f);
}


/**
 * Main application entry point.
 */
int main( void )
{
    // Target board initialization
    BoardInitMcu( );
    BoardInitPeriph( );

    // get 'floor' base power reading
    aVal = readAdc();
    floorPower = convertdBm(aVal / 32768.0f);

    // Radio initialization
    RadioEvents.TxTimeout = OnRadioTxTimeout;
    Radio.Init( &RadioEvents );

    for (txPower = 0; txPower < 22; txPower++) {
        txDone = false;
        Radio.SetTxContinuousWave( RF_FREQUENCY, txPower, TX_TIMEOUT );
        Delay(0.1);
        aVal = readAdc();
        fVal[txPower] = (aVal / 32768.0f);
        pVal[txPower] = convertdBm(fVal[txPower]);

        while (!txDone) {
            // wait
        }
        Delay(1.0);
    }
    // Radio.SetTxContinuousWave( RF_FREQUENCY, txPower, TX_TIMEOUT );

    // Blink LEDs just to show some activity
    while( 1 )
    {
        // Tick the RTC to execute callback in context of the main loop (in stead of the IRQ)
        TimerProcess( );
    }
}
