/*!
 * \file      board-config.h
 *
 * \brief     Board configuration
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
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 *
 * \author    Johannes Bruder ( STACKFORCE )
 *
 * \author    Marten Lootsma(TWTG) on behalf of Microchip/Atmel (c)2017
 */
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#include <hal_gpio.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME                      1

/*!
 * Board MCU pins definitions
 */

#define RADIO_RESET                                 GPIO( GPIO_PORTA, 13 )
#define RXEN_PIN                                    GPIO( GPIO_PORTB, 23 )
#define TXEN_PIN                                    GPIO( GPIO_PORTB, 22 )

#define RADIO_MOSI                                  GPIO( GPIO_PORTB, 10 )
#define RADIO_MISO                                  GPIO( GPIO_PORTA, 12 )
#define RADIO_SCLK                                  GPIO( GPIO_PORTB, 11 )
#define RADIO_NSS                                   GPIO( GPIO_PORTA, 19 )

#define RADIO_DIO_0                                 GPIO( GPIO_PORTA, 20 )
#define RADIO_DIO_1                                 GPIO( GPIO_PORTA, 18 )
#define RADIO_DIO_2                                 GPIO( GPIO_PORTA, 16 )

#define LED_RX                                      GPIO( GPIO_PORTB, 3 )
#define LED_TX                                      GPIO( GPIO_PORTA, 27 )
#define LED_D13                                     GPIO( GPIO_PORTA, 17 )

#define UART_TX                                     GPIO( GPIO_PORTA, 10 )
#define UART_RX                                     GPIO( GPIO_PORTA, 11 )
#define I2C_SDA                                     GPIO( GPIO_PORTA, 22 )
#define I2C_SCL                                     GPIO( GPIO_PORTA, 23 )

#define EEPROM_MOSI                                 GPIO( GPIO_PORTA, 8)
#define EEPROM_MISO                                 GPIO( GPIO_PORTA, 14)
#define EEPROM_SCLK                                 GPIO( GPIO_PORTA, 9)
#define EEPROM_CS                                   GPIO( GPIO_PORTA, 6)

#define ADC_IN1                                     GPIO( GPIO_PORTA, 2)
#define ADC_IN2                                     GPIO( GPIO_PORTA, 3)

// Debug pins definition.
#define RADIO_DBG_PIN_TX                            NC
#define RADIO_DBG_PIN_RX                            NC

#ifdef __cplusplus
}
#endif

#endif // __BOARD_CONFIG_H__
