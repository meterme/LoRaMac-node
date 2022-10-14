
//
//
//

#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "uart.h"


/* process GPS characters */

void ProcessGps( Uart_t* uart );
extern uint8_t coords[];

#ifdef __cplusplus
}
#endif

#endif // GPS_H