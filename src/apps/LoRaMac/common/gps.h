
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

void GpsProcess( Uart_t* uart );

#ifdef __cplusplus
}
#endif

#endif // GPS_H