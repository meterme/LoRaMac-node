/*!
 * \file      gps.c
 *
 * \brief     GPS driver implementation
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
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "utilities.h"
#include "board.h"
#include "rtc-board.h"
#include "gps-board.h"
#include "gps.h"

#define TRIGGER_GPS_CNT                             10

/* Various type of NMEA data we can receive with the Gps */
const char NmeaDataTypeGPGGA[] = "GPGGA";
const char NmeaDataTypeGPGSA[] = "GPGSA";
const char NmeaDataTypeGPGSV[] = "GPGSV";
const char NmeaDataTypeGPRMC[] = "GPRMC";

/* Value used for the conversion of the position from DMS to decimal */
const int32_t MaxNorthPosition = 8388607;       // 2^23 - 1
const int32_t MaxSouthPosition = 8388608;       // -2^23
const int32_t MaxEastPosition = 8388607;        // 2^23 - 1
const int32_t MaxWestPosition = 8388608;        // -2^23

NmeaGpsData_t NmeaGpsData;

static bool HasFix = false;
static double Latitude = 0;
static double Longitude = 0;

static int32_t LatitudeBinary = 0;
static int32_t LongitudeBinary = 0;

static int16_t Altitude = ( int16_t )0xFFFF;

static uint32_t PpsCnt = 0;

bool PpsDetected = false;

void GpsPpsHandler( bool *parseData )
{
    PpsDetected = true;
    PpsCnt++;
    *parseData = false;

    if( PpsCnt >= TRIGGER_GPS_CNT )
    {
        PpsCnt = 0;
        *parseData = true;
    }
}

void GpsInit( void )
{
    PpsDetected = false;
    GpsMcuInit( );
}

void GpsStart( void )
{
    GpsMcuStart( );
}

void GpsStop( void )
{
    GpsMcuStop( );
}

void GpsProcess( void )
{
    GpsMcuProcess( );
}

bool GpsGetPpsDetectedState( void )
{
    bool state = false;

    CRITICAL_SECTION_BEGIN( );
    state = PpsDetected;
    PpsDetected = false;
    CRITICAL_SECTION_END( );
    return state;
}

bool GpsHasFix( void )
{
    return HasFix;
}

void GpsConvertPositionIntoBinary( void )
{
    long double temp;

    if( Latitude >= 0 ) // North
    {
        temp = Latitude * MaxNorthPosition;
        LatitudeBinary = temp / 90;
    }
    else                // South
    {
        temp = Latitude * MaxSouthPosition;
        LatitudeBinary = temp / 90;
    }

    if( Longitude >= 0 ) // East
    {
        temp = Longitude * MaxEastPosition;
        LongitudeBinary = temp / 180;
    }
    else                // West
    {
        temp = Longitude * MaxWestPosition;
        LongitudeBinary = temp / 180;
    }
}

void GpsConvertPositionFromStringToNumerical( void )
{
    int i;

    double valueTmp1;
    double valueTmp2;
    double valueTmp3;
    double valueTmp4;

    // Convert the latitude from ASCII to uint8_t values
    for( i = 0 ; i < 10 ; i++ )
    {
        NmeaGpsData.NmeaLatitude[i] = NmeaGpsData.NmeaLatitude[i] & 0xF;
    }
    // Convert latitude from degree/minute/second (DMS) format into decimal
    valueTmp1 = ( double )NmeaGpsData.NmeaLatitude[0] * 10.0 + ( double )NmeaGpsData.NmeaLatitude[1];
    valueTmp2 = ( double )NmeaGpsData.NmeaLatitude[2] * 10.0 + ( double )NmeaGpsData.NmeaLatitude[3];
    valueTmp3 = ( double )NmeaGpsData.NmeaLatitude[5] * 1000.0 + ( double )NmeaGpsData.NmeaLatitude[6] * 100.0 +
                ( double )NmeaGpsData.NmeaLatitude[7] * 10.0 + ( double )NmeaGpsData.NmeaLatitude[8];

    Latitude = valueTmp1 + ( ( valueTmp2 + ( valueTmp3 * 0.0001 ) ) / 60.0 );

    if( NmeaGpsData.NmeaLatitudePole[0] == 'S' )
    {
        Latitude *= -1;
    }

    // Convert the longitude from ASCII to uint8_t values
    for( i = 0 ; i < 10 ; i++ )
    {
        NmeaGpsData.NmeaLongitude[i] = NmeaGpsData.NmeaLongitude[i] & 0xF;
    }
    // Convert longitude from degree/minute/second (DMS) format into decimal
    valueTmp1 = ( double )NmeaGpsData.NmeaLongitude[0] * 100.0 + ( double )NmeaGpsData.NmeaLongitude[1] * 10.0 + ( double )NmeaGpsData.NmeaLongitude[2];
    valueTmp2 = ( double )NmeaGpsData.NmeaLongitude[3] * 10.0 + ( double )NmeaGpsData.NmeaLongitude[4];
    valueTmp3 = ( double )NmeaGpsData.NmeaLongitude[6] * 1000.0 + ( double )NmeaGpsData.NmeaLongitude[7] * 100;
    valueTmp4 = ( double )NmeaGpsData.NmeaLongitude[8] * 10.0 + ( double )NmeaGpsData.NmeaLongitude[9];

    Longitude = valueTmp1 + ( valueTmp2 / 60.0 ) + ( ( ( valueTmp3 + valueTmp4 ) * 0.0001 ) / 60.0 );

    if( NmeaGpsData.NmeaLongitudePole[0] == 'W' )
    {
        Longitude *= -1;
    }
}


LmnStatus_t GpsGetLatestGpsPositionDouble( double *lati, double *longi )
{
    LmnStatus_t status = LMN_STATUS_ERROR;
    if( HasFix == true )
    {
        status = LMN_STATUS_OK;
    }
    else
    {
        GpsResetPosition( );
    }
    *lati = Latitude;
    *longi = Longitude;
    return status;
}

LmnStatus_t GpsGetLatestGpsPositionBinary( int32_t *latiBin, int32_t *longiBin )
{
    LmnStatus_t status = LMN_STATUS_ERROR;

    CRITICAL_SECTION_BEGIN( );
    if( HasFix == true )
    {
        status = LMN_STATUS_OK;
    }
    else
    {
        GpsResetPosition( );
    }
    *latiBin = LatitudeBinary;
    *longiBin = LongitudeBinary;
    CRITICAL_SECTION_END( );
    return status;
}

int16_t GpsGetLatestGpsAltitude( void )
{
    CRITICAL_SECTION_BEGIN( );
    if( HasFix == true )
    {
        Altitude = atoi( NmeaGpsData.NmeaAltitude );
    }
    else
    {
        Altitude = ( int16_t )0xFFFF;
    }
    CRITICAL_SECTION_END( );

    return Altitude;
}

/*!
 * Calculates the checksum for a NMEA sentence
 *
 * Skip the first '$' if necessary and calculate checksum until '*' character is
 * reached (or buffSize exceeded).
 *
 * \retval chkPosIdx Position of the checksum in the sentence
 */
int32_t GpsNmeaChecksum( int8_t *nmeaStr, int32_t nmeaStrSize, int8_t * checksum )
{
    int i = 0;
    uint8_t checkNum = 0;

    // Check input parameters
    if( ( nmeaStr == NULL ) || ( checksum == NULL ) || ( nmeaStrSize <= 1 ) )
    {
        return -1;
    }

    // Skip the first '$' if necessary
    if( nmeaStr[i] == '$' )
    {
        i += 1;
    }

    // XOR until '*' or max length is reached
    while( nmeaStr[i] != '*' )
    {
        checkNum ^= nmeaStr[i];
        i += 1;
        if( i >= nmeaStrSize )
        {
            return -1;
        }
    }

    // Convert checksum value to 2 hexadecimal characters
    checksum[0] = Nibble2HexChar( checkNum / 16 ); // upper nibble
    checksum[1] = Nibble2HexChar( checkNum % 16 ); // lower nibble

    return i + 1;
}

/*!
 * Calculate the checksum of a NMEA frame and compare it to the checksum that is
 * present at the end of it.
 * Return true if it matches
 */
static bool GpsNmeaValidateChecksum( int8_t *serialBuff, int32_t buffSize )
{
    int32_t checksumIndex;
    int8_t checksum[2]; // 2 characters to calculate NMEA checksum

    checksumIndex = GpsNmeaChecksum( serialBuff, buffSize, checksum );

    // could we calculate a verification checksum ?
    if( checksumIndex < 0 )
    {
        return false;
    }

    // check if there are enough char in the serial buffer to read checksum
    if( checksumIndex >= ( buffSize - 2 ) )
    {
        return false;
    }

    // check the checksum
    if( ( serialBuff[checksumIndex] == checksum[0] ) && ( serialBuff[checksumIndex + 1] == checksum[1] ) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*
 *
 */


#include    <math.h>

extern volatile uint8_t gpsReportPending;

#define GPS_MAX_FIELDS  30

static char gpsBuffer[83];

static enum { GP_BEGIN, GP_COMMA, GP_CHECK } gpsState = GP_BEGIN;
static uint8_t gpsIndex;
static char *gpsField[GPS_MAX_FIELDS];
static char *gpsPtr = gpsBuffer;

uint8_t coords[18] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

static double lastLat, lastLng;
static bool lastPositionValid = false;

static double
degreesToRadians(double degrees)
{
    return ((degrees * M_PI) / 180.0);
}

static float
gpsDistance(double lat1, double lon1, double lat2, double lon2)
{
    const double earthRadiusKm = 6371.0;
    double dLat, dLon;
    double a, c;
    
    dLat = degreesToRadians(lat2 - lat1);
    dLon = degreesToRadians(lon2 - lon1);
    lat1 = degreesToRadians(lat1);
    lat2 = degreesToRadians(lat2);
    
    a = sin(dLat / 2.0) * sin(dLat / 2.0) +
      sin(dLon / 2.0) * sin(dLon / 2.0) * cos(lat1) * cos(lat2); 
    
    c = 2.0 * atan2(sqrt(a), sqrt(1-a));
    return (earthRadiusKm * c);
}

static double
scanDDMM(char *p)
{
    double v, d;

    v = strtod(p, NULL);
    d = floor(v / 100.0);
    v -= d * 100.0;
    d += v / 60.0;
    
    return (d);
}

static void
processGpsGGA(void)
{
    double lat, lon;
    float hdop, altitude;

    // memset(coords, 0xff, sizeof(coords));

    // GGA has 16 total fields
    // 2, 3: lat, 4, 5: long, 6: fix, 8: hdop, 9, 10: altitude
    if (gpsIndex < 15) {
        return;
    }

    if (atoi(gpsField[6]) < 2) {
        // not good enough fix
        return;
    }

    lat = scanDDMM(gpsField[2]);
    if (*gpsField[3] == 'S') {
        lat = -lat;
    }

    lon = scanDDMM(gpsField[4]);
    if (*gpsField[5] == 'W') {
        lon = -lon;
    }

    if (lastPositionValid && (gpsDistance(lat, lon, lastLat, lastLng) < 0.1)) {
        return;
    }

    hdop = strtof(gpsField[8], NULL);

    altitude = strtof(gpsField[9], NULL);

    *((int32_t *)(coords + 0)) = __builtin_bswap32( (int32_t)(lat * 1000000L) );
    *((int32_t *)(coords + 4)) = __builtin_bswap32( (int32_t)(lon * 1000000L) );

    altitude = roundf(altitude);
  
    *((uint16_t *)(coords + 8)) = __builtin_bswap16(4000); // mV
    coords[10] = 0x04; // FLAG, LED off, no movement mode, version 1.6.4
    *((uint16_t *)(coords + 11)) = __builtin_bswap16(0); // roll
    *((uint16_t *)(coords + 13)) = __builtin_bswap16(0); // pitch
    coords[15] = (int8_t)(hdop * 100.0);
    *((uint16_t *)(coords + 16)) = __builtin_bswap16((int16_t)(altitude * 100.0)); // altitude

    lastLat = lat;
    lastLng = lon;
    lastPositionValid = true;

    gpsReportPending = 1;
}

static void
processGpsGSA(void)
{

}

extern int LmHandlerJoinStatus(void);

static void
processGpsSentence(uint8_t check)
{
    uint8_t checkVal;

    checkVal = strtol(gpsField[gpsIndex], NULL, 16);
    if (checkVal != check || gpsIndex < 2) {
        // invalid sentence, return
        return;
    }

    if (strcmp(gpsField[0], "GPGGA") == 0) {
        processGpsGGA();
        return;
    } else if (strcmp(gpsField[0], "GPGSA") == 0) {
        processGpsGSA();
        return;
    }
}

void
processGpsChar(uint8_t c)
{
    static uint8_t checkVal = 0;
    static uint8_t gpsCount = 0;

    switch (gpsState) {
    case GP_BEGIN:
        if (c != '$') {
            break;
        }
        gpsIndex = 0;
        gpsPtr = gpsBuffer;
        gpsField[0] = gpsBuffer;
        gpsState = GP_COMMA;
        checkVal = 0;
        gpsCount = 0;
        break;

    case GP_COMMA:
        if (++gpsCount >= 82) {
            // invalid
            gpsState = GP_BEGIN;
            break;
        }

        if (c == ',' || c == '*') {
            *gpsPtr++ = '\0';
            if (c == '*') {
                gpsState = GP_CHECK;
            } else {
                checkVal ^= c;
            }
            gpsField[++gpsIndex] = gpsPtr;
            break;
        } else {
            *gpsPtr++ = c;
            checkVal ^= c;
            break;
        }
        break;

    case GP_CHECK:
        if (++gpsCount >= 82) {
            // invalid
            gpsState = GP_BEGIN;
            break;
        }

        if (c == '\n' || c == '\r') {
            *gpsPtr++ = '\0';
            processGpsSentence(checkVal);
            gpsState = GP_BEGIN;
        } else {
            *gpsPtr++ = c;
        }
        break;

    default:
        gpsState = GP_BEGIN;
        break;
    }

}


/*
 * XXX: old crap
 */

LmnStatus_t GpsParseGpsData( int8_t *rxBuffer, int32_t rxBufferSize )
{
    uint8_t i = 1;
    uint8_t j = 0;
    uint8_t fieldSize = 0;

    if( rxBuffer[0] != '$' )
    {
        GpsMcuInvertPpsTrigger( );
        return LMN_STATUS_ERROR;
    }

    if( GpsNmeaValidateChecksum( rxBuffer, rxBufferSize ) == false )
    {
        return LMN_STATUS_ERROR;
    }

    fieldSize = 0;
    while( rxBuffer[i + fieldSize++] != ',' )
    {
        if( fieldSize > 6 )
        {
            return LMN_STATUS_ERROR;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        NmeaGpsData.NmeaDataType[j] = rxBuffer[i];
    }
    // Parse the GPGGA data
    if( strncmp( ( const char* )NmeaGpsData.NmeaDataType, ( const char* )NmeaDataTypeGPGGA, 5 ) == 0 )
    {
        // NmeaUtcTime
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 11 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaUtcTime[j] = rxBuffer[i];
        }
        // NmeaLatitude
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 10 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaLatitude[j] = rxBuffer[i];
        }
        // NmeaLatitudePole
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 2 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaLatitudePole[j] = rxBuffer[i];
        }
        // NmeaLongitude
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 11 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaLongitude[j] = rxBuffer[i];
        }
        // NmeaLongitudePole
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 2 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaLongitudePole[j] = rxBuffer[i];
        }
        // NmeaFixQuality
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 2 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaFixQuality[j] = rxBuffer[i];
        }
        // NmeaSatelliteTracked
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 3 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaSatelliteTracked[j] = rxBuffer[i];
        }
        // NmeaHorizontalDilution
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 6 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaHorizontalDilution[j] = rxBuffer[i];
        }
        // NmeaAltitude
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 8 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaAltitude[j] = rxBuffer[i];
        }
        // NmeaAltitudeUnit
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 2 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaAltitudeUnit[j] = rxBuffer[i];
        }
        // NmeaHeightGeoid
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 8 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaHeightGeoid[j] = rxBuffer[i];
        }
        // NmeaHeightGeoidUnit
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 2 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaHeightGeoidUnit[j] = rxBuffer[i];
        }

        GpsFormatGpsData( );
        return LMN_STATUS_OK;
    }
    else if ( strncmp( ( const char* )NmeaGpsData.NmeaDataType, ( const char* )NmeaDataTypeGPRMC, 5 ) == 0 )
    {
        // NmeaUtcTime
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 11 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaUtcTime[j] = rxBuffer[i];
        }
        // NmeaDataStatus
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 2 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaDataStatus[j] = rxBuffer[i];
        }
        // NmeaLatitude
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 10 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaLatitude[j] = rxBuffer[i];
        }
        // NmeaLatitudePole
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 2 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaLatitudePole[j] = rxBuffer[i];
        }
        // NmeaLongitude
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 11 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaLongitude[j] = rxBuffer[i];
        }
        // NmeaLongitudePole
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 2 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaLongitudePole[j] = rxBuffer[i];
        }
        // NmeaSpeed
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 8 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaSpeed[j] = rxBuffer[i];
        }
        // NmeaDetectionAngle
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 8 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaDetectionAngle[j] = rxBuffer[i];
        }
        // NmeaDate
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > 8 )
            {
                return LMN_STATUS_ERROR;
            }
        }
        for( j = 0; j < fieldSize; j++, i++ )
        {
            NmeaGpsData.NmeaDate[j] = rxBuffer[i];
        }

        GpsFormatGpsData( );
        return LMN_STATUS_OK;
    }
    else
    {
        return LMN_STATUS_ERROR;
    }
}

void GpsFormatGpsData( void )
{
    if( strncmp( ( const char* )NmeaGpsData.NmeaDataType, ( const char* )NmeaDataTypeGPGGA, 5 ) == 0 )
    {
        HasFix = ( NmeaGpsData.NmeaFixQuality[0] > 0x30 ) ? true : false;
    }
    else if ( strncmp( ( const char* )NmeaGpsData.NmeaDataType, ( const char* )NmeaDataTypeGPRMC, 5 ) == 0 )
    {
        HasFix = ( NmeaGpsData.NmeaDataStatus[0] == 0x41 ) ? true : false;
    }
    GpsConvertPositionFromStringToNumerical( );
    GpsConvertPositionIntoBinary( );
}

void GpsResetPosition( void )
{
    Altitude = ( int16_t )0xFFFF;
    Latitude = 0;
    Longitude = 0;
    LatitudeBinary = 0;
    LongitudeBinary = 0;
}
