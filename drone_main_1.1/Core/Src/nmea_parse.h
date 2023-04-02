//
// Created by sztuka on 22.01.2023.
//

#ifndef STM32_SERIAL_DMA_NEO6M_PARSE_H
#define STM32_SERIAL_DMA_NEO6M_PARSE_H

#endif //STM32_SERIAL_DMA_NEO6M_PARSE_H
#include <stm32f4xx.h>
#include <string.h>
#include <stdlib.h>


/*******************************************************************************
 * @brief GPS data structure for storing parsed data
 ******************************************************************************/
typedef struct NMEA_DATA {
    double latitude; //latitude in degrees with decimal places
    char latSide;  // N or S
    double longitude; //longitude in degrees with decimal places
    char lonSide; // E or W
    float altitude; //altitude in meters
    float hdop; //horizontal dilution of precision
    int satelliteCount; //number of satellites used in measurement
    int fix; // 1 = fix, 0 = no fix
    char lastMeasure[10]; // hhmmss.ss UTC of last successful measurement; time read from the GPS module
} GPS;
/*******************************************************************************
 * @brief Parses NMEA data from the GPS module
 * @param gps_data Pointer to GPS struct, writes data to it
 * @param buffer Pointer to buffer string with NMEA data
 ******************************************************************************/
void nmea_parse(GPS *gps_data, uint8_t *buffer);