/*
 * gps.h
 *
 *  Created on: Mar 18, 2023
 *      Author: sahin
 */

#ifndef SRC_GPS_H_
#define SRC_GPS_H_

int gps_decode(uint8_t *message,uint32_t*latitude,uint32_t *longitude);
int gps_init();
int is_gps_connected();
void calc_gps(double x1,double y1,double x2,double y2,double *result);
#include "main.h"
#include <stdlib.h>
//uint8_t disable_GLL[]={0xB5 ,0x62 ,0x06 ,0x01 ,0x03 ,0x00 ,0xF0 ,0x01 ,0x00 ,0xFB ,0x11};
//uint8_t disable_GSA[]={0xB5, 0x62, 0x06 ,0x01 ,0x03, 0x00, 0xF0 ,0x02 ,0x00 ,0xFC, 0x13};
//uint8_t disable_GSV[]={0xB5, 0x62 ,0x06, 0x01, 0x03 ,0x00 ,0xF0, 0x03, 0x00 ,0xFD ,0x15};
//uint8_t disable_RMC[]={0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0 ,0x04 ,0x00 ,0xFE, 0x17};
//uint8_t disable_VTG[]={0xB5, 0x62 ,0x06 ,0x01 ,0x03 ,0x00 ,0xF0 ,0x05 ,0x00 ,0xFF ,0x19};
//uint8_t disable_GGA[]={0xB5, 0x62 ,0x06 ,0x01 ,0x03 ,0x00 ,0xF0 ,0x00 ,0x00 ,0xFA ,0x0F};
//uint8_t enable_UBX[]={0xB5, 0x62 , 0x06, 0x01 ,0x03 ,0x00 ,0x01 ,0x07, 0x01,0x13 ,0x51};
//uint8_t uart_115200[]={0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E};
//uint8_t HZ_10[]={0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01 ,0x00, 0x7A, 0x12};
//B5 62 06 01 03 00 01 07 01 13 51
char message[] = "$GNGGA,123519,4807.038,N,2113.000,E,1,08,0.9,545.4,M,46.9,M,,*42";
char *Lat[2];
char *Lon[2];
char *Lat_1;
char *Lat_2;
char *Lon_1;
char *Lon_2;
char *delp;
uint32_t latitude,longitude;
int cnt_gps=0;




#endif /* SRC_GPS_H_ */
