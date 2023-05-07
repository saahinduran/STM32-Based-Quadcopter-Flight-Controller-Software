/*
 * gps.c
 *
 *  Created on: Mar 18, 2023
 *      Author: sahin
 */
#include "main.h"
#include <stdlib.h>
#include "math.h"
struct Position{
	double lon;
	double lat;
};

extern UART_HandleTypeDef huart6;
uint8_t disable_GLL[]={0xB5 ,0x62 ,0x06 ,0x01 ,0x03 ,0x00 ,0xF0 ,0x01 ,0x00 ,0xFB ,0x11};
uint8_t disable_GSA[]={0xB5, 0x62, 0x06 ,0x01 ,0x03, 0x00, 0xF0 ,0x02 ,0x00 ,0xFC, 0x13};
uint8_t disable_GSV[]={0xB5, 0x62 ,0x06, 0x01, 0x03 ,0x00 ,0xF0, 0x03, 0x00 ,0xFD ,0x15};
uint8_t disable_RMC[]={0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0 ,0x04 ,0x00 ,0xFE, 0x17};
uint8_t disable_VTG[]={0xB5, 0x62 ,0x06 ,0x01 ,0x03 ,0x00 ,0xF0 ,0x05 ,0x00 ,0xFF ,0x19};
uint8_t disable_GGA[]={0xB5, 0x62 ,0x06 ,0x01 ,0x03 ,0x00 ,0xF0 ,0x00 ,0x00 ,0xFA ,0x0F};
uint8_t enable_UBX[]={0xB5, 0x62 , 0x06, 0x01 ,0x03 ,0x00 ,0x01 ,0x07, 0x01,0x13 ,0x51};
uint8_t uart_115200[]={0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E};
uint8_t HZ_10[]={0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01 ,0x00, 0x7A, 0x12};
extern char *message;
extern char *Lat[2];
extern char *Lon[2];
extern char *Lat_1;
extern char *Lat_2;
extern char *Lon_1;
extern char *Lon_2;
extern char *delp;
extern double latitude,longitude;
extern int gps_connected;
extern uint8_t gps_buffer[100];


////// THIS FUNCTION IS TO DECODE NMEA MESSAGES, HOWEVER I USED UBX MESSAGES TO DECODE LATITUDE AND LONGITUDE
/*
int gps_decode(uint8_t *message,double*latitude,double *longitude){

	if(message[0]!='$'){
		return 1;

	}
	//"$GNGGA,123519,4807.038,N,2113.000,E,1,08,0.9,545.4,M,46.9,M,,*42";
	else{
		delp=strtok(message,",");
				while(delp!=NULL){
					if(cnt_gps==2){
						Lat[0]=delp;
					}
					if(cnt_gps==4) Lon[0]=delp;

					delp=strtok(NULL,",");
					cnt_gps++;
				}
				cnt_gps=0;


				*latitude=strtod(Lat[0],NULL);
				*longitude=strtod(Lon[0],NULL);
				*latitude=(int) *latitude/100 +(*latitude -(int) (*latitude/100)*100)/60;
				*longitude=(int) *longitude/100 +(*longitude -(int) (*longitude/100)*100)/60;
				return 0;
	}


}
*/

int gps_decode(uint8_t *message,uint32_t*latitude,uint32_t *longitude){

	*latitude=(message[30])+(message[31]<<8)+(message[32]<<16)+(message[33]<<24);
	*longitude=(message[34])+(message[35]<<8)+(message[36]<<16)+(message[37]<<24);

	if(message[0]==181 && message[1]==98){  // ARE THE FIRST TWO BYTES CORRECT ?
		if(message[26]==3){  // IF SO, IS GPS FIXED ?
			return 0;
		}
		else
			return 1;
	}

}


int gps_init(){  // connect to gps and set parameters accordingly
	HAL_UART_Receive_DMA(&huart6, (uint8_t *)gps_buffer,100);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart6, disable_GLL, sizeof(disable_GLL), 100);
	HAL_Delay(5);
	HAL_UART_Transmit(&huart6, disable_RMC, sizeof(disable_RMC), 100);
	HAL_Delay(5);
	HAL_UART_Transmit(&huart6, disable_GSV, sizeof(disable_GSV), 100);
	HAL_Delay(5);
	HAL_UART_Transmit(&huart6, disable_GSA, sizeof(disable_GSA), 100);
	HAL_Delay(5);
	HAL_UART_Transmit(&huart6, disable_VTG, sizeof(disable_VTG), 100);
	HAL_Delay(5);
	HAL_UART_Transmit(&huart6, disable_GGA, sizeof(disable_GGA), 100);
	HAL_Delay(5);
	HAL_UART_Transmit(&huart6, HZ_10, sizeof(HZ_10), 100);
	HAL_Delay(5);
	HAL_UART_Transmit(&huart6, enable_UBX, sizeof(enable_UBX), 100);
	HAL_Delay(5);
	HAL_UART_Transmit(&huart6, uart_115200, sizeof(uart_115200), 100);
	HAL_Delay(300);
	huart6.Init.BaudRate=115200;
	HAL_UART_Init(&huart6);
	HAL_UART_Receive_DMA(&huart6, (uint8_t *)gps_buffer,100);
	HAL_Delay(5);
	while(gps_connected<999999){  // wait until frame fits into 100 bytes buffer
		if(gps_buffer[0]!=181){
			HAL_UART_DMAStop(&huart6);
			HAL_Delay(10);
			HAL_UART_DeInit(&huart6);
			HAL_Delay(10);
			HAL_UART_Init(&huart6);
			HAL_Delay(10);
			HAL_UART_Receive_DMA(&huart6, (uint8_t *)gps_buffer,100);
			HAL_Delay(5);
			//gps_connected=0;
		}
		else if (gps_buffer[0]==181){
			gps_connected+=1;
		}
	}
	return 0;
}

int is_gps_connected(){ // additional frame check function
						// this function is needed because after init process of gps
						// it sends acknowledge of init process functions that will cause frame sync error
	gps_connected=0;
	while(gps_connected<99999){
			if(gps_buffer[0]!=181){
				HAL_UART_DMAStop(&huart6);
				HAL_Delay(10);
				HAL_UART_DeInit(&huart6);
				HAL_Delay(10);
				HAL_UART_Init(&huart6);
				HAL_Delay(10);
				HAL_UART_Receive_DMA(&huart6, (uint8_t *)gps_buffer,100);
				HAL_Delay(5);
				//gps_connected=0;
			}
			else if (gps_buffer[0]==181){
				gps_connected+=1;
			}
		}
		return 0;

}

void calc_gps(double x1,double y1,double x2,double y2,double *result){ // this function calculates the distance and bearing angle between two geographical positions
		struct Position myPosition;
		struct Position arr_Position;


		myPosition.lat=x1;
		myPosition.lon=y1;
		arr_Position.lat=x2;
		arr_Position.lon=y2;
		//printf("First lat and long respectively %lf, %lf \n",myPosition.lat,myPosition.lon);
		//printf("Second lat and long respectively %lf, %lf \n",arr_Position.lat,arr_Position.lon);
		double R=6372795.477598;
		double distance=R*acos(sin(myPosition.lat*M_PI/180)*sin(arr_Position.lat*M_PI/180)+cos(myPosition.lat*M_PI/180)*cos(arr_Position.lat*M_PI/180)*cos((myPosition.lon-arr_Position.lon)*M_PI/180));
		//printf("Distance is: %lf\n",distance);


		double latB=arr_Position.lat;
		double latA=myPosition.lat;
		double dfi=log(tan(M_PI/180*latB/2+M_PI/4)/tan(M_PI/180*latA/2+M_PI/4));
		//printf("First half result:%lf\n",tan(latB/2*M_PI/180+M_PI/4));
		//printf("Second half result:%lf\n",tan(latA/2*M_PI/180+M_PI/4));
		//printf("Dfi result is:%.15lf\n",dfi);


		double lonA=myPosition.lon;
		double lonB=arr_Position.lon;
		double dlon=(M_PI/180*lonA-M_PI/180*lonB);

		//printf("Dlon result: %lf\n",dlon);

		double angle=2*M_PI-atan2(dlon,dfi);

		if (angle>2*M_PI){
		angle=angle-2*M_PI;
		}
		result[0]=distance;
		result[1]=angle*180/M_PI;

}
