/*
 * gps_parse.c
 *
 *  Created on: 20 Mar 2023
 *      Author: sahin
 */
/*

#include "stm32f4xx.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define BUFFER_SIZE 256
#define DELIM ",*"

uint8_t rx_buffer[BUFFER_SIZE];
uint8_t tx_buffer[BUFFER_SIZE];
uint8_t rx_index = 0;

void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        rx_buffer[rx_index++] = USART_ReceiveData(USART2);
        if (rx_index == BUFFER_SIZE) {
            rx_index = 0;
        }
    }
}

int main(void) {
    // Initialize USART2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART2_IRQn);

    USART_Cmd(USART2, ENABLE);

    // Main loop
    while (1) {
        if (rx_index > 0) {
            // Check if we have a complete GPGGA message
            uint8_t* gpgga = strstr(rx_buffer, "$GPGGA");
            if (gpgga != NULL) {
                uint8_t* end = strstr(gpgga, "\r\n");
                if (end != NULL) {
                    uint8_t* token = strtok(gpgga, DELIM);
                    uint8_t i = 0;
                    while (token != NULL) {
                        if (i == 2) {
                            // latitude
                            float lat = atof(token);
                            printf("Latitude: %f\n", lat);
                        }
                        else if (i == 4) {
                            // longitude
                            float lon = atof(token);
                            printf("Longitude: %f\n", lon);
                        }
                        else if (i == 6) {
                            // fix quality
                            uint8_t fix = atoi(token);
                            printf("Fix quality: %d\n", fix);
                        }
                        else if (i == 9) {
                            // altitude
                            float alt = atof(token);
                            printf("Altitude: %f\n", alt);
                        }
                        token = strtok(NULL, DELIM);
                        i++;
                    }
                    // Clear the receive buffer
                    memset(rx_buffer, 0, sizeof(rx_buffer));
                    rx_index = 0;
                }
            }
        }
*/
