/*
 * e220.h
 *
 *  Created on: Feb 18, 2023
 *      Author: sahin
 */

#ifndef SRC_E220_H_
#define SRC_E220_H_


#include "main.h"


struct LoRa_Handler{
	UART_HandleTypeDef *uart_handler;
	GPIO_TypeDef *M_GPIO_PORT;
	GPIO_TypeDef *AUX_GPIO_PORT;
	uint16_t M0_PIN;
	uint16_t M1_PIN;
	uint16_t MAUX_PIN;

};

#define CMD_SET_REG 0xC0 // COMMAND FOR SETTING REGISTER
#define CMD_READ_REG 0xC1 // COMMAND FOR READING REGISTER
#define REG_ADD_H 0x0 // DEVICE ADDRESS HIGH BYTE
#define REG_ADD_L 0x1 // DEVICE ADDRESS LOW BYTE
#define REG0 0x2 // UART CONFIGURATION REGISTER
#define REG1 0x3 // RF CONFIGURATION REGISTER
#define REG2 0x4 // CHANNEL CONTROL
#define REG3 0x5 // TRANSMISSION PARAMETER CONTROL
#define KEY_H 0x6 // PASSWORD KEY HIGH BYTE
#define KEY_L 0x7 // PASSWORD KEY LOW BYTE

int8_t E220_read_register(struct LoRa_Handler *LoRa,uint8_t reg);
int8_t E220_write_register(struct LoRa_Handler *LoRa,uint8_t reg,uint8_t parameter);
int8_t E220_enter_config_mode(struct LoRa_Handler *LoRa);
int8_t E220_read_register_all(struct LoRa_Handler *LoRa,uint8_t *data);
int8_t E220_enter_normal_mode(struct LoRa_Handler *LoRa);
int8_t E220_transmit_payload(struct LoRa_Handler *LoRa,uint8_t *data,uint8_t datasize);
int8_t E220_transmit_payload_DMA(struct LoRa_Handler *LoRa,uint8_t *data,uint8_t datasize);
int8_t E220_transmit_payload_DMA_v2(struct LoRa_Handler *LoRa,uint8_t *data,uint8_t datasize);
int8_t E220_receive_payload(struct LoRa_Handler *LoRa,uint8_t *data,uint8_t datasize);
int8_t E220_receive_payload_DMA(struct LoRa_Handler *LoRa,uint8_t *data,uint8_t datasize);
int8_t E220_set_packetsize_32k(struct LoRa_Handler *LoRa);
int8_t E220_set_datarate_62k(struct LoRa_Handler *LoRa);
void E220_init_declare_pins(struct LoRa_Handler *LoRa, GPIO_TypeDef *M_GPIO_PORT,
GPIO_TypeDef *AUX_GPIO_PORT, uint16_t M0_PIN,uint16_t M1_PIN,uint16_t MAUX_PIN,
UART_HandleTypeDef *uart_handler);

void E220_reset(struct LoRa_Handler *LoRa);


#endif /* SRC_E220_H_ */
