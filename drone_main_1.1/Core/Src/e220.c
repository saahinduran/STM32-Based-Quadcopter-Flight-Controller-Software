/*
 * e220.c
 *
 *  Created on: Feb 18, 2023
 *      Author: sahin
 */


#include "main.h"
#include "e220.h"

extern UART_HandleTypeDef huart7;

void E220_init_declare_pins(struct LoRa_Handler *LoRa, GPIO_TypeDef *M_GPIO_PORT,
GPIO_TypeDef *AUX_GPIO_PORT, uint16_t M0_PIN,uint16_t M1_PIN,uint16_t MAUX_PIN,
UART_HandleTypeDef *uart_handler)
{
	LoRa->uart_handler=uart_handler;
	LoRa->AUX_GPIO_PORT=AUX_GPIO_PORT;
	LoRa->M_GPIO_PORT=M_GPIO_PORT;
	LoRa->M0_PIN=M0_PIN;
	LoRa->M1_PIN=M1_PIN;
	LoRa->MAUX_PIN=MAUX_PIN;
}



int8_t E220_enter_config_mode(struct LoRa_Handler *LoRa)
{

	HAL_GPIO_WritePin(LoRa->M_GPIO_PORT, LoRa->M0_PIN, RESET);
	HAL_GPIO_WritePin(LoRa->M_GPIO_PORT, LoRa->M1_PIN, RESET);

	HAL_Delay(3);

	HAL_GPIO_WritePin(LoRa->M_GPIO_PORT, LoRa->M0_PIN, SET);
	HAL_GPIO_WritePin(LoRa->M_GPIO_PORT, LoRa->M1_PIN, SET);

	HAL_Delay(50);

	uint8_t aux_status=HAL_GPIO_ReadPin(LoRa->AUX_GPIO_PORT, LoRa->MAUX_PIN);

	return aux_status;


}



int8_t E220_read_register(struct LoRa_Handler *LoRa,uint8_t reg)
{


	uint8_t send_data[3]={CMD_READ_REG,reg,1};
	uint8_t receive_data[4]={0};
	HAL_UART_Transmit(LoRa->uart_handler,send_data ,3, 100);

	while(HAL_GPIO_ReadPin(LoRa->AUX_GPIO_PORT, LoRa->MAUX_PIN)==0);

	HAL_UART_Receive(LoRa->uart_handler, receive_data, 4, 100);

	if(receive_data[0]==CMD_READ_REG && receive_data[1]==reg && receive_data[2]==1)
		return receive_data[3];
	else
		return -1;
}

int8_t E220_write_register(struct LoRa_Handler *LoRa,uint8_t reg,uint8_t parameter)
{

	HAL_GPIO_WritePin(LoRa->M_GPIO_PORT, LoRa->M0_PIN, SET);
	HAL_GPIO_WritePin(LoRa->M_GPIO_PORT, LoRa->M1_PIN, SET);

	HAL_Delay(3);

	uint8_t send_data[4]={CMD_SET_REG,reg,1,parameter};
	uint8_t receive_data[4]={0};

	HAL_UART_Transmit(LoRa->uart_handler,send_data ,4, 100);
	HAL_UART_Receive(LoRa->uart_handler, receive_data, 4, 100);


	if(receive_data[0]==CMD_READ_REG && receive_data[1]==reg && receive_data[2]==1 && receive_data[3] == parameter)
		return receive_data[3];
	else
		return -1;

}

int8_t E220_read_register_all(struct LoRa_Handler *LoRa,uint8_t *data)
{

	for(int i=0; i<8;i++){
		HAL_Delay(2);
		data[i]=E220_read_register(LoRa,i);

		if((int8_t)data[i]==(int8_t)-1)
			return -1;
	}
	return 1;



}

int8_t E220_enter_normal_mode(struct LoRa_Handler *LoRa)
{
	HAL_GPIO_WritePin(LoRa->M_GPIO_PORT, LoRa->M0_PIN, RESET);
	HAL_GPIO_WritePin(LoRa->M_GPIO_PORT, LoRa->M1_PIN, RESET);
	while(HAL_GPIO_ReadPin(LoRa->AUX_GPIO_PORT, LoRa->MAUX_PIN)==0);
	return 1;


}


int8_t E220_transmit_payload(struct LoRa_Handler *LoRa,uint8_t *data,uint8_t datasize)
{
	HAL_UART_Transmit(LoRa->uart_handler,data ,datasize, 100);
	return 1;
}

int8_t E220_transmit_payload_DMA(struct LoRa_Handler *LoRa,uint8_t *data,uint8_t datasize)
{
	HAL_UART_Transmit_DMA(LoRa->uart_handler, data, datasize);
	return 1;
}

int8_t E220_transmit_payload_DMA_v2(struct LoRa_Handler *LoRa,uint8_t *data,uint8_t datasize)
{
	HAL_UART_Transmit_DMA(LoRa->uart_handler, data, datasize);
	return 1;
}




int8_t E220_receive_payload(struct LoRa_Handler *LoRa,uint8_t *data,uint8_t datasize)
{
	//while(HAL_GPIO_ReadPin(LoRa->AUX_GPIO_PORT, LoRa->MAUX_PIN)==0);
	HAL_UART_Receive(LoRa->uart_handler, data, datasize, 100);
	return 1;

}

int8_t E220_receive_payload_DMA(struct LoRa_Handler *LoRa,uint8_t *data,uint8_t datasize)
{
	//while(HAL_GPIO_ReadPin(LoRa->AUX_GPIO_PORT, LoRa->MAUX_PIN)==0);
	HAL_UART_Receive_DMA(LoRa->uart_handler, data, datasize);
	return 1;

}


int8_t E220_set_packetsize_32k(struct LoRa_Handler *LoRa)
{
	uint8_t data=E220_read_register(LoRa,REG1);
	E220_write_register(LoRa,REG1,data | 0b11<<6);

	if((uint8_t)E220_read_register(LoRa,REG1)==(data | 0b11<<6)) return 1;
	else return -1;
}

int8_t E220_set_datarate_62k(struct LoRa_Handler *LoRa)
{
	uint8_t data=E220_read_register(LoRa,REG0);
	E220_write_register(LoRa,REG0,data | 0b111);

	if((uint8_t)E220_read_register(LoRa,REG0)==(data | 0b111)) return 1;
	else return -1;
}

void E220_reset(struct LoRa_Handler *LoRa)
{

	E220_write_register(LoRa,0x0,0);
	E220_write_register(LoRa,0x1,0);
	E220_write_register(LoRa,0x2,98);
	E220_write_register(LoRa,0x3,0);
	E220_write_register(LoRa,0x4,15);
	E220_write_register(LoRa,0x5,0);
	E220_write_register(LoRa,0x6,0);
	E220_write_register(LoRa,0x7,0);

}





