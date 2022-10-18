/*
 * zoe_m8q.c
 *
 *  Created on: Jul 2, 2021
 *      Author: erdem
 */

//Bu script Ublox Zoe M8Q haberleşmesi amaçlı yazılmıştır.
//İstek datası POSLLH 0xB5,0x62,0x01,0x02,0x00,0x00,0x03,0xA'dır.

#include"zoe_m8q.h"
#include"stm32f7xx.h"
#include"stm32f7xx_hal.h"
#include"math.h"
int i,j;
uint8_t request_posllh_array[8] = {0xB5,0x62,0x01,0x02,0x00,0x00,0x03,0x0A};
uint8_t request_velned_array[8] = {0xB5,0x62,0x01,0x12,0x00,0x00,0x13,0x3A};

zoe_m8q zoe;

extern UART_HandleTypeDef huart1;

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_14);  // toggle PA0

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_DMA(&huart1, zoe.rx_zoe_data, 80);

}

void POLL_WDMA_ZOE(void)
{
	HAL_UART_Receive_DMA (&huart1, zoe.rx_zoe_data, 80);
}

void zoe_llhvelned_request(void)
{
	HAL_UART_Transmit(&huart1, request_posllh_array , 8, 1000);
	HAL_UART_Transmit(&huart1, request_velned_array , 8, 1000);
}

void zoe_parse_data(void)
{


		if(zoe.rx_zoe_data[0] == 181 &&
			zoe.rx_zoe_data[1] == 98 &&
			zoe.rx_zoe_data[2] == 1 &&
			zoe.rx_zoe_data[3] == 2)
	{
	  zoe.lattitude_zoe_m8q = zoe.rx_zoe_data[10] 	    |
				  	  	  	 (zoe.rx_zoe_data[11] << 8) |
				  	  	  	 (zoe.rx_zoe_data[12] << 16)|
							 (zoe.rx_zoe_data[13] << 24);

	  zoe.longtitude_zoe_m8q= zoe.rx_zoe_data[14] 	    |
						  	 (zoe.rx_zoe_data[15] << 8) |
						  	 (zoe.rx_zoe_data[16] << 16)|
							 (zoe.rx_zoe_data[17] << 24);

	  zoe.altitude_zoe_m8q  = zoe.rx_zoe_data[22] 	    |
						  	 (zoe.rx_zoe_data[23] << 8) |
						  	 (zoe.rx_zoe_data[24] << 16)|
							 (zoe.rx_zoe_data[25] << 24);
	}

		 if(zoe.rx_zoe_data[0+36] == 181 &&
			        zoe.rx_zoe_data[1+36] == 98 &&
					zoe.rx_zoe_data[2+36] == 1 &&
					zoe.rx_zoe_data[3+36] == 18)
	{
	  zoe.velocity_north    =  zoe.rx_zoe_data[10+36] 	    |
				  	  	  	  (zoe.rx_zoe_data[11+36] << 8) |
				  	  	  	  (zoe.rx_zoe_data[12+36] << 16)|
							  (zoe.rx_zoe_data[13+36] << 24);

	  zoe.velocity_east     =  zoe.rx_zoe_data[14+36] 	    |
						  	  (zoe.rx_zoe_data[15+36] << 8) |
						  	  (zoe.rx_zoe_data[16+36] << 16)|
							  (zoe.rx_zoe_data[17+36] << 24);

	  zoe.velocity_down     =  zoe.rx_zoe_data[18+36] 	    |
			  	  	  	  	  (zoe.rx_zoe_data[19+36] << 8) |
							  (zoe.rx_zoe_data[20+36] << 16)|
							  (zoe.rx_zoe_data[21+36] << 24);

	  zoe.speed_3d			=  zoe.rx_zoe_data[22+36] 	    |
			  	  	  	  	  (zoe.rx_zoe_data[23+36] << 8) |
							  (zoe.rx_zoe_data[24+36] << 16)|
							  (zoe.rx_zoe_data[25+36] << 24);

	  zoe.speed_gnd			=  zoe.rx_zoe_data[26+36] 	    |
			  	  	  	  	  (zoe.rx_zoe_data[27+36] << 8) |
							  (zoe.rx_zoe_data[28+36] << 16)|
							  (zoe.rx_zoe_data[29+36] << 24);
	}


}

void ZOE(void)
{
	  zoe_llhvelned_request();
	  zoe_parse_data();
}
