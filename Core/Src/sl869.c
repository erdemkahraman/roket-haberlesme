/*
 * sl869.c
 *
 *  Created on: Jul 2, 2021
 *      Author: erdem
 */


#include"sl869.h"
#include"stm32f7xx.h"
#include"stm32f7xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

UART_HandleTypeDef huart2;

GPS_t GPS;

void	POLL_SL869(void)
{

		HAL_UART_Receive_DMA (&huart2, GPS.rxBuffer, 512);


}
//##################################################################################################################
void	GPS_CallBack(void)
{
	GPS.LastTime=HAL_GetTick();
	if(GPS.rxIndex < sizeof(GPS.rxBuffer)-2)
	{
		GPS.rxBuffer[GPS.rxIndex] = GPS.rxTmp;
		GPS.rxIndex++;
	}
	HAL_UART_Receive_IT(&huart2,&GPS.rxTmp,1);
}
