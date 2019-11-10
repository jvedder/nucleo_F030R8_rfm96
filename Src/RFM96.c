/*
 * RFM96.c
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "RFM96.h"
#include <string.h>

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
/* User code -----------------------------------------------------------------*/

uint8_t RFM96_Read( uint8_t reg )
{
	HAL_StatusTypeDef status;

	// clear reg msb for read
	reg &= 0x7f;

	// buffers to transmit/receive
	uint8_t txData[] = {reg, 0x00};
	uint8_t rxData[] = {0x00, 0x00};
	const uint16_t	size = sizeof(txData);

	// default data value if error
	uint8_t data = 0x00;

	// Set CS low (active)
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

	// write 8 bit reg and read 8 bit data
	status = HAL_SPI_TransmitReceive(&hspi2, txData, rxData, size, TIMEOUT_1_SEC);

	if (status == HAL_OK)
	{
		//second byte is the register value
		data = rxData[1];
	}
	else
	{
		HAL_UART_Transmit(&huart2, (uint8_t *) &"*HAL_ERROR*\r\n", 13, TIMEOUT_1_SEC);
		// handle errors here
	}

	  sprintf(msg, "RFM96 RD 0x%02X=0x%02X\r\n", (int) reg, (int) data );
	  msg_size = strlen(msg);
	  HAL_UART_Transmit(&huart2, (uint8_t *) msg, msg_size, TIMEOUT_1_SEC);


	// Set CS high (inactive)
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

	return data;
}

void RFM96_Write( uint8_t reg, uint8_t data )
{
	HAL_StatusTypeDef status;

	  sprintf(msg, "RFM96 WR 0x%02X=0x%02X\r\n", (int) reg, (int) data );
	  msg_size = strlen(msg);
	  HAL_UART_Transmit(&huart2, (uint8_t *) msg, msg_size, TIMEOUT_1_SEC);

	//set the reg msb for write
	reg |= 0x80;

	// Transmit buffer
	uint8_t txData[2] = {reg, data};
	const uint16_t size = sizeof(txData);


	// Set CS low (active)
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

	// write 8 bit reg and read 8 bit data
	status = HAL_SPI_Transmit(&hspi2, txData, size, TIMEOUT_1_SEC);

	if (status != HAL_OK)
	{
		HAL_UART_Transmit(&huart2, (uint8_t *) &"*HAL_ERROR*\r\n", 13, TIMEOUT_1_SEC);
		// handle errors here
	}

	//HACK: Wait for SPI transfer to complete
	Delay_ms(1);
	// Set CS high (inactive)
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}


void Delay_ms( int n )
{
	const uint32_t TIMEOUT = 9600; // Estimated for 1 ms

	while(n--)
	{
		volatile uint32_t tick = TIMEOUT;
		while(tick--);
	}

	return;
}


