/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
uint8_t uartRxBuffer[3] = {0, 0, 0};
int screen_buffer[3] = {-48, -48, -48};
extern uint8_t uartRx;
extern float temperature;
extern uint8_t screen;
char screen_message[150];
uint8_t fan_speed = 50;
uint8_t idx = 0;
uint8_t PVT = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart5;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles UART5 global interrupt.
*/
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */
  /* USER CODE END UART5_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void clearBuffer() {
	for (int i=0;i<sizeof(screen_message);i++){
		screen_message[i] = 0;
	}
}

/*
 *Uart interrupt callback routine.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
		switch (screen) {
			case 0:
				if (uartRx == '1')
					screen = 1;
				else if (uartRx == '2')
					screen = 2;
				else if (uartRx == '3')
					screen = 3;
				else
					screen = 0;
				break;
			case 1:
				if (uartRx == '0')
					screen = 0;
				else if (uartRx == '2') 
					screen = 2;
				else if (uartRx == '3') 
					screen = 3;
				break;
			case 2:
				if (idx == 0 || idx == 1) {
					screen = 2;
					uartRxBuffer[idx] = uartRx-48;
					screen_buffer[idx]= uartRx-48;
					idx ++;
				} else if(idx == 2) {
					PVT = uartRxBuffer[0]*100 + uartRxBuffer[1]*10 + uartRxBuffer[2];
					idx = 0;
					screen = 1;
					uartRxBuffer[0] = 0;
					uartRxBuffer[1] = 0;
					uartRxBuffer[2] = 0;
					screen_buffer[0] = -48;
					screen_buffer[1] = -48;
					screen_buffer[2] = -48;
				}
				break;
			case 3:
				if (idx == 0 || idx == 1) {
					screen = 3;
					uartRxBuffer[idx] = uartRx-48;
					screen_buffer[idx]= uartRx-48;
					idx ++;
				} else if(idx == 2) {
					fan_speed = uartRxBuffer[0]*100 + uartRxBuffer[1]*10 + uartRxBuffer[2];
					if (fan_speed > 100)
						fan_speed = 100;
					idx = 0;
					screen = 1;
					uartRxBuffer[0] = 0;
					uartRxBuffer[1] = 0;
					uartRxBuffer[2] = 0;
					screen_buffer[0] = -48;
					screen_buffer[1] = -48;
					screen_buffer[2] = -48;
				}
				break;
				
		}//END CASE.
		HAL_UART_Receive_IT(&huart5, &uartRx, sizeof(uartRx));
  }//END UART5 INTERRUPT.
}
/*
 *Timer interrupt callback routine.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){

	if (htim->Instance == TIM6){
		switch (screen) {
			case 0:
				clearBuffer();	
				sprintf(screen_message, "Select an operation                                                   \r");
				HAL_UART_Transmit_IT(&huart5, (uint8_t*)screen_message, sizeof(screen_message));
				break;
			case 1:
				clearBuffer();
				sprintf(screen_message, "Temperature : %f [Celsus] Fan speed: %d [percentage]                   \r", temperature, fan_speed);
				HAL_UART_Transmit_IT(&huart5, (uint8_t*)screen_message, sizeof(screen_message));
				break;
		  case 2:
				clearBuffer();
				sprintf(screen_message, "\r Insert temperature to reach [000 - 300] %c%c%c                      \r", screen_buffer[0]+48,screen_buffer[1]+48, screen_buffer[2]+48);
				HAL_UART_Transmit_IT(&huart5, (uint8_t*)screen_message, sizeof(screen_message));
				break;
			case 3:
				clearBuffer();
				sprintf(screen_message, "Insert fan speed [000 - 100] %c%c%c                                    \r", screen_buffer[0]+48,screen_buffer[1]+48, screen_buffer[2]+48);
				HAL_UART_Transmit_IT(&huart5, (uint8_t*)screen_message, sizeof(screen_message));
				break;
		}
	}
	HAL_TIM_Base_Start_IT(htim);
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
