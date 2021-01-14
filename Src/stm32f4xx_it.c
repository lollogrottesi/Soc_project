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
extern float duty;
char screen_message[150];
uint8_t tmp_fan_speed = 0;
uint8_t fan_speed = 0;
uint8_t idx = 0;
uint16_t PVT = 0;
uint16_t tmp_PVT = 0;
uint16_t usrTime = 0;
uint16_t tmp_usrTime = 0;
uint8_t flag_usrTime = 0;
uint32_t time_cnt = 0;
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
  if (huart->Instance == UART5)//UART 5 interrupt routine.
  {
		switch (screen) {
			case 0:	//Welcome screen.
				if (uartRx == '1')
					screen = 1;
				else if (uartRx == '2')
					screen = 2;
				else if (uartRx == '3')
					screen = 3;
				else if (uartRx == '4')
					screen = 7;
				else if (uartRx == '5')
					screen = 9;
				else
					screen = 0;
				break;
			case 1://Show Temperature and Fan screen.
				if (uartRx == '0')
					screen = 0;
				else if (uartRx == '2') 
					screen = 2;
				else if (uartRx == '3') 
					screen = 3;
				else if (uartRx == '4')
					screen = 7;
				else if (uartRx == '5')
					screen = 9;
				break;
			case 2://Insert temperature value screen.
				if (uartRx < 48 || uartRx > 57) {
					screen = 4;
				} else {	
					if (idx == 0 || idx == 1) {
						screen = 2;
						uartRxBuffer[idx] = uartRx-48;
						screen_buffer[idx]= uartRx-48;
						idx ++;
					} else if(idx == 2) {
						uartRxBuffer[idx] = uartRx-48;
						tmp_PVT = (uint16_t)(uartRxBuffer[0])*100 + (uint16_t)(uartRxBuffer[1])*10 + (uint16_t)(uartRxBuffer[2]);
						if (tmp_PVT > 300)
							tmp_PVT = 300;
						idx = 0;
						screen = 5;
						uartRxBuffer[0] = 0;
						uartRxBuffer[1] = 0;
						uartRxBuffer[2] = 0;
						screen_buffer[0] = -48;
						screen_buffer[1] = -48;
						screen_buffer[2] = -48;
					}
				}
				break;
			case 3://Fan speed screen.
				if (uartRx < 48 || uartRx > 57) {
					screen = 4;
				} else {	
					if (idx == 0 || idx == 1) {
						screen = 3;
						uartRxBuffer[idx] = uartRx-48;
						screen_buffer[idx]= uartRx-48;
						idx ++;
					} else if(idx == 2) {
						uartRxBuffer[idx] = uartRx-48;
						tmp_fan_speed = uartRxBuffer[0]*100 + uartRxBuffer[1]*10 + uartRxBuffer[2];
						if (tmp_fan_speed > 100)
							tmp_fan_speed = 100;
						idx = 0;
						screen = 6;
						uartRxBuffer[0] = 0;
						uartRxBuffer[1] = 0;
						uartRxBuffer[2] = 0;
						screen_buffer[0] = -48;
						screen_buffer[1] = -48;
						screen_buffer[2] = -48;
					}
				}
				break;
			case 4://Error screen.
				screen = 0;
				uartRxBuffer[0] = 0;
				uartRxBuffer[1] = 0;
				uartRxBuffer[2] = 0;
				screen_buffer[0] = -48;
				screen_buffer[1] = -48;
				screen_buffer[2] = -48;
				idx = 0;
				break;
			case 5://Confirmation of PV screen.
				if (uartRx == 'y'){
					PVT = tmp_PVT;
					screen = 1;
				}
				else if(uartRx == 'n') {
					PVT = 0;
					screen = 1;
				}
				else
					screen = 4;
				break;
			case 6://Confirmation of fan screen.
				if (uartRx == 'y'){
					fan_speed = tmp_fan_speed;
					screen = 1;
				}
				else if(uartRx == 'n') {
					screen = 1;
				}
				else
					screen = 4;
				break;
			case 7://Insert timer value screen.
				if (uartRx < 48 || uartRx > 57) {
					screen = 4;
				} else {	
					if (idx == 0 || idx == 1) {
						screen = 7;
						uartRxBuffer[idx] = uartRx-48;
						screen_buffer[idx]= uartRx-48;
						idx ++;
					} else if(idx == 2) {
						uartRxBuffer[idx]= uartRx-48;
						tmp_usrTime = (uint16_t)(uartRxBuffer[0])*100 + (uint16_t)(uartRxBuffer[1])*10 + (uint16_t)(uartRxBuffer[2]);
						if (tmp_usrTime > 500)
							tmp_usrTime = 500;
						idx = 0;
						screen = 8;
						uartRxBuffer[0] = 0;
						uartRxBuffer[1] = 0;
						uartRxBuffer[2] = 0;
						screen_buffer[0] = -48;
						screen_buffer[1] = -48;
						screen_buffer[2] = -48;
					}
				}
				break;
			case 8://Confirmation of program temprature and timer..
				if (uartRx == 'y'){
					usrTime = tmp_usrTime;
					flag_usrTime = 1;
					time_cnt = 0;
					screen = 2;
				}
				else if(uartRx == 'n') {
					usrTime = 0;
					screen = 1;
				}
				else
					screen = 4;
				break;	
			default:
				screen = 0;
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
			case 0://Display welcome screen.
				clearBuffer();	
				sprintf(screen_message, " Select an operation                                                                \r");
				HAL_UART_Transmit_IT(&huart5, (uint8_t*)screen_message, sizeof(screen_message));
				break;
			case 1://Show Temperature and Fan screen.
				clearBuffer();
			  sprintf(screen_message, " Temperature : %f [Celsus] Fan speed: %d [percentage] Actuator PWM: %d               \r", temperature, fan_speed, (uint8_t)duty);
				HAL_UART_Transmit_IT(&huart5, (uint8_t*)screen_message, sizeof(screen_message));
				break;
		  case 2://Insert temperature value screen.
				clearBuffer();
				sprintf(screen_message, " Insert temperature to reach [000 - 300] %c%c%c                                      \r", screen_buffer[0]+48,screen_buffer[1]+48, screen_buffer[2]+48);
				HAL_UART_Transmit_IT(&huart5, (uint8_t*)screen_message, sizeof(screen_message));
				break;
			case 3://Fan speed screen.
				clearBuffer();
				sprintf(screen_message, " Insert fan speed [000 - 100] %c%c%c                                                 \r", screen_buffer[0]+48,screen_buffer[1]+48, screen_buffer[2]+48);
				HAL_UART_Transmit_IT(&huart5, (uint8_t*)screen_message, sizeof(screen_message));
				break;
			case 4://Error screen.
				clearBuffer();
				sprintf(screen_message, " Error, '%c' is not a valid input character. Press any key to continue...            \r", uartRx);
				HAL_UART_Transmit_IT(&huart5, (uint8_t*)screen_message, sizeof(screen_message));
				break;
		 case 5://Confimation PV.
				clearBuffer();
				sprintf(screen_message, " The projected value fot the temperatire is %d confirm?[y/n]                         \r", tmp_PVT);
				HAL_UART_Transmit_IT(&huart5, (uint8_t*)screen_message, sizeof(screen_message));
				break;
		 case 6://Confirmation fan.
				clearBuffer();
				sprintf(screen_message, " The fan speed %d confirm?[y/n]                                                       \r", tmp_fan_speed);
				HAL_UART_Transmit_IT(&huart5, (uint8_t*)screen_message, sizeof(screen_message));
				break;
		 case 7://Insert timer.
				clearBuffer();
				sprintf(screen_message, " Insert timer value, in minutes [000 - 500] %c%c%c                                     \r", screen_buffer[0]+48,screen_buffer[1]+48, screen_buffer[2]+48);
				HAL_UART_Transmit_IT(&huart5, (uint8_t*)screen_message, sizeof(screen_message));
				break;
		 case 8://Confirmation Timer.
				clearBuffer();
				sprintf(screen_message, " The timer is set to %d minutes, confirm?[y/n]                                          \r", tmp_usrTime);
				HAL_UART_Transmit_IT(&huart5, (uint8_t*)screen_message, sizeof(screen_message));
				break;
		 case 9://Show PVT and Timer.
				clearBuffer();
				if (flag_usrTime == 0) {
					sprintf(screen_message, " PVT : %d , Timer : %d [disable]                                                      \r", PVT, usrTime);
					HAL_UART_Transmit_IT(&huart5, (uint8_t*)screen_message, sizeof(screen_message));
				} else {
					sprintf(screen_message, " PVT : %d , Timer : %d [enable], timeCnt: %d                                                     \r", PVT, usrTime, time_cnt);
					HAL_UART_Transmit_IT(&huart5, (uint8_t*)screen_message, sizeof(screen_message));
				}
				break;
		 default://Default screen.
				clearBuffer();	
				sprintf(screen_message, " Select an operation                                                                     \r");
				HAL_UART_Transmit_IT(&huart5, (uint8_t*)screen_message, sizeof(screen_message));
				break;
		 
		}//END SWITCH.
		if (flag_usrTime == 1){ //Routine for oven tmer.
			time_cnt++; //Update time_cnt every 100ms.
			if (time_cnt == 600) { //100ms*600000 = 1min.
				usrTime--;
				time_cnt = 0;
				if (usrTime <= 0) {
					PVT = 0;
					flag_usrTime = 0;
					usrTime = 0;
				}
			}
		}
	}//END TIMER6 INTEERUPT ROUTINE.
	HAL_TIM_Base_Start_IT(htim);
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
