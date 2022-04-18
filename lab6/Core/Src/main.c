/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t count = 0;
uint8_t cmdBuffer[3];
uint8_t dataBuffer[8];
uint32_t count2 = 0;
uint32_t count3 = 2999;
uint32_t t = 0;
uint32_t h = 0;
uint32_t hutu = 0;
uint32_t x = 0;
uint32_t tat = 0;
uint32_t termomiter = 28;
uint32_t bin[6] = {0,0,0,0,0,0};
uint32_t alert[3] = {0,1,0};
uint32_t alert2[3] = {0,2,0};
uint32_t st = 1;
uint32_t nub = 0;

float humid =30.0, temp=40.0;
uint8_t step = 0;
HAL_StatusTypeDef status;
char str[50];
char num[] = " ";
char numn[] = " ";
char num1[] = " ";
char num2[] = " ";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t CRC16_2(uint8_t *,uint8_t );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  cmdBuffer[0] = 0x03;
  cmdBuffer[1] = 0x00;
  cmdBuffer[2] = 0x04;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT (&htim1);
  HAL_TIM_Base_Start_IT (&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_Delay(400);
//	  displayNumber2(count);
	  char strt[] = "\rTimer          \r\n\tpress button to set time\r\n\tAlert Time : 05:00:00";
	  char strt2[] = "\r\n\tAlert Time2 : ";
	  sprintf(num2,"%.2d:%.2d:%.2d\n",alert2[0],alert2[1],alert2[2]);
	  while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
	  HAL_UART_Transmit(&huart3, (uint32_t*)strt,strlen(strt),1000);
	  HAL_UART_Transmit(&huart3, (uint32_t*)strt2,strlen(strt2),1000);
	  HAL_UART_Transmit(&huart3, (uint32_t*)num2,strlen(num2),1000);
	  HAL_Delay(800);
	  termomiter = 31;
	  while(st){
//
//		  Temp();
		  if(count2 == 1){
			  count2 = 0;
		  	displayNumber2();
		  	checkalert();
		  	ctemp(termomiter);
		   }
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 108;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_USART6|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Temp(){

	  HAL_I2C_Master_Transmit(&hi2c1,0x5c<<1,cmdBuffer,3,200);
	  HAL_I2C_Master_Transmit(&hi2c1,0x5c<<1,cmdBuffer,3,200);

	  HAL_Delay(1);

		  HAL_I2C_Master_Receive(&hi2c1,0x5c<<1,dataBuffer,8,200);

		  uint16_t Rcrc = dataBuffer[7]<<8;
		  Rcrc += dataBuffer[6];
		  if(Rcrc == CRC16_2(dataBuffer,6)){
		  uint16_t temperature = ((dataBuffer[4] & 0x7F) << 8) + dataBuffer[5];

		  temp = (((dataBuffer[4] & 0x80) >> 7) == 1)?(t*(-1)) : t ;
		  temp = temperature / 10.0;

		  uint16_t humidity = (dataBuffer[2] << 8) + dataBuffer[3];
		  humid = humidity /10.0;

//		  sprintf(str,"Temp  %4.1f C\n\rHumid %4.1f %%\n\r\n",temp,humid);
//		  while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
//		  HAL_UART_Transmit(&huart3,(uint8_t*) str,strlen(str),200);
}
}

//void displayTemp(uint8_t t,uint8_t h){
//	 sprintf(str,"Temp  %4.1f C\n\rHumid %4.1f %%\n\r\n",t,h);
//     while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
//	 HAL_UART_Transmit(&huart3,(uint8_t*) str,strlen(str),200);
//}


uint16_t CRC16_2(uint8_t *ptr, uint8_t length)
{
    uint16_t crc = 0xFFFF;
    uint8_t s = 0x00;

    while(length--){
        crc ^= *ptr++;
        for(s = 0; s < 8; s++){
            if((crc & 0x01) != 0){
                crc >>= 1;
                crc ^= 0xA001;
            } else crc >>= 1;
        }
    }
    return crc;
}

//void displayNumber(uint32_t x)
//{
//	sprintf(num,"%d",x);
//	while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
//	HAL_UART_Transmit(&huart3, (uint32_t*) "\n\r", 4,1000);
//	HAL_UART_Transmit(&huart3, (uint32_t*)num,strlen(num),1000);
//
//}
void checkalert(){
	if(alert[0]==h ){
		if(alert[1]== t ){
			if(alert[2]== x ){
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, 1);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1);

			}}}
	if(alert2[0]==h ){
			if(alert2[1]==t ){
				if(alert2[2]==x ){
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, 0);
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 1);}}}
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 0);
}
void ctemp(int e ){
		if(0== t ){
			if(7 == x){
				senttemp(termomiter);
				Temp();
				hutu = humid;
				termomiter = temp;
			}
		}
		if(x%10 ==0){
				sprintf(num2," %.2d:%.2d:%.2d ",h,t,x);
			nub = strlen(num2);
			while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)==RESET){}
			HAL_UART_Transmit(&huart2, (uint32_t*) num2,strlen(num2),1000 );
			}
		else if(x%5 == 0){
			sprintf(num2,"T=%.2d H=%.2d ",termomiter,hutu);
			nub = strlen(num2);
			while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)==RESET){}
			HAL_UART_Transmit(&huart2, (uint32_t*) num2,strlen(num2),1000 );
					}

		if(t%1 == 0 ){
			if(18 == x){
			Temp();
			termomiter = (termomiter+temp)/2;
			hutu = (hutu+humid)/2;
			senttemp(termomiter);

		}
				}

}
void senttemp(int e){
	for(int w=0 ; w < 6 ;w++){
		bin[w] = e % 2;
		e /= 2;
	}
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 1);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, bin[0]);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, bin[1]);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, bin[2]);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, bin[3]);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, bin[4]);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, bin[5]);
	HAL_Delay(300);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 0);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 0);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 0);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 0);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 0);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 0);
}
void displayNumber2()
{
//	x = x/1000;
	count3 += 2;
//	x += 1;
//	if(x>59){
//		t++;
//		x = 0;}
//	if(t>59){
//			t = 0;
//			h++;
//			}
//	if(h>23){
//			h=0;
//			}
//	sprintf(num,"%d",count3);
//	sprintf(num,"%.2d:%.2d:%.2d Temp = %2d  ",h,t,x,termomiter);
	Temp();
	termomiter = temp;
	hutu = humid;
	sprintf(num,"%.2d:%.2d:%.2d Temp = %2d Humid %2d",h,t,x,termomiter,hutu);
	while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
	HAL_UART_Transmit(&huart3, (uint32_t*) "\r", 4,1000);
	HAL_UART_Transmit(&huart3, (uint32_t*)num,strlen(num),1000);
//	sprintf(num,"%2d",termomiter);
//	while(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_TC)==RESET){}
//	HAL_UART_Transmit(&huart6, (uint32_t*)num,strlen(num),1000);
//	if(count3 == 3029){
//			while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
//				HAL_UART_Transmit(&huart3, (uint32_t*) "\n\r", 4,1000);
//				HAL_UART_Transmit(&huart3, (uint32_t*)"stop",4,1000);
//				st = 0;
//		}

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	if(GPIO_Pin == GPIO_PIN_13){
//		HAL_Delay(1000);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
//		HAL_Delay(1000);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
//		HAL_Delay(1000);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
//	}
	tat = 0 ;
	while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
					HAL_UART_Transmit(&huart3, (uint32_t*) "\n\r Next", 9,1000);
	while(1){
//		while(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) == RESET){}
//				HAL_UART_Receive(&huart3, (uint8_t*) &usrType, 1, 1000);
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == GPIO_PIN_RESET ){
			HAL_Delay(7000);
			if(tat == 0){
				h++;
			if(h>23){
				h=0;
			}
			sprintf(num,"Time = %.2d:%.2d:%.2d",h,t,x);
			while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
				HAL_UART_Transmit(&huart3, (uint32_t*) "\r", 2,1000);
				HAL_UART_Transmit(&huart3, (uint32_t*)num,strlen(num),1000);
			sprintf(num2," %.2d:%.2d:%.2d ",h,t,x);
				while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)==RESET){}
				HAL_UART_Transmit(&huart2, (uint32_t*) num2,strlen(num2),1000 );
			}
			if(tat == 1){
				alert[0]++;
				if(alert[0] > 23){
					alert[0] = 0;
				}
				sprintf(num,"Open %.2d:%.2d:%.2d",alert[0],alert[1],alert[2]);
				while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
				HAL_UART_Transmit(&huart3, (uint32_t*) "\r", 2,1000);
				HAL_UART_Transmit(&huart3, (uint32_t*)num,strlen(num),1000);
				sprintf(num2," %.2d:%.2d:%.2d ",alert[0],alert[1],alert[2]);
				while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)==RESET){}
				HAL_UART_Transmit(&huart2, (uint32_t*) num2,strlen(num2),1000 );

			}
			if(tat == 2){
							alert2[0]++;
							if(alert2[0] > 23){
								alert2[0] = 0;
							}
							sprintf(num,"Close %.2d:%.2d:%.2d",alert2[0],alert2[1],alert2[2]);
							while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
							HAL_UART_Transmit(&huart3, (uint32_t*) "\r", 2,1000);
							HAL_UART_Transmit(&huart3, (uint32_t*)num,strlen(num),1000);
							sprintf(num2," %.2d:%.2d:%.2d ",alert2[0],alert2[1],alert2[2]);
							while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)==RESET){}
							HAL_UART_Transmit(&huart2, (uint32_t*) num2,strlen(num2),1000 );

						}

		}
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == GPIO_PIN_RESET ){
			HAL_Delay(7000);
			if(tat ==  0){
				t++;
					if(t>59){
									t=0;
								}
					sprintf(num,"Time = %.2d:%.2d:%.2d",h,t,x);
								while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
									HAL_UART_Transmit(&huart3, (uint32_t*) "\r", 2,1000);
									HAL_UART_Transmit(&huart3, (uint32_t*)num,strlen(num),1000);
									sprintf(num2," %.2d:%.2d:%.2d ",h,t,x);
									while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)==RESET){}
									HAL_UART_Transmit(&huart2, (uint32_t*) num2,strlen(num2),1000 );
			}
			if(tat == 1){
							alert[1]++;
							if(alert[1] > 59){
								alert[1] = 0;
							}
							sprintf(num,"Open %.2d:%.2d:%.2d",alert[0],alert[1],alert[2]);
							while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
							HAL_UART_Transmit(&huart3, (uint32_t*) "\r", 2,1000);
							HAL_UART_Transmit(&huart3, (uint32_t*)num,strlen(num),1000);
							sprintf(num2," %.2d:%.2d:%.2d ",alert[0],alert[1],alert[2]);
											while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)==RESET){}
											HAL_UART_Transmit(&huart2, (uint32_t*) num2,strlen(num2),1000 );

						}
			if(tat == 2){
										alert2[1]++;
										if(alert2[1] > 59){
											alert2[1] = 0;
										}
										sprintf(num,"Close %.2d:%.2d:%.2d",alert2[0],alert2[1],alert2[2]);
										while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
										HAL_UART_Transmit(&huart3, (uint32_t*) "\r", 2,1000);
										HAL_UART_Transmit(&huart3, (uint32_t*)num,strlen(num),1000);
										sprintf(num2," %.2d:%.2d:%.2d ",alert2[0],alert2[1],alert2[2]);
										while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)==RESET){}
										HAL_UART_Transmit(&huart2, (uint32_t*) num2,strlen(num2),1000 );

									}

				}
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET ){
			HAL_Delay(7000);
			if(tat == 0){
				x++;
					if(x>59){
							x=0;
					}
					sprintf(num,"Time = %.2d:%.2d:%.2d",h,t,x );
								while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
									HAL_UART_Transmit(&huart3, (uint32_t*) "\r", 2,1000);
									HAL_UART_Transmit(&huart3, (uint32_t*)num,strlen(num),1000);
									sprintf(num2," %.2d:%.2d:%.2d ",h,t,x);
									while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)==RESET){}
									HAL_UART_Transmit(&huart2, (uint32_t*) num2,strlen(num2),1000 );
			}
			if(tat == 1){
							alert[2]++;
							if(alert[2] > 59){
								alert[2] = 0;
							}
							sprintf(num,"Open %.2d:%.2d:%.2d",alert[0],alert[1],alert[2]);
							while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
							HAL_UART_Transmit(&huart3, (uint32_t*) "\r", 2,1000);
							HAL_UART_Transmit(&huart3, (uint32_t*)num,strlen(num),1000);
							sprintf(num2," %.2d:%.2d:%.2d ",alert[0],alert[1],alert[2]);
							while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)==RESET){}
							HAL_UART_Transmit(&huart2, (uint32_t*) num2,strlen(num2),1000 );

						}
			if(tat == 2){
										alert2[2]++;
										if(alert2[2] > 59){
											alert2[2] = 0;
										}
										sprintf(num,"Close %.2d:%.2d:%.2d",alert2[0],alert2[1],alert2[2]);
										while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
										HAL_UART_Transmit(&huart3, (uint32_t*) "\r", 2,1000);
										HAL_UART_Transmit(&huart3, (uint32_t*)num,strlen(num),1000);
										sprintf(num2," %.2d:%.2d:%.2d ",alert2[0],alert2[1],alert2[2]);
										while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)==RESET){}
										HAL_UART_Transmit(&huart2, (uint32_t*) num2,strlen(num2),1000 );

									}
				}

		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET ){
			HAL_Delay(7000);
			tat += 1 ;
			HAL_UART_Transmit(&huart3, (uint32_t*) "\n\r Next", 9,1000);
			if(tat == 3 ){
				break;
			}
				}
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
