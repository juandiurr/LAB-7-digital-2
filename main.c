/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "fatfs_sd.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
SPI_HandleTypeDef hspi1;
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;
uint8_t RX[1]; // Buffer para recepción de datos
uint8_t input = 0;
char buffer[100];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
void transmit_uart(char *string);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, RX, 1);



  //ABRIR SD
  /*
  fes = f_open(&fil,"Prueba.txt",FA_OPEN_APPEND | FA_WRITE | FA_READ);
  if (fres==FR_OK){
	  transmit_uart("file opened for reading.\n");
  }else if(fres!=FR_OK){
	  ransmit_uart("file was not opened for reading.\n");
  }*/

  //LEER DATO
  /*
	while(f_gets(buffer,sizeof(buffer),&fil)){
	  char mRd[100];
	  sprintf(mRd,"%s",buffer);
	  transmit_uart(mRd);
  }
  */

  //ESCRIBIR DATO
  /*
	for(uint8_t i=0;i<10;i++){
		f_puts("Hola mundo.\n",&fil);
	}
   */

  //CERRAR SD
  /*
  fres = f_close(&fil);
  if (fres==FR_OK){
	  transmit_uart("The file is closed.\n");
  }else if(fres==FR_OK){
	  transmit_uart("The file was not closed.\n");
  }
  */

  //DESMONTAR SD
  /*
 fres = f_mount(NULL,"",1);
  if(fres == FR_OK){
	  transmit_uart("The Micro SD Card is unmounted!\n");
  }else if(fres != FR_OK){
	  transmit_uart("The Micro SD Card is NOT unmounted!\n")
  }
  */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  transmit_uart("1 para ver Mario.\r\n");
	  transmit_uart("2 para ver Yoshi.\r\n");
	  transmit_uart("3 para ver un corazón.\r\n");
	  do{

	  }while(input == 0);

	  if (input == 1){
		  HAL_Delay(500);
//Montar SD
		  fres = f_mount(&fs, "/",0);
		  if(fres==FR_OK){
			 // transmit_uart("Micro SD card is mounted succesfully!\n");
			  //transmit_uart("1 para ver Mario.\n");
			  //transmit_uart("2 para ver Yoshi.\n");
			  //transmit_uart("3 para ver un corazón.\n");
		  }else if(fres!=FR_OK){
			 // transmit_uart("Micro SD card's mount error!\n");
		  }
	  //Abrir SD
		  fres = f_open(&fil,"mario.txt",FA_OPEN_APPEND | FA_WRITE | FA_READ);
		    if (fres==FR_OK){
		    	//transmit_uart("file opened for reading.\n");
		    	//Leer SD
		    	 //Leer el archivo línea por línea
		    	    int line_count = 0; // Inicializar un contador para las líneas leídas
		    	    while (f_gets(buffer, sizeof(buffer), &fil)) {
		    	        // Incrementar el contador de líneas por cada línea leída
		    	        line_count++;
		    	        // Declarar un arreglo para formatear la línea leída
		    	        char formatted_line[200];
		    	        // Copiar el contenido del buffer a formatted_line
		    	        sprintf(formatted_line, "%s", buffer);
		    	        // Transmitir la línea formateada a través de UART
		    	        transmit_uart(formatted_line);
		    	        transmit_uart("\r\n");
		    	    }

		    }else if(fres!=FR_OK){
		    	//transmit_uart("file was not opened for reading.\n");
		    }
		    //Cerrar archivo
		    fres = f_close(&fil);
		    if (fres==FR_OK){
		    	//transmit_uart("The file is closed.\n");
		    }else if(fres==FR_OK){
		    	//transmit_uart("The file was not closed.\n");
		    }
		    //DEsmontar SD
		    fres = f_mount(NULL,"",1);
		    if(fres == FR_OK){
		    	//transmit_uart("The Micro SD Card is unmounted!\n");
		    }else if(fres != FR_OK){
		    	//transmit_uart("The Micro SD Card is NOT unmounted!\n");
		    }
		    input = 0;


	  }else if (input == 2){
		  HAL_Delay(500);
		  fres = f_mount(&fs, "/",0);
		  		  if(fres==FR_OK){
		  			  //transmit_uart("Micro SD card is mounted succesfully!\n");
		  			  //transmit_uart("1 para ver Mario.\n");
		  			  //transmit_uart("2 para ver Yoshi.\n");
		  			  //transmit_uart("3 para ver un corazón.\n");
		  		  }else if(fres!=FR_OK){
		  			  //transmit_uart("Micro SD card's mount error!\n");
		  		  }
		  	  //Abrir SD
		  fres = f_open(&fil,"yoshi.txt",FA_OPEN_APPEND | FA_WRITE | FA_READ);
			if (fres==FR_OK){
				//transmit_uart("file opened for reading.\n");
				//Leer SD
				 //Leer el archivo línea por línea
					int line_count = 0; // Inicializar un contador para las líneas leídas
					while (f_gets(buffer, sizeof(buffer), &fil)) {
						// Incrementar el contador de líneas por cada línea leída
						line_count++;
						// Declarar un arreglo para formatear la línea leída
						char formatted_line[200];
						// Copiar el contenido del buffer a formatted_line
						sprintf(formatted_line, "%s", buffer);
						// Transmitir la línea formateada a través de UART
						transmit_uart(formatted_line);
						transmit_uart("\r\n");
					}

			}else if(fres!=FR_OK){
				//transmit_uart("file was not opened for reading.\n");
			}
			//Cerrar archivo
			fres = f_close(&fil);
			if (fres==FR_OK){
				//transmit_uart("The file is closed.\n");
			}else if(fres==FR_OK){
				//transmit_uart("The file was not closed.\n");
			}
			//DEsmontar SD
			fres = f_mount(NULL,"",1);
			if(fres == FR_OK){
				//transmit_uart("The Micro SD Card is unmounted!\n");
			}else if(fres != FR_OK){
				//transmit_uart("The Micro SD Card is NOT unmounted!\n");
			}
			input = 0;
	  }else if(input == 3){
		  HAL_Delay(500);
		  fres = f_mount(&fs, "/",0);
		  		  if(fres==FR_OK){
		  			  //transmit_uart("Micro SD card is mounted succesfully!\n");
		  			  //transmit_uart("1 para ver Mario.\n");
		  			  //transmit_uart("2 para ver Yoshi.\n");
		  			  //transmit_uart("3 para ver un corazón.\n");
		  		  }else if(fres!=FR_OK){
		  			  //transmit_uart("Micro SD card's mount error!\n");
		  		  }
		  	  //Abrir SD
		  fres = f_open(&fil,"corazon.txt",FA_OPEN_APPEND | FA_WRITE | FA_READ);
			if (fres==FR_OK){
				//transmit_uart("file opened for reading.\n");
				//Leer SD
				 //Leer el archivo línea por línea
					int line_count = 0; // Inicializar un contador para las líneas leídas
					while (f_gets(buffer, sizeof(buffer), &fil)) {
						// Incrementar el contador de líneas por cada línea leída
						line_count++;
						// Declarar un arreglo para formatear la línea leída
						char formatted_line[200];
						// Copiar el contenido del buffer a formatted_line
						sprintf(formatted_line, "%s", buffer);
						// Transmitir la línea formateada a través de UART
						transmit_uart(formatted_line);
						transmit_uart("\r\n");
					}

			}else if(fres!=FR_OK){
				//transmit_uart("file was not opened for reading.\n");
			}
			//Cerrar archivo
			fres = f_close(&fil);
			if (fres==FR_OK){
				//transmit_uart("The file is closed.\n");
			}else if(fres==FR_OK){
				//transmit_uart("The file was not closed.\n");
			}
			//DEsmontar SD
			fres = f_mount(NULL,"",1);
			if(fres == FR_OK){
				//transmit_uart("The Micro SD Card is unmounted!\n");
			}else if(fres != FR_OK){
				//transmit_uart("The Micro SD Card is NOT unmounted!\n");
			}
			input = 0;

	  }
    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void transmit_uart(char *string){
	  //uint8_t len = strlen(string);
	HAL_UART_Transmit(&huart1,(uint8_t*) string, strlen(string), HAL_MAX_DELAY);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (RX[0] == '1'){//mostrar mario
			input = 1;
		}
	if (RX[0] == '2'){//mostrar yoshi
			input = 2;
		}
	if (RX[0] == '3'){//mostrar corazon
			input = 3;
		}
	HAL_UART_Receive_IT(&huart1, RX, 1);
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
