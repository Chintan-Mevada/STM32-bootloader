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
#include "sdio.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define bufferSize 512
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

FATFS fatfs;
FIL fil;
FRESULT fresult;
FILINFO info;

UINT bytesRead = 0;
uint8_t readBuffer[bufferSize];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void gotoApp(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sendData(char *msg){
	while(*msg != '\0'){
		HAL_UART_Transmit(&huart3, (uint8_t *)msg, 1, 1000);
		msg++;
	}
}

void flashErase(void) {
	HAL_StatusTypeDef res;
	uint32_t sectorError;

	HAL_FLASH_Unlock();
	/* Check if the FLASH_FLAG_BSY */
	FLASH_WaitForLastOperation(HAL_MAX_DELAY);

	/* Clear all flags before write it to flash */
	__HAL_FLASH_CLEAR_FLAG(
			FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	sendData("Erasing the Flash Memory...\r\n");

	/* Erase the flash */
	FLASH_EraseInitTypeDef flashErase;

	flashErase.TypeErase = FLASH_TYPEERASE_SECTORS;
	flashErase.Sector = FLASH_SECTOR_5;
	flashErase.NbSectors = 2;
	flashErase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	res = HAL_FLASHEx_Erase(&flashErase, &sectorError);
	if (res != HAL_OK) {
		sendData("Flash erqse Error!\r\n");

	} else {
		sendData("Flash erase Successfull...!\r\n");
	}
}

void flashProgram(void) {
	uint32_t programBytesToRead = info.fsize;  // Get file size
	uint32_t currentAddress = 0x08040000;

	while (programBytesToRead > 0) {
		// Read up to 512 bytes from the file
		if ((fresult = f_read(&fil, readBuffer, bufferSize, &bytesRead))
				!= FR_OK) {
			sendData("Application.bin file read unsuccessfull...!\r\n");
		} else {
			sendData("Application.bin file read successfull...!\r\n");
		}

		// Write the data into flash memory, 4 bytes at a time
		for (uint32_t i = 0; i < bytesRead; i += 4) {
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, currentAddress,
					*(uint32_t*) &readBuffer[i]);
			currentAddress += 4;
		}

		programBytesToRead -= bytesRead;
	}

	sendData(
			"/----------Write the data into flash memory Successfull----------/\r\n");
	HAL_FLASH_Lock();
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_SDIO_SD_Init();
	MX_USART3_UART_Init();
	MX_FATFS_Init();
	/* USER CODE BEGIN 2 */

	sendData("/----------Starting Bootloader v0.1----------/\r\n");

	if ((fresult = f_mount(&fatfs, "0:/", 0)) != FR_OK) {
		sendData("Sdcard mounting unsuccessfull...!\r\n");
	} else {
		sendData("Sdcard mounting successfull...!\r\n");

		if ((fresult = f_open(&fil, "0:/Application.bin",
		FA_READ | FA_OPEN_ALWAYS)) != FR_OK) {
			sendData("Application.bin file open unsuccessfull...!\r\n");
		} else {
			sendData("Application.bin file open successfull...!\r\n");

			f_stat("0:/Application.bin", &info);
			flashErase();
			flashProgram();

			if ((fresult = f_close(&fil)) != FR_OK) {
				sendData("Application.bin file close unsuccessfull...!\r\n");
			} else {
				sendData("Application.bin file close successfull...!\r\n");
			}
		}
	}
	gotoApp();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
/*#ifdef __GNUC__
int __to_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
	 Place your implementation of fputc here
	 e.g. write a character to the UART3 and Loop until the end of transmission
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 1000);

	return ch;
}*/

static void gotoApp(void) {
	sendData("/----------Jump to Application----------/\r\n");

	void (*appResetHandler)(void) = (void*)(*((volatile uint32_t *)(0x08040000 + 4U)));

	HAL_GPIO_WritePin(userLed1Out_GPIO_Port, userLed1Out_Pin, GPIO_PIN_SET);

	HAL_RCC_DeInit();
	HAL_DeInit();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	__set_MSP(*(volatile uint32_t*) 0x08040000);
	/*Jump to application*/
	appResetHandler(); // call the appResetHandler function
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
