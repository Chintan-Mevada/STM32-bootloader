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
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define application_address 	0x08040000
#define bufferSize 				512
#define file_name				"0:/F407_usb_app.bin"
#define usb_mount 				f_mount(&fatfs, USBHPath, 0)
#define usb_unmount 			f_mount(NULL, USBHPath, 0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern ApplicationTypeDef Appli_state;

uint32_t start = 0, end = 0;

FATFS fatfs;
FIL fil;
FRESULT fresult;
FILINFO info;

UINT bytesRead = 0;
uint8_t readBuffer[bufferSize];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static void gotoApp(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void flashErase(void) {
	HAL_StatusTypeDef res;
	uint32_t sectorError;

	HAL_FLASH_Unlock();
	/* Check if the FLASH_FLAG_BSY */
	FLASH_WaitForLastOperation(HAL_MAX_DELAY);

	/* Clear all flags before write it to flash */
	__HAL_FLASH_CLEAR_FLAG(
			FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	/* Erase the flash */
	FLASH_EraseInitTypeDef flashErase;

	flashErase.TypeErase = FLASH_TYPEERASE_SECTORS;
	flashErase.Sector = FLASH_SECTOR_5;
	flashErase.NbSectors = 2;
	flashErase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	res = HAL_FLASHEx_Erase(&flashErase, &sectorError);
	if (res != HAL_OK) {
	}
}

void flashProgram(void) {
	uint32_t programBytesToRead = info.fsize;  // Get file size
	uint32_t currentAddress = application_address;

	while (programBytesToRead > 0) {
		// Read up to 512 bytes from the file
		if ((fresult = f_read(&fil, readBuffer, bufferSize, &bytesRead))
				!= FR_OK) {
		}

		// Write the data into flash memory, 4 bytes at a time
		for (uint32_t i = 0; i < bytesRead; i += 4) {
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, currentAddress,
					*(uint32_t*) &readBuffer[i]);
			currentAddress += 4;
		}

		programBytesToRead -= bytesRead;
	}
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
	MX_FATFS_Init();
	MX_USB_HOST_Init();
	/* USER CODE BEGIN 2 */
	usb_mount;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		MX_USB_HOST_Process();

		/* USER CODE BEGIN 3 */
		end = HAL_GetTick() - start;
		if (end > 2000) {
			gotoApp();
		}
		if (Appli_state == APPLICATION_READY && end < 2000) {
			if ((fresult = f_open(&fil, file_name, FA_READ | FA_OPEN_ALWAYS))
					!= FR_OK) {
			} else {
				f_stat(file_name, &info);
				flashErase();
				flashProgram();

				fresult = f_close(&fil);
			}
			gotoApp();
		}
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
static void gotoApp(void) {
	void (*appResetHandler)(
			void) = (void*)(*((volatile uint32_t *)(application_address + 4U)));

	HAL_GPIO_WritePin(user_led_1_out_GPIO_Port, user_led_1_out_Pin, GPIO_PIN_SET);

	HAL_RCC_DeInit();
	HAL_DeInit();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	__set_MSP(*(volatile uint32_t*) application_address);
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
