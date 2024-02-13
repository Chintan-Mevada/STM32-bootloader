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
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	Action_None,
	Action_Update_App1,
	Action_Update_App2,
	Action_Run_App1,
	Action_Run_App2
} ActionType;

typedef struct {
	uint32_t slot0;
	uint32_t slot1;
} slotAddress;

typedef struct {
	uint8_t pin1State;
	uint8_t pin2State;
	uint8_t pin3State;
	uint8_t pin4State;
} pinState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define slot0Addr 0x08040000
#define slot1Addr 0x08080000

#define bufferSize 1024

#define userSwitch1InRead HAL_GPIO_ReadPin(userSwitch1In_GPIO_Port, userSwitch1In_Pin)
#define userSwitch2InRead HAL_GPIO_ReadPin(userSwitch2In_GPIO_Port, userSwitch2In_Pin)
#define userSwitch3InRead HAL_GPIO_ReadPin(userSwitch3In_GPIO_Port, userSwitch3In_Pin)
#define userSwitch4InRead HAL_GPIO_ReadPin(userSwitch4In_GPIO_Port, userSwitch4In_Pin)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FATFS fatfs;
FIL fil;
FRESULT fres;
FILINFO info;

ActionType currentAction = Action_None;
slotAddress currentAddress;
pinState button;

uint8_t readBuffer[bufferSize];
UINT bytesRead;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void gotoApp1(void);
static void gotoApp2(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sendData(char *msg) {
	while (*msg != '\0') {
		HAL_UART_Transmit(&huart3, (uint8_t*) msg, 1, 1000);
		msg++;
	}
}

void flashErase(uint8_t slotNum) {
	FLASH_EraseInitTypeDef flashErase;
	HAL_StatusTypeDef ret;
	uint32_t sectorError;

	HAL_FLASH_Unlock();

	/* Check if the FLASH_FLAG_BSY */
	FLASH_WaitForLastOperation(HAL_MAX_DELAY);

	/* Clear all flags before write it to flash */
	__HAL_FLASH_CLEAR_FLAG(
			FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);


	if (slotNum == 0) {
			sendData("Erasing the Slot0 Flash memory...\r\n");
				flashErase.TypeErase = FLASH_TYPEERASE_SECTORS;
				flashErase.Sector = FLASH_SECTOR_5;
				flashErase.NbSectors = 2;
				flashErase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
			} else {
			sendData("Erasing the Slot1 Flash memory...\r\n");
				flashErase.TypeErase = FLASH_TYPEERASE_SECTORS;
				flashErase.Sector = FLASH_SECTOR_7;
				flashErase.NbSectors = 2;
				flashErase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		}

	ret = HAL_FLASHEx_Erase(&flashErase, &sectorError);
	if (ret != HAL_OK) {
		sendData("Flash erase Error!\r\n");

	} else {
		sendData("Flash erase Successfull...!\r\n");
	}
}

void flashProgram(void) {
	uint32_t programBytesToRead = info.fsize;

	switch (currentAction) {
	case (Action_Update_App1):
		sendData("/-----Application 1 update Start-----/\r\n");
		currentAddress.slot0 = slot0Addr;
		while (programBytesToRead > 0) {
			// Read up to 512 bytes from the file
			fres = f_read(&fil, readBuffer, bufferSize, &bytesRead);

			// Write the data into flash memory, 4 bytes at a time
			for (uint32_t i = 0; i < bytesRead; i += 4) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, currentAddress.slot0,
						*(uint32_t*) &readBuffer[i]);
				currentAddress.slot0 += 4;
			}

			programBytesToRead -= bytesRead;
		}

		sendData(
				"/-----Application1.bin Read & Write Successfull in Slot0-----/\r\n");
		HAL_FLASH_Lock();
		currentAction = Action_None;
		memset(readBuffer, '\0', bufferSize); // clear buffer
		sendData("/-----Application 1 updated Successfully-----/\r\n");
		break;

	case (Action_Update_App2):
		sendData("/-----Application 2 update Start-----/\r\n");
		currentAddress.slot1 = slot1Addr;
		while (programBytesToRead > 0) {
			// Read up to 1024 bytes from the file
			fres = f_read(&fil, readBuffer, bufferSize, &bytesRead);

			// Write the data into flash memory, 4 bytes at a time
			for (uint32_t i = 0; i < bytesRead; i += 4) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, currentAddress.slot1,
						*(uint32_t*) &readBuffer[i]);
				currentAddress.slot1 += 4;
			}

			programBytesToRead -= bytesRead;
		}

		sendData(
				"/-----Application2.bin Read & Write Successfull in Slot1-----/\r\n");
		HAL_FLASH_Lock();
		currentAction = Action_None;
		memset(readBuffer, '\0', bufferSize); //clear buffer
		sendData("/-----Application 2 updated Successfully-----/\r\n");
		break;

	case (Action_Run_App1):
		gotoApp1();
		break;

	case (Action_Run_App2):
		gotoApp2();
		break;

	default:
		break;

	}

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
	sendData("/---------------Starting Bootloader v2.0---------------/\r\n");

	HAL_GPIO_WritePin(userLed1Out_GPIO_Port, userLed1Out_Pin, GPIO_PIN_RESET);
	if ((fres = f_mount(&fatfs, "0:/", 0)) != FR_OK) {
		sendData("Sdcard mounting unsuccessfull...!\r\n");
	} else {
		sendData("Sdcard mounting successfull...!\r\n");
		sendData("Pressed User Switch to create action...!\r\n");
		HAL_Delay(1000);
		button.pin1State = userSwitch1InRead;
		button.pin2State = userSwitch2InRead;
		button.pin3State = userSwitch3InRead;
		button.pin4State = userSwitch4InRead;

		/* Check which button is Pressed */
		if (button.pin1State == GPIO_PIN_RESET) {
			currentAction = Action_Update_App1;

			/* Open applicatio1.bin file */
			if ((fres = f_open(&fil, "0:/application1.bin",
			FA_READ | FA_OPEN_ALWAYS)) != FR_OK) {
				sendData("application1.bin file open unsuccessfull...!\r\n");
			} else {
				sendData("application1.bin file open successfull...!\r\n");

				f_stat("0:/application1.bin", &info);
				flashErase(0); // clear Slot0
				flashProgram(); // call flash function

				if ((fres = f_close(&fil)) != FR_OK) {
					sendData(
							"application1.bin file close unsuccessfull...!\r\n");
				} else {
					sendData("application1.bin file close successfull...!\r\n");
				}
			}
		} else if (button.pin2State == GPIO_PIN_RESET) {
			currentAction = Action_Update_App2;

			/* Open applicatio2.bin file */
			if ((fres = f_open(&fil, "0:/application2.bin",
			FA_READ | FA_OPEN_ALWAYS)) != FR_OK) {
				sendData("application2.bin file open unsuccessfull...!\r\n");
			} else {
				sendData("application2.bin file open successfull...!\r\n");

				f_stat("0:/application2.bin", &info);
				flashErase(1); // clear Slot1
				flashProgram(); // call flash function

				if ((fres = f_close(&fil)) != FR_OK) {
					sendData(
							"application2.bin file close unsuccessfull...!\r\n");
				} else {
					sendData("application2.bin file close successfull...!\r\n");
				}
			}
		} else if (button.pin3State == GPIO_PIN_RESET) {
			currentAction = Action_Run_App1;
			flashProgram(); // call flash function
		} else if (button.pin4State == GPIO_PIN_RESET) {
			currentAction = Action_Run_App2;
			flashProgram(); // call flash function
		}
	}
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
static void gotoApp1(void) {
	sendData("Jump to the Application 1...\r\n");

	void (*appResetHandler)(
			void) = (void*) (*((volatile uint32_t*)(slot0Addr + 4U)));

	HAL_GPIO_WritePin(userLed1Out_GPIO_Port, userLed1Out_Pin, GPIO_PIN_SET);

	HAL_RCC_DeInit();
	HAL_DeInit();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	__set_MSP(*(volatile uint32_t*) slot0Addr);

//	Jump to the Application
	appResetHandler(); // call Application reset handler
}

static void gotoApp2(void) {
	sendData("Jump to the Application 2...\r\n");

	void (*appResetHandler)(
			void) = (void*) (*((volatile uint32_t*)(slot1Addr + 4U)));

	HAL_GPIO_WritePin(userLed1Out_GPIO_Port, userLed1Out_Pin, GPIO_PIN_SET);

	HAL_RCC_DeInit();
	HAL_DeInit();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	__set_MSP(*(volatile uint32_t*) slot1Addr);

//	Jump to the Application
	appResetHandler(); // call Application reset handler
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
