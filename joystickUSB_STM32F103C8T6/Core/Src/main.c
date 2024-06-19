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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	uint8_t botones; //  | mode | Z | Y | X | start | C | B | A |
	int8_t ejeX;
	int8_t ejeY;
}T_REPORTE_USB;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//variables USB
extern USBD_HandleTypeDef hUsbDeviceFS;
T_REPORTE_USB reporteUsb = {0};
//variables timer
uint8_t flag_tim3 = 0;
//variables joystick
uint8_t boton_up;
uint8_t boton_down;
uint8_t boton_left;
uint8_t boton_right;
uint8_t	boton_a;
uint8_t boton_b;
uint8_t boton_c;
uint8_t boton_x;
uint8_t boton_y;
uint8_t boton_z;
uint8_t boton_start;
uint8_t boton_mode;
uint8_t joystickTipe = 0; //0 -> no hay joystick; 3 -> de 3 botones; 6 -> de 6 botones.


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
  HAL_GPIO_WritePin(OUT_DB9_PIN7_GPIO_Port, OUT_DB9_PIN7_Pin, 1); //IDLE

  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (flag_tim3 != 0){

		  reporteUsb.ejeX = 0;
		  reporteUsb.ejeY = 0;
		  reporteUsb.botones = 0;
		  joystickTipe = 0;
		  HAL_GPIO_WritePin(OUT_DB9_PIN7_GPIO_Port, OUT_DB9_PIN7_Pin, 0); //1 low
		  HAL_GPIO_WritePin(OUT_DB9_PIN7_GPIO_Port, OUT_DB9_PIN7_Pin, 1); //1 high
		  HAL_GPIO_WritePin(OUT_DB9_PIN7_GPIO_Port, OUT_DB9_PIN7_Pin, 0); //2 low
		  HAL_GPIO_WritePin(OUT_DB9_PIN7_GPIO_Port, OUT_DB9_PIN7_Pin, 1); //2 high

		  if ( !HAL_GPIO_ReadPin(IN_DB9_PIN1_GPIO_Port, IN_DB9_PIN1_Pin) ){ //UP
			  reporteUsb.ejeY = -128;
		  }
		  if ( !HAL_GPIO_ReadPin(IN_DB9_PIN2_GPIO_Port, IN_DB9_PIN2_Pin) ){ //DOWN
			  reporteUsb.ejeY = 127;
		  }
		  if ( !HAL_GPIO_ReadPin(IN_DB9_PIN3_GPIO_Port, IN_DB9_PIN3_Pin) ){ //LEFT
			  reporteUsb.ejeX = -128;
		  }
		  if ( !HAL_GPIO_ReadPin(IN_DB9_PIN4_GPIO_Port, IN_DB9_PIN4_Pin) ){ //RIGHT
			  reporteUsb.ejeX = 127;
		  }
		  if ( !HAL_GPIO_ReadPin(IN_DB9_PIN6_GPIO_Port, IN_DB9_PIN6_Pin) ){ //B
			  reporteUsb.botones |= 0b10;
		  }
		  if ( !HAL_GPIO_ReadPin(IN_DB9_PIN9_GPIO_Port, IN_DB9_PIN9_Pin) ){ //C
			  reporteUsb.botones |= 0b100;
		  }

		  HAL_GPIO_WritePin(OUT_DB9_PIN7_GPIO_Port, OUT_DB9_PIN7_Pin, 0); //3 low

		  if ( !HAL_GPIO_ReadPin(IN_DB9_PIN3_GPIO_Port, IN_DB9_PIN3_Pin) && !HAL_GPIO_ReadPin(IN_DB9_PIN4_GPIO_Port, IN_DB9_PIN4_Pin) ){ //deteccion de 3 botones
			  joystickTipe = 3;
		  }
		  if ( !HAL_GPIO_ReadPin(IN_DB9_PIN1_GPIO_Port, IN_DB9_PIN1_Pin) && !HAL_GPIO_ReadPin(IN_DB9_PIN2_GPIO_Port, IN_DB9_PIN2_Pin) ){ //deteccion de 6 botones
			  joystickTipe = 6;
		  }
		  if ( !HAL_GPIO_ReadPin(IN_DB9_PIN6_GPIO_Port, IN_DB9_PIN6_Pin) ){ //A
			  reporteUsb.botones |= 0b1;
		  }
		  if ( !HAL_GPIO_ReadPin(IN_DB9_PIN9_GPIO_Port, IN_DB9_PIN9_Pin) ){ //start
			  reporteUsb.botones |= 0b1000;
		  }

		  HAL_GPIO_WritePin(OUT_DB9_PIN7_GPIO_Port, OUT_DB9_PIN7_Pin, 1); //3 high
		  HAL_GPIO_WritePin(OUT_DB9_PIN7_GPIO_Port, OUT_DB9_PIN7_Pin, 1); //3 high
		  HAL_GPIO_WritePin(OUT_DB9_PIN7_GPIO_Port, OUT_DB9_PIN7_Pin, 1); //3 high

		  if ( !HAL_GPIO_ReadPin(IN_DB9_PIN1_GPIO_Port, IN_DB9_PIN1_Pin) ){ //Z
			  reporteUsb.botones |= 0b1000000;
		  }
		  if ( !HAL_GPIO_ReadPin(IN_DB9_PIN2_GPIO_Port, IN_DB9_PIN2_Pin) ){ //Y
			  reporteUsb.botones |= 0b100000;
		  }
		  if ( !HAL_GPIO_ReadPin(IN_DB9_PIN3_GPIO_Port, IN_DB9_PIN3_Pin) ){ //X
			  reporteUsb.botones |= 0b10000;
		  }
		  if ( !HAL_GPIO_ReadPin(IN_DB9_PIN4_GPIO_Port, IN_DB9_PIN4_Pin) ){ //mode
			  reporteUsb.botones |= 0b10000000;
		  }

		  HAL_GPIO_WritePin(OUT_DB9_PIN7_GPIO_Port, OUT_DB9_PIN7_Pin, 0); //4 low
		  HAL_GPIO_WritePin(OUT_DB9_PIN7_GPIO_Port, OUT_DB9_PIN7_Pin, 1); //4 high

		  if (joystickTipe != 0){
			  USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&reporteUsb, 3);
		  }

		  flag_tim3 = 0;
	  }


//	  HAL_Delay(500);
//	  reporteUsb.ejeX = 127;
//	  USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&reporteUsb, 3);
//	  HAL_Delay(500);
//	  reporteUsb.ejeX = -128;
//	  USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&reporteUsb, 3);
//	  HAL_Delay(500);
//	  reporteUsb.ejeX = 0;
//	  reporteUsb.ejeY = -128;
//	  USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&reporteUsb, 3);
//	  HAL_Delay(500);
//	  reporteUsb.ejeY = 127;
//	  USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&reporteUsb, 3);
//	  HAL_Delay(500);
//	  reporteUsb.ejeY = 0;
//	  reporteUsb.botones = 0x02;
//	  USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&reporteUsb, 3);
//	  HAL_Delay(500);
//	  reporteUsb.botones = 0x04;
//	  USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&reporteUsb, 3);
//	  HAL_Delay(500);
//	  reporteUsb.botones = 0x08;
//	  USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&reporteUsb, 3);
//	  reporteUsb.botones = 0;

    /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	flag_tim3 = 1;
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
