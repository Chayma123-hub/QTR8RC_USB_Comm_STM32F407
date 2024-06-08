/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 *  Created on: JUIN, 2024
 *      Author: GARGOUCHAA
 */
******************************************************************************* /
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "string.h"
    /* Private includes ----------------------------------------------------------*/
    /* USER CODE BEGIN Includes */

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
    TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint32_t sensor_read = 0x00000000;
int pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us(uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0); // set the counter value a 0
  while (__HAL_TIM_GET_COUNTER(&htim2) < us)
    ; // wait for the counter to reach the us input in the parameter
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void QTR8_Test()
{
  Set_Pin_Output(SENSOR1_GPIO_Port, SENSOR1_Pin);
  Set_Pin_Output(SENSOR2_GPIO_Port, SENSOR2_Pin);
  Set_Pin_Output(SENSOR3_GPIO_Port, SENSOR3_Pin);
  Set_Pin_Output(SENSOR4_GPIO_Port, SENSOR4_Pin);
  Set_Pin_Output(SENSOR5_GPIO_Port, SENSOR5_Pin);
  Set_Pin_Output(SENSOR6_GPIO_Port, SENSOR6_Pin);
  Set_Pin_Output(SENSOR7_GPIO_Port, SENSOR7_Pin);
  Set_Pin_Output(SENSOR8_GPIO_Port, SENSOR8_Pin);

  HAL_GPIO_WritePin(SENSOR1_GPIO_Port, SENSOR1_Pin, 1);
  HAL_GPIO_WritePin(SENSOR2_GPIO_Port, SENSOR2_Pin, 1);
  HAL_GPIO_WritePin(SENSOR3_GPIO_Port, SENSOR3_Pin, 1);
  HAL_GPIO_WritePin(SENSOR4_GPIO_Port, SENSOR4_Pin, 1);
  HAL_GPIO_WritePin(SENSOR5_GPIO_Port, SENSOR5_Pin, 1);
  HAL_GPIO_WritePin(SENSOR6_GPIO_Port, SENSOR6_Pin, 1);
  HAL_GPIO_WritePin(SENSOR7_GPIO_Port, SENSOR7_Pin, 1);
  HAL_GPIO_WritePin(SENSOR8_GPIO_Port, SENSOR8_Pin, 1);
  delay_us(10);

  Set_Pin_Input(SENSOR1_GPIO_Port, SENSOR1_Pin);
  Set_Pin_Input(SENSOR2_GPIO_Port, SENSOR2_Pin);
  Set_Pin_Input(SENSOR3_GPIO_Port, SENSOR3_Pin);
  Set_Pin_Input(SENSOR4_GPIO_Port, SENSOR4_Pin);
  Set_Pin_Input(SENSOR5_GPIO_Port, SENSOR5_Pin);
  Set_Pin_Input(SENSOR6_GPIO_Port, SENSOR6_Pin);
  Set_Pin_Input(SENSOR7_GPIO_Port, SENSOR7_Pin);
  Set_Pin_Input(SENSOR8_GPIO_Port, SENSOR8_Pin);

  delay_us(10000);

  pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0, pos5 = 0, pos6 = 0, pos7 = 0, pos8 = 0;
  if (HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin))
  {
    sensor_read |= 0x00000001;
    pos1 = 1000;
  }

  if (HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin))
  {
    sensor_read |= 0x00000010;
    pos2 = 2000;
  }

  if (HAL_GPIO_ReadPin(SENSOR3_GPIO_Port, SENSOR3_Pin))
  {
    sensor_read |= 0x00000100;
    pos3 = 3000;
  }

  if (HAL_GPIO_ReadPin(SENSOR4_GPIO_Port, SENSOR4_Pin))
  {
    sensor_read |= 0x00001000;
    pos4 = 4000;
  }

  if (HAL_GPIO_ReadPin(SENSOR5_GPIO_Port, SENSOR5_Pin))
  {
    sensor_read |= 0x00010000;
    pos5 = 5000;
  }

  if (HAL_GPIO_ReadPin(SENSOR6_GPIO_Port, SENSOR6_Pin))
  {
    sensor_read |= 0x00100000;
    pos6 = 6000;
  }

  if (HAL_GPIO_ReadPin(SENSOR7_GPIO_Port, SENSOR7_Pin))
  {
    sensor_read |= 0x01000000;
    pos7 = 7000;
  }

  if (HAL_GPIO_ReadPin(SENSOR8_GPIO_Port, SENSOR8_Pin))
  {
    sensor_read |= 0x10000000;
    pos8 = 8000;
  }
}

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
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    QTR8_Test();

    char buffer[1000];

    snprintf(buffer, sizeof(buffer), "Pos1 :%d  Pos2 :%d  Pos3 :%d  Pos4 :%d  Pos5 :%d  Pos6 :%d  Pos7 :%d  Pos8 :%d\n", pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8);

    CDC_Transmit_FS((uint8_t *)buffer, strlen(buffer));

    HAL_Delay(500);

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 20;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SENSOR1_Pin | SENSOR2_Pin | SENSOR3_Pin | SENSOR4_Pin | SENSOR5_Pin | SENSOR6_Pin | SENSOR7_Pin | SENSOR8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SENSOR1_Pin SENSOR2_Pin SENSOR3_Pin SENSOR4_Pin
                           SENSOR5_Pin SENSOR6_Pin SENSOR7_Pin SENSOR8_Pin */
  GPIO_InitStruct.Pin = SENSOR1_Pin | SENSOR2_Pin | SENSOR3_Pin | SENSOR4_Pin | SENSOR5_Pin | SENSOR6_Pin | SENSOR7_Pin | SENSOR8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef USE_FULL_ASSERT
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
