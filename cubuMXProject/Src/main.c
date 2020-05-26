/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ADXL355.h"
#include "EKF.h"
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
uint32_t dTime = 0;
uint8_t gb_calibrate_zero = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/***********************【Debug Info】**************************/
void send_debug_info(float pitch, float roll, float err_x, float err_y, float err_z, float dtime)
{
  const size_t data_length = 1 + 6 * sizeof(float) + 2;
  uint8_t data[data_length];
  data[0] = 0xa5;
  data[data_length - 2] = 0;
  data[data_length - 1] = 0x5a;

  memcpy(data + 1 + 0 * sizeof(float), (uint8_t *)(&pitch), sizeof(float));
  memcpy(data + 1 + 1 * sizeof(float), (uint8_t *)(&roll), sizeof(float));
  memcpy(data + 1 + 2 * sizeof(float), (uint8_t *)(&err_x), sizeof(float));
  memcpy(data + 1 + 3 * sizeof(float), (uint8_t *)(&err_y), sizeof(float));
  memcpy(data + 1 + 4 * sizeof(float), (uint8_t *)(&err_z), sizeof(float));
  memcpy(data + 1 + 5 * sizeof(float), (uint8_t *)(&dtime), sizeof(float));

  for (size_t i = 1; i < data_length - 2; i++)
  {
    data[data_length - 2] += data[i];
  }

  HAL_UART_Transmit(&huart1, data, data_length, 0xFFFF);
}
/***********************【Debug Info】**************************/

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
  /* USER CODE BEGIN 2 */
  ADXL355_Start_Sensor();
  float acc[3] = {0, 0, 0};
  const int avg_times = 10;
  for (int i = 0; i < avg_times; i++)
  {
    HAL_Delay(100);
    ADXL355_Data_Scan();
    acc[0] += i32SensorX / ADXL_SENSITIVITY;
    acc[1] += i32SensorY / ADXL_SENSITIVITY;
    acc[2] += i32SensorZ / ADXL_SENSITIVITY;
  }

  for (int i = 0; i < 3; i++)
  {
    acc[i] = acc[i] / avg_times;
  }
  EKFInit(acc);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  int i = 0;
  while (1)
  {
    i++;
    HAL_Delay(5);
    if (i % 10 == 5)
      HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_SET);
    else if (i % 10 == 0)
      HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_RESET);

    // ADXL355_Data_Scan();
    // EKFPredict();
    // EKFMeasure(i32SensorX / ADXL_SENSITIVITY, i32SensorY / ADXL_SENSITIVITY, i32SensorZ / ADXL_SENSITIVITY);
    send_debug_info(RAD2DEG(gX_hat[0]), RAD2DEG(gX_hat[1]), gErr[0][0], gErr[1][0], gErr[2][0], dTime);

    //接收处理校零指令
    uint8_t received_data[4];
    HAL_StatusTypeDef ret = HAL_UART_Receive(&huart1, received_data, 4, 10);
    if (ret == HAL_OK && gb_calibrate_zero == 0)
    {
      if (received_data[0] == 0xa5 && received_data[3] == 0x5a && received_data[1] == received_data[2])
      {
        gb_calibrate_zero = received_data[1];
      }
    }

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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == IMU_DRDYP_Pin)
  {
    static uint32_t pre_tick = 0;
    uint32_t cur_tick = HAL_GetTick();
    dTime = cur_tick - pre_tick;
    pre_tick = cur_tick;
    ADXL355_Data_Scan();

    // 缓存最近3s的加速度数据，为校零做准备
    uint32_t idx = calibrate_buffer_id % CALIB_BUFFER_SIZE;
    acc_calibrate_buffer[0][idx] = i32SensorX;
    acc_calibrate_buffer[1][idx] = i32SensorY;
    acc_calibrate_buffer[2][idx] = i32SensorZ;
    calibrate_buffer_id++;
    if (calibrate_buffer_id >= CALIB_BUFFER_SIZE && gb_calibrate_zero)
    {
      int64_t acc_sums[3] = {0};
      double gg = 0;
      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < CALIB_BUFFER_SIZE; j++)
        {
          acc_sums[i] += acc_calibrate_buffer[i][j];
        }
        acc_sums[i] /= CALIB_BUFFER_SIZE;
        g_acc_offsets[i] = acc_sums[i] / ADXL_SENSITIVITY;
        gg += g_acc_offsets[i] * g_acc_offsets[i];
      }
      gG = sqrt(gg);
      gb_calibrate_zero = 0;
    }

    EKFPredict();
    EKFMeasure(i32SensorX / ADXL_SENSITIVITY, i32SensorY / ADXL_SENSITIVITY, i32SensorZ / ADXL_SENSITIVITY);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
