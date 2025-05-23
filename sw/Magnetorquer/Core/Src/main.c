/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "QMC5883.h"
#include "mpu6050.h"
#include "math.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VERSION_STR_LENG  35
#define SAMPLE_FREQ	50
/* Define aliases for timer channels */
#define COIL_Y1 TIM_CHANNEL_2
#define COIL_Y2 TIM_CHANNEL_3
#define COIL_X2 TIM_CHANNEL_4
#define COIL_X1 TIM_CHANNEL_1

#define DEG_TO_RAD 0.017453292519943295769236907685

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;
QMC_t compassStruct;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int _write(int file, char const *buf, int n);
void SetPWMDutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t dutyCycle);
void UpdateAllPWMDutyCycles(uint8_t dutyCycle);
void SetCoilAngle(float angle);
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize all PWM channels */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  //Set all timers to 0% duty cycle
  UpdateAllPWMDutyCycles(0);


  while(QMC_init(&compassStruct, &hi2c1, 200) == 1);
  while (MPU6050_Init(&hi2c1) == 1);

  // Set LED on if successful init
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  static float coilAntiAngle = 0.0f;

	  //Deenergize coils
	  UpdateAllPWMDutyCycles(0);

	  HAL_Delay(100);

	  // Read sensor data
	  MPU6050_Read_All(&hi2c1, &MPU6050);
	  QMC_read(&compassStruct);

	  // Tolerance of x degrees
	  if(fabs(compassStruct.compas) > 10.0){

		  //Compute opposite angle to external field, needs to be remapped since the magnetometer 0° and coil 0° are offset
		  // 360 - (External Angle) + 270 -> deg to rad
		  coilAntiAngle = (630 - compassStruct.heading) * DEG_TO_RAD;

		  //Set coil angle
		  SetCoilAngle(coilAntiAngle);
		  HAL_Delay(500);

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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int _write(int file, char const *buf, int n)
{
	/* stdout redirection to USB CDC */
	//CDC_Transmit_FS((uint8_t*)(buf), n);
	return n;
}

void SetPWMDutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t dutyCycle)
{
    if (dutyCycle > 100) dutyCycle = 100; // Limit duty cycle to 100%
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim); // Get the timer period
    uint32_t pulse = (period * dutyCycle) / 100; // Calculate the pulse width
    __HAL_TIM_SET_COMPARE(htim, channel, pulse); // Set the CCR value
}

void UpdateAllPWMDutyCycles(uint8_t dutyCycle)
{
    // Update duty cycle for TIM1 channels
    SetPWMDutyCycle(&htim1, TIM_CHANNEL_2, dutyCycle);
    SetPWMDutyCycle(&htim1, TIM_CHANNEL_3, dutyCycle);
    SetPWMDutyCycle(&htim1, TIM_CHANNEL_4, dutyCycle);

    // Update duty cycle for TIM2 channel
    SetPWMDutyCycle(&htim2, TIM_CHANNEL_1, dutyCycle);
}

void SetCoilAngle(float angle){

	static int8_t x = 0;	 // X Coil component
	static int8_t y = 0;	 // Y Coil component
	static float DutyMax = 100.0f;	//Maximum duty cycle value

	//Calculate X and Y components
	x = (int8_t) floor(DutyMax * cos(angle));
	y = (int8_t) floor(DutyMax * sin(angle));

	//Polarity handling
	if(x < 0){
		SetPWMDutyCycle(&htim2, COIL_X1, 0);
		SetPWMDutyCycle(&htim1, COIL_X2, (uint8_t) abs(x));
	}
	else{
		SetPWMDutyCycle(&htim1, COIL_X2, 0);
		SetPWMDutyCycle(&htim2, COIL_X1, (uint8_t) abs(x));
	}

	if(y < 0){
		SetPWMDutyCycle(&htim1, COIL_Y1, 0);
		SetPWMDutyCycle(&htim1, COIL_Y2, (uint8_t) abs(y));
	}
	else{
		SetPWMDutyCycle(&htim1, COIL_Y2, 0);
		SetPWMDutyCycle(&htim1, COIL_Y1, (uint8_t) abs(y));
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
