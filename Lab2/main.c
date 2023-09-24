/* USER CODE BEGIN Header */

/**

  ******************************************************************************

  * @file           : main.c

  * @brief          : Main program body

  ******************************************************************************

  * @attention

  *

  * Copyright (c) 2023 STMicroelectronics.

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



/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#define ARM_MATH_CM4

// #include "arm_math.h"



#include <math.h>

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

DAC_HandleTypeDef hdac1;



/* USER CODE BEGIN PV */



/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_DAC1_Init(void);

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



  /* Initialize all configured peripherals // DAC APB clock must be enabled to get write access to DAC registers using HAL_DAC_Init() */

  MX_GPIO_Init();

  MX_DAC1_Init();



  /* USER CODE BEGIN 2 */

  /*

   *  HAL_DAC_Start (DAC_HandleTypeDef * hdac, uint32_t Channel)

   *  Enables DAC and starts conversion of channel

   *  Assign each signal to a different DAC output channel

   */

  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1); // Enable the DAC channel using HAL_DAC_Start()

  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2); //

  //HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, counter);



  int counter = 0;

  // signals

  int triangleWave;

  int sawtoothWave;

  uint32_t delay = 0.00005;







  /* USER CODE END 2 */



  /* Infinite loop */

  /* USER CODE BEGIN WHILE */

  while (1)

  {





    /* USER CODE END WHILE */



    /* USER CODE BEGIN 3 */

 /*

  * PART 1

  */

 // Toggling the LED using a button

 if (!HAL_GPIO_ReadPin(BlueButton_GPIO_Port, BlueButton_Pin)){

 //void HAL_GPIO_WritePin (GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)

 //void HAL_GPIO_TogglePin (GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin)

 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

// HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

 }

 else{

 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

 }





 /*

  * PART 2

  */



 /*

  * Previously, we have read the state of a button, and written the state of an LED.

  * Now we need to initialize and write the state of the DAC to generate signals in

  * an audible frequency range (so we can verify the system with a small speaker).

  *

  * The value we will write to the DAC is a simple unsigned integer incrememented on

  * every iteration of the while loop. (reset it to zero when it reaches the max val) 4095 or 255

  * depending on 8 or 12 bit precision

  */



 // DACValue = DACValue % max;

 // HAL_DAC_SetValue (DAC_HandleTypeDef * hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data)

 /*

  *  Parameters:

  *  hdac: pointer to a DAC_HandleTypeDef structure that contains the configuration information

       for the specified DAC.

  *  Channel: The selected DAC channel. This parameter can be one of the following values:

– DAC_CHANNEL_1: DAC Channel1 selected

– DAC_CHANNEL_2: DAC Channel2 selected

 Alignment: Specifies the data alignment. This parameter can be one of the following values:

– DAC_ALIGN_8B_R: 8bit right data alignment selected

– DAC_ALIGN_12B_L: 12bit left data alignment selected

– DAC_ALIGN_12B_R: 12bit right data alignment selected     // integers should be right aligned

 Data: Data to be loaded in the selected data holding register.

  *

  */



 for (uint32_t counter = 0; counter < 255; counter++) {

   if (counter < 128){

   triangleWave++;



   }

   else{

   triangleWave--;



   }

   sawtoothWave = counter % 255;

   HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, triangleWave);

   HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, sawtoothWave);

   HAL_Delay (delay);

 }



 triangleWave = 0;

 sawtoothWave = 0;





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

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)

  {

    Error_Handler();

  }



  /** Initializes the RCC Oscillators according to the specified parameters

  * in the RCC_OscInitTypeDef structure.

  */

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;

  RCC_OscInitStruct.MSIState = RCC_MSI_ON;

  RCC_OscInitStruct.MSICalibrationValue = 0;

  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;

  RCC_OscInitStruct.PLL.PLLM = 1;

  RCC_OscInitStruct.PLL.PLLN = 40;

  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;

  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;

  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

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

  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;



  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)

  {

    Error_Handler();

  }

}



/**

  * @brief DAC1 Initialization Function

  * @param None

  * @retval None

  */

static void MX_DAC1_Init(void)

{



  /* USER CODE BEGIN DAC1_Init 0 */



  /* USER CODE END DAC1_Init 0 */



  DAC_ChannelConfTypeDef sConfig = {0};



  /* USER CODE BEGIN DAC1_Init 1 */



  /* USER CODE END DAC1_Init 1 */



  /** DAC Initialization

  */

  hdac1.Instance = DAC1;

  if (HAL_DAC_Init(&hdac1) != HAL_OK)

  {

    Error_Handler();

  }



  /** DAC channel OUT1 config

  */

  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;

  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;

  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;

  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;

  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)

  {

    Error_Handler();

  }



  /** DAC channel OUT2 config

  */

  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)

  {

    Error_Handler();

  }

  /* USER CODE BEGIN DAC1_Init 2 */



  /* USER CODE END DAC1_Init 2 */



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

  __HAL_RCC_GPIOC_CLK_ENABLE();

  __HAL_RCC_GPIOA_CLK_ENABLE();

  __HAL_RCC_GPIOB_CLK_ENABLE();



  /*Configure GPIO pin Output Level */

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);



  /*Configure GPIO pin : BlueButton_Pin */

  GPIO_InitStruct.Pin = BlueButton_Pin;

  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(BlueButton_GPIO_Port, &GPIO_InitStruct);



  /*Configure GPIO pin : LED_Pin */

  GPIO_InitStruct.Pin = LED_Pin;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);



/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */

}



/* USER CODE BEGIN 4 */

//while(1) {









//}

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