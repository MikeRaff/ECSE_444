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
#include "arm_math.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC1_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */

	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc3);

	// signals
	int triangleWave;
	int sawtoothWave;
	float sinWave;
    uint32_t delay = 10;

    int soundState=0;
    int buttonState=0;


    float VREF;
    uint32_t ADCDelay = 200;
    uint16_t V_MEASURED;
    int32_t VREFINT;
    uint16_t tempVoltage;
    float tempCelsius;
    int32_t TS_CAL1;
    int32_t TS_CAL2;
    int32_t TS_DATA;
    int32_t numerator = 110 - 30; // calibration constants from documentation

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

 /*
  * PART 4
  */
	  	  int y =0;

	  	  if (buttonState == 0){
	  		  // turn LED off
	  		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); // idk if this should be set or reset
	  		  // play sound based on a state
	  		  if (soundState == 0 && buttonState == 0){ // triangle wave
	  			  // play triangle sound
	  			  for (uint32_t counter = 0; counter < 255; counter++) {
	  			  	  if (counter < 128){
	  			  		  triangleWave += 2;
	  			  	  }
	  			  	  else{
	  			  		  triangleWave-=2;
	  			  	  }
	  			  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, triangleWave);
	  			  // delay
	  			  int x = 0;
	  			  for (int i = 0; i<1000; i++){ x++; } // gotta make sure this is audible and displays on the DAC thingy
	  			  y = x;
	  			  // poll for buttonPress
	  			  if (HAL_GPIO_ReadPin(BlueButton_GPIO_Port, BlueButton_Pin)){ // idk if this should have an !
	  				  // wait 10 ms for debounce
	  				  HAL_Delay (delay);
	  				  // change state
	  				  buttonState = 1;
	  				  soundState = 1;
	  				  }
	  			  }
	  		  }
	  		  if (soundState == 1 && buttonState == 0){ // sawtooth
	  			  // play sawtooth sound
	  			  for (uint32_t counter = 0; counter < 255; counter++) {
	  				  sawtoothWave = counter % 255;
	  			  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, sawtoothWave);
	  			  // delay
	  			  int x = 0;
	  			  for (int i = 0; i<1000; i++){ x++; }    // gotta make sure this is audible and displays on the DAC thingy
	  			  y = x;
	  			  // poll for buttonPress
	  			  if (HAL_GPIO_ReadPin(BlueButton_GPIO_Port, BlueButton_Pin)){ // idk if this should have an !
	  				  // wait 10 ms for debounce
	  				  HAL_Delay (delay);
	  				  // change state
	  				  buttonState = 1;
	  				  soundState = 2;
	  				  }
	  			  }
	  		  }
//	  		  if (soundState = 2 && buttonState = 0){ // sine
//	  			  // play sineWave sound
//	  			  for (uint32_t counter = 0; counter < 255; counter++) {
//
//	  			  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, sineWave);
//	  			  // delay
//	  			  int x = 0;
//	  			  for (int i = 0; i<1000; i++){ x++; }    // gotta make sure this is audible and displays on the DAC thingy
//	  			  y = x;
//	  			  // poll for buttonPress
//	  			  if (HAL_GPIO_ReadPin(BlueButton_GPIO_Port, BlueButton_Pin)){ // idk if this should have an !
//	  				  // wait 10 ms for debounce
//	  				  HAL_Delay (delay);
//	  				  // change state
//	  				  buttonState = 1;
//	  				  soundState = 0;
//	  				  }
//	  			  }
//	  		  }

	  	  }

	  	  if (buttonState == 1){
	  		  // turn LED ON
	  		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // idk if this should be set or reset
	  		  // play temperature sound
	  		  // wait until buttonPress, then switch to buttonState 0

	          // let the temperature affect the frequency thus changing the sound.
	          // just take the current constant frequency, divide it by approx room temp. Multiply this number times the current temperature. and voila

	  		  // Acquire temperature using adc1 and adc3
	         if(HAL_ADC_PollForConversion(&hadc1, 10000) == HAL_OK) {
	  	 		    V_MEASURED = HAL_ADC_GetValue(&hadc1);
	  	 		    VREFINT = (int32_t) *((uint16_t*) (0x1FFF75AAUL));
	  	 		  	 VREF = 3000.0 * (float) VREFINT / ((float) V_MEASURED);
	  	 		  	  //tempCelsius = __HAL_ADC_CALC_TEMPERATURE(3300, refVoltage, ADC_RESOLUTION_12B);	//does linear interpolation
	  	 	  }
	  	 	  if(HAL_ADC_PollForConversion(&hadc3, 10000) == HAL_OK) {
	  	 	  		tempVoltage = HAL_ADC_GetValue(&hadc3);
	  	 	  		// Acquire data from memory
	  	 			TS_CAL1 = (int32_t) *((uint16_t*) (0x1FFF75A8UL));
	  	 	  TS_CAL2 = (int32_t) *((uint16_t*) (0x1FFF75CAUL));
	  	 	  		// Perform calculations
	  		  	TS_DATA = (int32_t) tempVoltage * VREF / 3000.0 ;
	  	 			int32_t diff = TS_DATA - TS_CAL1;
	  	 			int32_t denominator = TS_CAL2 - TS_CAL1;
	  	 			float scaledDenom = VREF * (float) denominator;
	  	 			float scalar = ((float) numerator) / scaledDenom;
	  	 			// Temperature = ((TS_CAL2_TEMP - TS_CAL1_TEMP) / (TS_CAL2 - TS_CAL1)) * (TS_DATA - TS_CAL1) + 30
	  	 			tempCelsius = scalar * (diff) + 30;
	  	 	  }

	          // Play sound
	  			  for (uint32_t counter = 0; counter < 255; counter++) {
	  			  	  if (counter < 128){
	  			  		  triangleWave += 2;
	  			  	  }
	  			  	  else{
	  			  		  triangleWave-=2;
	  			  	  }
	  			  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, triangleWave);
	  			  // delay
	  			  int x = 0;
	  			  // This is how we will use the sound to vary the frequency.
	              int tempSound = 40;
	              tempSound = tempSound*((int) tempCelsius);
	              // Produce delay
	  			  for (int i = 0; i<tempSound; i++){ x++; } // gotta make sure this is audible and displays on the DAC thingy
	  			  y = x;
	              // Poll for button press
	              if (HAL_GPIO_ReadPin(BlueButton_GPIO_Port, BlueButton_Pin)){ // idk if this should have an !
	  				  // wait 10 ms for debounce
	  				  HAL_Delay (delay);
	  				  // change state
	  				  buttonState = 0;
	  				  soundState = 0;

	  	      }
	          }
	        }
	        }


  // PART 3

//
//	  HAL_Delay(ADCDelay); // necessary?
//	 	  // Acquire ref voltage
//	 	  if(HAL_ADC_PollForConversion(&hadc1, 10000) == HAL_OK) {
//	 		    V_MEASURED = HAL_ADC_GetValue(&hadc1);
//	 		    VREFINT = (int32_t) *((uint16_t*) (0x1FFF75AAUL));
//	 		  	VREF = 3000.0 * (float) VREFINT / ((float) V_MEASURED);
//	 		  	  //tempCelsius = __HAL_ADC_CALC_TEMPERATURE(3300, refVoltage, ADC_RESOLUTION_12B);	//does linear interpolation
//	 	  }
//	 	  if(HAL_ADC_PollForConversion(&hadc3, 10000) == HAL_OK) {
//	 	  		tempVoltage = HAL_ADC_GetValue(&hadc3);
//	 	  		// Acquire data from memory
//	 			TS_CAL1 = (int32_t) *((uint16_t*) (0x1FFF75A8UL));
//	 	  		TS_CAL2 = (int32_t) *((uint16_t*) (0x1FFF75CAUL));
//	 	  		// Perform calculations
//		  		TS_DATA = (int32_t) tempVoltage * VREF / 3000.0 ;
//	 			int32_t diff = TS_DATA - TS_CAL1;
//	 			int32_t denominator = TS_CAL2 - TS_CAL1;
//	 			float scaledDenom = VREF * (float) denominator;
//	 			float scalar = ((float) numerator) / scaledDenom;
//	 			// Temperature = ((TS_CAL2_TEMP - TS_CAL1_TEMP) / (TS_CAL2 - TS_CAL1)) * (TS_DATA - TS_CAL1) + 30
//	 			tempCelsius = scalar * (diff) + 30;
//	 	  }
//


  /* USER CODE END 3 */
}

