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


  HAL_Init();



  SystemClock_Config();

  PeriphCommonClock_Config();



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

    uint32_t ADCDelay = 200;

    uint16_t V_MEASURED;

    uint16_t tempVoltage;

    float tempCelsius;

    int soundState=0;

    int buttonState=0;

    float VREF;


  /* USER CODE END 2 */

  /* Infinite loop */

  /* USER CODE BEGIN WHILE */

  while (1)

  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

// PART 1
 if (!HAL_GPIO_ReadPin(BlueButton_GPIO_Port, BlueButton_Pin)){

      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

      }
  else{

      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  }


 int y =0;

// PART 4 

 if(buttonState = 0){
 // turn LED off
 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
 // play sound based on a state
 if (soundState = 0 && buttonState = 0){ // triangle wave

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
 if (HAL_GPIO_ReadPin(BlueButton_GPIO_Port, BlueButton_Pin)){
 // wait 10 ms for debounce
 HAL_Delay (delay);
 // change state
 buttonState = 1;
 soundState = 1;
 }
 }
 }

 if (soundState = 1 && buttonState = 0){ // sawtooth

 // play sawtooth sound
 for (uint32_t counter = 0; counter < 255; counter++) {
 sawtoothWave = counter % 255;
 HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, sawtoothWave);

 // delay
 int x = 0;
 for (int i = 0; i<1000; i++){ x++; }    // gotta make sure this is audible and displays on the DAC thingy
 y = x;

 // poll for buttonPress
 if (HAL_GPIO_ReadPin(BlueButton_GPIO_Port, BlueButton_Pin)){

 // wait 10 ms for debounce
 HAL_Delay (delay);

 // change state
 buttonState = 1;
 soundState = 2;
 }
 }
 }

 if (soundState = 2 && buttonState = 0){ // sine

 // play sineWave sound
 for (uint32_t counter = 0; counter < 255; counter++) {
 sawtoothWave = counter % 255;
 HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, sawtoothWave);

 // delay
 int x = 0;
 for (int i = 0; i<1000; i++){ x++; }    // gotta make sure this is audible and displays on the DAC thingy
 y = x;

 // poll for buttonPress
 if (HAL_GPIO_ReadPin(BlueButton_GPIO_Port, BlueButton_Pin)){

 // wait 10 ms for debounce
 HAL_Delay (delay);

 // change state
 buttonState = 1;
 soundState = 0;
 }
 }
 }

 }

 if (buttonState = 1){
 
 // turn on LED

 // play sound based on temperature

 // wait until buttonPress, then change the state

   // iow, wait 10 ms for bounce, change buttonState to 0

 }





 /*

  * PART 3... doesn't work yet.

  */


 HAL_Delay(ADCDelay);

 // Acquire ref voltage

 if(HAL_ADC_PollForConversion(&hadc1, 10000) == HAL_OK) {

  V_MEASURED = HAL_ADC_GetValue(&hadc1);

  int32_t VREFINT = (int32_t) *((uint16_t*) (0x1FFF75AA)); // TS_DATA   // UL

  VREF = 3.0 * (float) VREFINT / ((float) V_MEASURED);

 }


 HAL_ADC_Start(&hadc3);

 if(HAL_ADC_PollForConversion(&hadc3, 10000) == HAL_OK) {

  tempVoltage = HAL_ADC_GetValue(&hadc3);

  // Acquire data from memory
int32_t TS_CAL1 = (int32_t) *((uint16_t*) (0x1FFF75A8)); // UL
int32_t TS_CAL2 = (int32_t) *((uint16_t*) (0x1FFF75CA)); // UL

  // Perform calculations

int32_t TS_DATA = (int32_t) tempVoltage; // casting
int32_t diff = TS_DATA - TS_CAL1;
int32_t numerator = 110 - 30; // calibration temperature constants
int32_t denominator = TS_CAL2 - TS_CAL1;
float scalar = ((float) numerator) / scaledDenom;

// Temperature = ((TS_CAL2_TEMP - TS_CAL1_TEMP) / (TS_CAL2 - TS_CAL1)) * (TS_DATA - TS_CAL1) + 30
tempCelsius = scalar * (diff) + 30;

 } 

 HAL_ADC_Stop(&hadc1);
 HAL_ADC_Stop(&hadc3);

  /* USER CODE END 3 */

}}


