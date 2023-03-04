/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define NUMBER_OF_PHOTORESISTORS 5

#define DELAY_BETWEEN_MEASURMENT 1000
// In degrees * 10
#define ANGLE_STEP 100

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const float ANGLE_BETWEEN_PHOTORESISTORS = 22.5;
volatile int secAfterLastMeasurment = 0;
int currentPos = 0;
int lastPos = 0;
bool calibrated = false;

struct Measurment{
  	int readings[NUMBER_OF_PHOTORESISTORS];
  	int sum;
	};

	// create 2 instances of Measurment
	struct Measurment Last;
	struct Measurment AtNewPosiotion;

	int arr[NUMBER_OF_PHOTORESISTORS];// ADC pins
    // PA5
    // PA6
    // PA7
    // PB0
    // PB1

    int measure() {
    	HAL_ADC_Start(&hadc1);
    	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    	return HAL_ADC_GetValue(&hadc1);
    }

    void sumReadings(struct Measurment* measurment) {
        	measurment -> sum = 0;

        	for(int i = 0; i < NUMBER_OF_PHOTORESISTORS; i++) {
        		measurment -> sum += measurment -> readings[i];
        	}
        }

    void getAllReadings(struct Measurment* measurment) {
    	// clear readings
    		for(int i = 0; i < NUMBER_OF_PHOTORESISTORS; i++) {
    					measurment -> readings[i] = 0;
    				}

			for(int i = 0; i < NUMBER_OF_PHOTORESISTORS; i++) {
				measurment -> readings[i] = measure();
			}
       }

    void setAngleSmoothly(uint16_t ang, uint8_t servoIdentifier) {

    	if (ang > lastPos) {
    		for(int i = lastPos; i < ang; i += ANGLE_STEP / 10) {
    			setAngle(i, servoIdentifier);
    			HAL_Delay(50);
    		}
    	}
    	else {
    		for(int i = lastPos; i >= ang; i -= ANGLE_STEP / 10) {
    			setAngle(i, servoIdentifier);
    			HAL_Delay(50);
    		}
    	}

    	lastPos = ang;
    }

    int findBestYAngle(struct Measurment* measurment) {
    	int best = 0;
    	int second = 0;

    	// find best 2 positions
        	for(int i = 0; i < NUMBER_OF_PHOTORESISTORS; i++) {
        		if (measurment -> readings[i] > best) {
        			best = i;
        		}
        	}

        	for(int i = 0; i < NUMBER_OF_PHOTORESISTORS; i++) {
        		if (i == best)
        			continue;
        		if (measurment -> readings[i] > best)
        			second = i;
        	}

        // calculate mean of its positions
        	best *= ANGLE_BETWEEN_PHOTORESISTORS;
        	second *= ANGLE_BETWEEN_PHOTORESISTORS;

        	int angle =  10 * (best + second) / 2;

        	if (angle > ANGLE_MAX / 2)
        		return angle;

        	return ANGLE_MAX - ANGLE_MAX / 4;
        }

    void findNewPosition() {

		while(1) {
		  getAllReadings(&Last);
		  sumReadings(&Last);
		  HAL_Delay(DELAY_BETWEEN_MEASURMENT);

		  setAngle(currentPos + ANGLE_STEP, MEAS_SERVO);
		  getAllReadings(&AtNewPosiotion);
		  sumReadings(&AtNewPosiotion);
		  HAL_Delay(DELAY_BETWEEN_MEASURMENT);

		  if(AtNewPosiotion.sum < Last.sum){
			  // wróć na starą pozycję
			  setAngle(currentPos, MEAS_SERVO);
			  setAngleSmoothly(currentPos, X_SERVO);
			  HAL_Delay(DELAY_BETWEEN_MEASURMENT);
			  setAngle(findBestYAngle(&Last), Y_SERVO);

			  break;
			  }
		  currentPos += ANGLE_STEP;
		}
    }

    // function to get number of measure servo position
    int getMeasPositionNumber() {
    	int counter = 0;
    	for(int i = ANGLE_MIN; i <= ANGLE_MAX; i+= ANGLE_STEP) counter++;
    	return counter;
	 }


    void calibrate() {

    	struct Measurment meas[getMeasPositionNumber()];
    	int counter = 0;

    	for(int i = ANGLE_MIN; i <= ANGLE_MAX; i+= ANGLE_STEP) {
	  		  setAngle(i, MEAS_SERVO);
	  		  getAllReadings(&meas[counter]);
	  		  sumReadings(&meas[counter]);
	  		  HAL_Delay(DELAY_BETWEEN_MEASURMENT);
	  		  counter++;
	  	 }

    	// find biggest sum
    	int bestResult = 0;
    	for(int i = 1; i <= counter - 1; i++) {
		  if (meas[bestResult].sum < meas[i].sum) bestResult = i;
    	}

    	HAL_Delay(DELAY_BETWEEN_MEASURMENT);
    	currentPos = bestResult * ANGLE_STEP;

    	setAngleSmoothly(currentPos, X_SERVO);
    	HAL_Delay(DELAY_BETWEEN_MEASURMENT);
    	setAngle(currentPos, MEAS_SERVO);

    	setAngle(findBestYAngle(&meas[bestResult]), Y_SERVO);
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // timer do odliczania czasu przerwania
  HAL_TIM_Base_Start_IT(&htim2);

  	// X servo
    HAL_TIM_PWM_Start(&htim10, 	TIM_CHANNEL_1); // PB8

    // Y servo
    HAL_TIM_PWM_Start(&htim4, 	TIM_CHANNEL_2); // PB7
    // od 900 do max

    // Measurment servo
    HAL_TIM_PWM_Start(&htim1, 	TIM_CHANNEL_3); // PA10

    calibrate();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (secAfterLastMeasurment >= 45) {
	  			findNewPosition();
	  			secAfterLastMeasurment = 0;
	  		}
	  // ----------------------------------- Poruszanie 2 serwami -----------------------------------------


//	  	setAngleSmoothly(ANGLE_MIN + 9 * ANGLE_STEP, X_SERVO);
//	  	HAL_Delay(2000);
//	  	setAngle(ANGLE_MIN + ANGLE_STEP, X_SERVO);
//	  		  	HAL_Delay(2000);


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
      if (GPIO_Pin == BT1_Pin) {
    	  calibrate();
    	  HAL_Delay(3000);
      }
    }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2) {
		++secAfterLastMeasurment;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
