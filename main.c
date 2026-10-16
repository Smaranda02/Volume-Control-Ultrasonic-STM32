/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUZZER_PIN GPIO_PIN_6

#define TRIGGER_PIN GPIO_PIN_10
#define TRIG_PORT GPIOA
#define TRIG_DELAY_TIME 10 //10 us

#define ECHO_PORT GPIOA
#define ECHO_PIN GPIO_PIN_8

#define MIN_SENSOR_THRESHOLD 25
#define MIN_ALARM_TRIGGER_TIME 1000
#define MIN_SENSOR_DISTANCE 2

#define ECHO_TIMEOUT_US 25000  // ~4 meters

#define TONES_ARRAY_SIZE 7
#define NEXT_TONE_OFFSET 3

#define BUZZER_BASE_FREQ 3000

uint16_t tones[TONES_ARRAY_SIZE] = {
	2, // 2 cm
	5, // 5cm
    7,
	15,
	20,
	25,
	30 //24
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
volatile uint32_t start_time = 0;
volatile uint32_t end_time   = 0;
typedef enum
{
    ECHO_IDLE = 0,
    ECHO_WAIT_RISING,
    ECHO_WAIT_FALLING,
    ECHO_DONE
} EchoState_t;
volatile EchoState_t echo_state = ECHO_IDLE;

float distance  = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void TIM1_InputCapture_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->PSC = 15;
    TIM1->ARR = 0xFFFF;

    // CH1 input, mapped to TI1
    TIM1->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM1->CCMR1 |=  TIM_CCMR1_CC1S_0;

    // both edges
    TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
    TIM1->CCER |=  (TIM_CCER_CC1P | TIM_CCER_CC1NP);

    // enable capture on channel 1
    TIM1->CCER |= TIM_CCER_CC1E;

    // Enable capture interrupt
    TIM1->DIER |= TIM_DIER_CC1IE;

    // NVIC
    NVIC_SetPriority(TIM1_CC_IRQn, 0);
    NVIC_EnableIRQ(TIM1_CC_IRQn);

    TIM1->CR1 |= TIM_CR1_CEN;
}


void TIM1_CC_IRQHandler(void)
{
    if (TIM1->SR & TIM_SR_CC1IF)
    {
        uint32_t captured = TIM1->CCR1;

        if (echo_state == ECHO_WAIT_RISING)
        {
            start_time = captured;
            echo_state = ECHO_WAIT_FALLING;
        }
        else if (echo_state == ECHO_WAIT_FALLING)
        {
            end_time = captured;
            echo_state = ECHO_DONE;
        }


        TIM1->SR &= ~TIM_SR_CC1IF;
    }
}


void Sensor_Trigger(void)
{
    TIM1->CNT = 0;

    GPIOA->BSRR = (1 << 10);        // TRIG HIGH
    while(TIM1->CNT < TRIG_DELAY_TIME);  // Wait 10µs
    GPIOA->BSRR = (1 << (10 + 16)); // TRIG LOW

    // Wait a bit for the trigger pulse to settle before starting capture
    while(TIM1->CNT < 20);  // Wait 20µs total

    echo_state = ECHO_WAIT_RISING;   // start waiting for echo
}


  void Buzzer_Init(void)
  {
      // enable TIM3 clock
	  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;

	  TIM3->PSC = 15;

      //  PWM mode 1 on Channel 1
      TIM3->CCMR1 &= ~(7 << 4);
      TIM3->CCMR1 |=  (6 << 4);   // PWM mode 1

      // enable preload for CCR1
      TIM3->CCMR1 |= TIM_CCMR1_OC1PE;

      // Enable CH1 output
      TIM3->CCER |= TIM_CCER_CC1E;

      // Enable auto-reload preload
      TIM3->CR1 |= TIM_CR1_ARPE;

      // Start timer
      TIM3->CR1 |= TIM_CR1_CEN;
  }


  void Buzzer_SetFrequency(uint32_t freq)
  {
      if (freq == 0) {
          TIM3->CCR1 = 0;  // silence
          return;
      }

      uint32_t arr = (1000000 / freq) - 1;

      TIM3->ARR  = arr;
      TIM3->CCR1 = arr / 2;   // 50% duty cycle

      TIM3->EGR |= TIM_EGR_UG;  // force update
  }

  void Trigger_Alarm(uint16_t distance){

	  if(distance < MIN_SENSOR_THRESHOLD){ //we will trigger the alarm

		  uint16_t rest = (distance - MIN_SENSOR_DISTANCE) / NEXT_TONE_OFFSET;
		  uint16_t pwm_ratio = tones[rest];
		  Buzzer_SetFrequency(pwm_ratio);

	  }
	  else{

		  Buzzer_SetFrequency(0);

	  }
  }

  float Compute_Distance (uint32_t start_time, uint32_t end_time)
  {
      if (end_time <= start_time) {
          return -1.0f;  // Invalid measurement
      }

      float duration = (float)(end_time - start_time);

      // HC-SR04 should take at least 150µs for ~2.5cm
      if (duration < 100) {
          return -1.0f;  // Too short, likely noise
      }

      float distance = duration * 0.034f / 2.0f;
      return distance;
  }


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */


int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  TIM1_InputCapture_Init();  //my version of MX_TIM
  MX_USART2_UART_Init();
  Buzzer_Init();   // TIM3 init
  /* USER CODE BEGIN 2 */

  char msg[40];
  int len = sprintf(msg, "System started!\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);

  static uint32_t last_trigger = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
	    if (HAL_GetTick() - last_trigger >= 100) {  // Trigger every 100ms
	        Sensor_Trigger();
	        last_trigger = HAL_GetTick();
	    }

	    if (echo_state == ECHO_DONE)
	    {
	        distance = Compute_Distance(start_time, end_time);
	        echo_state = ECHO_IDLE;

	        if (distance > 0) {
	            char msg[50];
	            int len = sprintf(msg, "Distance: %.2f cm\r\n", distance);
	            HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
	            Trigger_Alarm(distance);
	        }
	    }
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


/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
//
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
//
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
//
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;   // alternate function push-pull
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;  // TIM1_CH1
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  /*Configure GPIO pins : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Configure PA6 as TIM3_CH1 (PWM output for buzzer) */
   GPIO_InitStruct.Pin = GPIO_PIN_6;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */


}

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
