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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "lcd.h"
#include "TMC2209.h"

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

TMC2209_HandleTypeDef htmc;
Lcd_HandleTypeDef lcd;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void handle_motor_movement(TMC2209_HandleTypeDef* htmc, uint32_t accel, uint32_t distance, float speed)
{
  while (true)
  {

    for (int i = 0; i <= accel; i++)
    {
      if (! HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin)) return;
      TMC2209_moveAtVelocity(htmc, speed * i / accel);
      HAL_Delay(2);
    }
    if (! HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin)) return;
    TMC2209_moveAtVelocity(htmc, speed);
    for (int j = 0; j < distance; j++)
    {
      if (! HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin)) return;
      HAL_Delay(10);
    }
    if (! HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin)) return;
    for (int i = accel; i > 0; i--)
    {
      if (! HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin)) return;
      TMC2209_moveAtVelocity(htmc, speed * i / accel);
      HAL_Delay(2);
    }
    HAL_Delay(100);

    for (int i = 0; i <= accel; i++)
    {
      if (! HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin)) return;
      TMC2209_moveAtVelocity(htmc, - speed * i / accel);
      HAL_Delay(2);
    }
    if (! HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin)) return;
    TMC2209_moveAtVelocity(htmc, -speed);
    for (int j = 0; j < distance; j++)
    {
      if (! HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin)) return;
      HAL_Delay(10);
    }
    if (! HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin)) return;
    for (int i = accel; i > 0; i--)
    {
      if (! HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin)) return;
      TMC2209_moveAtVelocity(htmc, - speed * i / accel);
      HAL_Delay(2);
    }
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  Lcd_PortType ports[] = {
    GPIOB, GPIOB, GPIOB, GPIOB
  };

  Lcd_PinType pins[] = {LCD_D4_Pin, LCD_D5_Pin, LCD_D6_Pin, LCD_D7_Pin};


  lcd = Lcd_create(ports, pins, GPIOA, LCD_RS_Pin, GPIOA, LCD_EN_Pin, LCD_4_BIT_MODE);

  // Lcd_string(&lcd, "Hello World");
  
  Lcd_cursor(&lcd, 1,0);
  

  // const char* hello = "Hello World\0";


  // HAL_UART_Transmit(&huart2, hello, sizeof(hello), 32767);

  htmc = TMC2209_create(&huart2, SERIAL_ADDRESS_0); // Assuming SERIAL_ADDRESS_0 is the desired address
  TMC2209_init(&htmc);

  
  // TMC2209_setStallGuardThreshold(&htmc, 20);
  
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  TMC2209_disable(&htmc);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t accel = 100;
  uint32_t distance = 100;
  float speed = 5000;
  bool stealthchop = false;

  bool run = false;
  int current = 100;
  bool last_changed;

  // Menu items
  const char* menuItems[] = {
    "accel",
    "distance",
    "speed",
    "stealthchop",
    "current",
    "start"
  };
  const int numMenuItems = sizeof(menuItems) / sizeof(menuItems[0]);

  int selectedMenuItem = 0;
  uint32_t encoderValue = 0;
  uint32_t initialEncoderValue = 0;
  bool adjustingParameter = false;

  while (1)
  {
    if (run)
    {
      Lcd_clear(&lcd);
      Lcd_cursor(&lcd, 0, 0);
      Lcd_string(&lcd, "RUNNING...");
      TMC2209_enable(&htmc);
      if (stealthchop)
      {
        TMC2209_enableStealthChop(&htmc);
      }
      else
      {
        TMC2209_disableStealthChop(&htmc);
      }
      HAL_Delay(200);
      TMC2209_setRunCurrent(&htmc, current);
      HAL_Delay(200);

      handle_motor_movement(&htmc, accel, distance, speed);

      TMC2209_moveAtVelocity(&htmc, 0);
      TMC2209_setRunCurrent(&htmc, 10);
      TMC2209_disable(&htmc);
      run = false;
      while (! HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin)); // Wait for button release
    }
    else
    {
      if (!adjustingParameter)
      {
        // Menu navigation
        encoderValue = (TIM2->CNT) >> 2;
        int menuIndex = (encoderValue - initialEncoderValue) % numMenuItems;
        if (menuIndex < 0) menuIndex += numMenuItems;
        selectedMenuItem = menuIndex;

        // Display menu
        Lcd_clear(&lcd);
        Lcd_cursor(&lcd, 0, 0);
        Lcd_string(&lcd, ">");

        for (int i = 0; i < 2; i++) // Display only two items at a time
        {
          int itemIndex = (selectedMenuItem + i) % numMenuItems;
          Lcd_cursor(&lcd, i, 1);
          Lcd_string(&lcd, menuItems[itemIndex]);
          Lcd_cursor(&lcd, i, 10); // Display values on the right side
          switch (itemIndex)
          {
            case 0: // accel
              Lcd_int(&lcd, accel);
              break;
            case 1: // distance
              Lcd_int(&lcd, distance);
              break;
            case 2: // speed
              Lcd_int(&lcd, speed);
              break;
            case 3: // stealthchop
              if (stealthchop)
              {
                Lcd_string(&lcd, "true");
              }
              else
              Lcd_string(&lcd, "false");
              break;
            case 4: // current
              Lcd_int(&lcd, current);
              break;
            case 5: // start
              Lcd_string(&lcd, "");
              break;
          }
        }

        // Select item
        if (! HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin))
        {
          HAL_Delay(100); // Debounce
          if (selectedMenuItem == numMenuItems - 1) // Start selected
          {
            HAL_Delay(500); // Delay before starting
            run = true;
          }
          else
          {
            adjustingParameter = true;
            initialEncoderValue = (TIM2->CNT) >> 2; // Store initial value for adjustment
          }
          while (! HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin)); // Wait for button release
        }
      }
      else // Adjusting parameter
      {
        encoderValue = (TIM2->CNT) >> 2;
        int diff = encoderValue - initialEncoderValue;

        Lcd_clear(&lcd);
        Lcd_cursor(&lcd, 0, 0);
        Lcd_string(&lcd, menuItems[selectedMenuItem]);
        Lcd_cursor(&lcd, 1, 0);
        Lcd_string(&lcd, "Adjust:");

        switch (selectedMenuItem)
        {
          case 0: // accel
            accel += diff;
            if (accel < 1) accel = 1;
            if (accel > 200) accel = 200;
            Lcd_cursor(&lcd, 1, 10);
            Lcd_int(&lcd, accel);
            break;
          case 1: // distance
            distance += diff;
            if (distance < 10) distance = 10;
            if (distance > 200) distance = 200;
            Lcd_cursor(&lcd, 1, 10);
            Lcd_int(&lcd, distance);
            break;
          case 2: // speed
            speed += diff * 10; // Adjust speed in larger steps
            if (speed < 100) speed = 100;
            if (speed > 10000) speed = 10000;
            Lcd_cursor(&lcd, 1, 10);
            Lcd_int(&lcd, speed);
            break;
          case 3: // stealthchop
            if (diff != 0) // Toggle on any encoder movement
            {
              stealthchop = !stealthchop;
              initialEncoderValue = encoderValue; // Reset initial value after toggle
            }
            Lcd_cursor(&lcd, 1, 10);
            if (stealthchop)
            {
              Lcd_string(&lcd, "true");
            }
            else
            {
              Lcd_string(&lcd, "false");
            }
            break;
          case 4: // current
            current += diff;
            if (current < 2) current = 2;
            if (current > 100) current = 100;
            Lcd_cursor(&lcd, 1, 10);
            Lcd_int(&lcd, current);
            break;
        }

        initialEncoderValue = encoderValue; // Update initial value for continuous adjustment

        // Exit adjustment
        if (! HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin))
        {
          HAL_Delay(100); // Debounce
          adjustingParameter = false;
          initialEncoderValue = (TIM2->CNT) >> 2; // Store initial value for menu navigation
          while (! HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin)); // Wait for button release
        }
      }
      if (!last_changed)
      {
        // last_changed = !((TIM2->CNT) >> 2 == encoderValue && (HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin)));
        while ((TIM2->CNT) >> 2 == encoderValue && (HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin)))
        {
          /* code */
          HAL_Delay(10); // Reduced delay for smoother menu scrolling
        }
      }
      last_changed = !((HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin)));
      
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
