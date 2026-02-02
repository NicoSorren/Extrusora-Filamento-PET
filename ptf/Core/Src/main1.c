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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7789.h"
#include "fonts.h"
#include "Menu.h"
#include <stdint.h>
#include "Thermistor.h"
#include "Buzzer.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

volatile int32_t previous_counter=0;
volatile int32_t counterx=0;
volatile int32_t countery=0;
volatile int8_t ESW_Flag=0;//Encoder Switch Flag

uint16_t adc_value = 0;  //Variables For Reading The Thermistor
float temperature = 0.0;

volatile uint8_t page=0;//For Navigating The Menu
volatile uint8_t edit_mode = 0;// Flags...
volatile uint8_t START_flag = 0;
volatile uint8_t Heat_flag = 0;
volatile uint8_t Heat_Start_Flag=0;
volatile uint8_t protection_flag=0;

uint16_t Target_Temperature=260; // Memory Temp
uint8_t  Speed=50; // Memory Speed
uint8_t  Buzzer_State=1; // Memory Buzzer State
uint8_t  Switch_State=0; // Memory Switch State

uint8_t END_STOP_FLAG=0;// Filament run out detection flag

PID_TypeDef HeaterPID;
double input_temp, output_pwm, setpoint_temp;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) //Reading Rotary Encoder...
{
    if (htim->Instance == TIM2) {

       int32_t current_counter = __HAL_TIM_GET_COUNTER(htim);
       int32_t delta = current_counter - previous_counter;

		if (delta > 32767) delta -= 65536;
		else if (delta < -32768) delta += 65536;

		counterx += delta;
		countery = counterx/2;
		previous_counter = current_counter;

		if(page==1 && START_flag==0 && Heat_flag==0){
		   if(countery>2){ countery=0; counterx=0;}
		   if(countery<0){ countery=2; counterx=2*2;}
		   mm_selection_indicator(countery);  // Update indicator
		}

		if(page==1 && START_flag==0 && Heat_flag==1){
			if (countery > 1) { countery = 0; counterx = 0; }
			if (countery < 0) { countery = 1; counterx = 1 * 2; }
			mm_selection_indicator(countery); // Update indicator
		}

        if(page==2 && edit_mode==0){
            if (countery > 4) { countery = 0; counterx = 0; }
            if (countery < 0) { countery = 4; counterx = 4 * 2; }
            sm_selection_indicator(countery); // Update indicator
        }
}    }

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //Reading Button...
{

	 if (GPIO_Pin == GPIO_PIN_2) // PA2 = EXTI2
	    {
	        static uint32_t last_press = 0;
	        uint32_t now = HAL_GetTick();

	        if (now - last_press > 200)
	        {
	            ESW_Flag = 1;
	            last_press = now;
	        }
	    }

}

void Stepper_SetSpeed(uint8_t speed) {// Adjust the PWM frequency to control the stepper
    uint32_t freq = speed * 10;// Value we read from the menu * 10
    uint32_t timer_clock = 48000000; // 48 MHz system clock!!
    uint32_t prescaler = 48 - 1;     // 1 MHz timer tick
    uint32_t period = (1000000 / freq) - 1;

    __HAL_TIM_SET_PRESCALER(&htim1, prescaler);
    __HAL_TIM_SET_AUTORELOAD(&htim1, period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, period / 2); // 50% duty
}

void Stepper_SoftStart(uint8_t targetSpeed)// Soft starting stepper
{
    uint8_t currentSpeed = 2;  // starting from very slow
    uint8_t step = 2;           // speed increment per step
    uint8_t delay = 50;         // ms delay between increments

    while (currentSpeed < targetSpeed)
    {
        Stepper_SetSpeed(currentSpeed);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_Delay(delay);
        currentSpeed += step;
    }

    // Final target speed
    Stepper_SetSpeed(targetSpeed);
}

void Heater_SetPower(float duty)// Control the PWM of the MOSFET...
{
    // Clamp duty cycle between 0–100%
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 100.0f) duty = 100.0f;

    // Get the timer period (ARR)
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3);

    // Calculate the compare value based on duty %
    uint32_t compare = (uint32_t)((duty / 100.0f) * period);

    // Apply new PWM duty
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  ST7789_Init();
  opening_melody();// Play the melody while opening
  main_menu();// Call the main menu

  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 1); //Direction of the stepper...
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 1);// Disable stepper driver...

  setpoint_temp = Target_Temperature;   // Desired temp (°C)
  input_temp = temperature;              // Initial temperature reading
  output_pwm = 0.0;                      // Start with no heater power

  PID(&HeaterPID, &input_temp, &output_pwm, &setpoint_temp,
      0.8, 0.006, 0.7,                   // Tune for better results...
      _PID_P_ON_E, _PID_CD_DIRECT);     // Standard direct control

  PID_SetOutputLimits(&HeaterPID, 0, 100);    // PWM % output limit
  PID_SetMode(&HeaterPID, _PID_MODE_AUTOMATIC);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	HAL_Delay(250);


	END_STOP_FLAG = HAL_GPIO_ReadPin(EndSW_GPIO_Port, EndSW_Pin);
	adc_value = Read_ADC_Channel(&hadc);
	temperature = Get_Temperature(adc_value);

	if(page==1){ // Refresh the readings once in a while
	mm_values((uint16_t)temperature, Target_Temperature, Speed);
	}
	if(page==2){ // Refresh the readings once in a while
	settings_values(Target_Temperature, Speed, Buzzer_State, Switch_State);
	}


/*--------------------------------------------------------------------------------*/
// Main Menu Section
/*--------------------------------------------------------------------------------*/// Start Option
	if((ESW_Flag==1 && countery==0 && page==1 && Heat_flag==0 && START_flag==0) || Heat_Start_Flag==1){   //START
		ESW_Flag=0;
		Heat_Start_Flag=0;
		START_flag=1;
		HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 0);// Enable stepper driver
		ST7789_WriteString(60, 135, "Starting", Font_16x26, WHITE, BLACK);
		start_melody();
		ST7789_WriteString(60, 135, "        ", Font_16x26, WHITE, BLACK);
		if(temperature >= Target_Temperature - 10){
		Stepper_SoftStart(Speed);
		}

	}
	if(START_flag==1){
		ST7789_WriteString(88, 135, "Stop ", Font_16x26, WHITE, BLACK);

		adc_value = Read_ADC_Channel(&hadc);
		temperature = Get_Temperature(adc_value);
		mm_values((uint16_t)temperature, Target_Temperature, Speed);

		input_temp = temperature;             // Current temperature
		setpoint_temp = Target_Temperature;   // Target

		if (PID_Compute(&HeaterPID))
		{
		    Heater_SetPower(output_pwm);      // Apply PID output to heater PWM

		}

		if(temperature >= Target_Temperature - 4){ // If the measured temperature is bigger then target only than start the stepper...
		Stepper_SetSpeed(Speed);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		protection_flag=1;
		}

		if((temperature <= Target_Temperature - 15) && protection_flag==1){ // PID values might be too small or there might be a heating problem, stop the stepper...
		protection_flag=0;
		}

		if((END_STOP_FLAG == 0 && Switch_State == 1) || protection_flag==0 ){ // If the filament runs out or nozzle too cold

			START_flag = 0;// Stop the machine when the filament runs out

			__HAL_TIM_SET_COUNTER(&htim2, 0);
			counterx = 0;
			countery = 0;
			previous_counter = 0;
			mm_selection_indicator(countery);

			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);     //Stop stepper
            HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 1);

            Heater_SetPower(0); //Stop heater
            output_pwm = 0.0;
            ST7789_WriteString(80, 135, "Start", Font_16x26, WHITE, BLACK);
            stop_melody();

		}


		if (ESW_Flag == 1) { // If user clicks Stop
			ESW_Flag = 0;
			START_flag = 0;

			__HAL_TIM_SET_COUNTER(&htim2, 0);
			counterx = 0;
			countery = 0;
			previous_counter = 0;
			mm_selection_indicator(countery);

		    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);     //Stop stepper
			HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 1);

			Heater_SetPower(0); //Stop heater
			output_pwm = 0.0;

			ST7789_WriteString(80, 135, "Start", Font_16x26, WHITE, BLACK);
			short_stop_melody();
    }}

/*--------------------------------------------------------------------------------*/// Pre-heat Option
	if(ESW_Flag==1 && countery==1 && page==1 && START_flag==0 && Heat_flag==0){  // Enter Pre-Heat mode
	    ESW_Flag=0;
	    Heat_flag=1;
	    ST7789_WriteString(56, 170, "  ", Font_16x26, WHITE, BLACK);
	    ST7789_WriteString(76, 170, "Cancel ", Font_16x26, WHITE, BLACK);
	    start_melody();
	}
	if (Heat_flag == 1) {

		input_temp = temperature;             // Current temperature
		setpoint_temp = Target_Temperature;   // Target

		if (PID_Compute(&HeaterPID))
		{
			Heater_SetPower(output_pwm);      // Apply PID output to heater PWM
		}

		if(temperature>=Target_Temperature - 4){
			protection_flag=1;
		}

	    if (ESW_Flag == 1 && countery == 0) { // Start
	    	ST7789_WriteString(56, 170, "Pre-Heat", Font_16x26, WHITE, BLACK);
	    	Heat_Start_Flag=1;
	    	ESW_Flag=0;
	        Heat_flag = 0;
	        START_flag = 1;

	        __HAL_TIM_SET_COUNTER(&htim2, 0);
	        counterx = 0;
	        countery = 0;
	        previous_counter = 0;
	        mm_selection_indicator(countery);
	    }

	    if (ESW_Flag == 1 && countery == 1) { // Cancel
	        ESW_Flag = 0;
	        Heat_flag = 0;

	        Heater_SetPower(0); //Stop heater
	        output_pwm = 0.0;

			__HAL_TIM_SET_COUNTER(&htim2, 0);
			counterx = 0;
			countery = 0;
			previous_counter = 0;
			mm_selection_indicator(countery);

	        ST7789_WriteString(56, 170, "Pre-Heat", Font_16x26, WHITE, BLACK);
	        short_stop_melody();
	}}

/*--------------------------------------------------------------------------------*/// Settings Option
	if(ESW_Flag==1 && countery==2 && page==1){  //Settings
	   ESW_Flag=0;
	   START_flag=0;
	   Heat_flag=0;
	   settings_melody();
	   settings_menu();

	   __HAL_TIM_SET_COUNTER(&htim2, 0);  // Reset encoder hardware counter
	   previous_counter = 0;
	   counterx = 0;
	   countery = 0;

    }



/*--------------------------------------------------------------------------------*/
// Settings Menu Section
/*--------------------------------------------------------------------------------*/// Temperature Edit
	if (ESW_Flag == 1 && countery == 0 && page == 2) {  // Temp edit mode
	    ESW_Flag = 0;
	    edit_mode = 1;

	    uint16_t temp_edit = Target_Temperature;  // Start from current
	    int16_t base_counter = countery;           // Remember where encoder was

	    while (edit_mode == 1) {
	    	HAL_Delay(100);
	        // Calculate movement relative to entry position
	        int16_t diff = countery - base_counter;

	        if (diff != 0) {
	            temp_edit += diff;        // apply movement
	            base_counter = countery;  // update baseline
	        }

	        if (temp_edit > 260) temp_edit = 260; // Clamp value to limits
	        if (temp_edit < 180) temp_edit = 180;

	        // Show on screen
	        settings_values(temp_edit, Speed, Buzzer_State, Switch_State);

	        if (ESW_Flag == 1) { // Save if button pressed
	            ESW_Flag = 0;
	            Target_Temperature = temp_edit;
	            edit_mode = 0;
	            counterx = 0;
	            countery = 0;// reset encoder for next use
	}}}

/*--------------------------------------------------------------------------------*/// Speed Edit
	if (ESW_Flag == 1 && countery == 1 && page == 2) {  // Speed edit mode
	    ESW_Flag = 0;
	    edit_mode = 1;

	    uint8_t temp_speed = Speed;       // Start from current Speed
	    int16_t base_counter = countery;  // Remember current encoder position

	    while (edit_mode == 1) {
	        HAL_Delay(100);

	        // Calculate encoder movement
	        int16_t diff = countery - base_counter;

	        if (diff != 0) {
	            temp_speed += diff;        // Adjust speed
	            base_counter = countery;   // Update baseline
	        }

	        // Clamp speed to safe limits
	        if (temp_speed > 100) temp_speed = 100;
	        if (temp_speed < 4)  temp_speed = 4;

	        // Update display live
	        settings_values(Target_Temperature, temp_speed, Buzzer_State, Switch_State);

	        // Save and exit on button press
	        if (ESW_Flag == 1) {
	            ESW_Flag = 0;
	            Speed = temp_speed;
	            edit_mode = 0;
	            counterx = 1*2; // Reset encoder for next use
	            countery = 1;
    }}}

/*--------------------------------------------------------------------------------*/// Buzzer Edit
	if (ESW_Flag == 1 && countery == 2 && page == 2) {  // Buzzer edit
	    ESW_Flag = 0;
	    edit_mode = 1;

	    while (edit_mode == 1) {
	        HAL_Delay(100);

	        // Toggle buzzer with encoder rotation
	        if (countery != 2) {        // user rotated encoder
	            Buzzer_State = !Buzzer_State;
	            countery = 2;           // reset back to its menu slot
	            settings_values(Target_Temperature, Speed, Buzzer_State, Switch_State);
	        }

	        if (ESW_Flag == 1) {// Exit on button press
	            ESW_Flag = 0;
	            edit_mode = 0;
	            counterx = 2*2;
	            countery = 2;
	}}}

/*--------------------------------------------------------------------------------*/// Switch Edit
	if (ESW_Flag == 1 && countery == 3 && page == 2) {  // Switch edit
	    ESW_Flag = 0;
	    edit_mode = 1;

	    while (edit_mode == 1) {
	        HAL_Delay(100);

	        // Toggle switch with encoder rotation
	        if (countery != 3) {        // user rotated encoder
	            Switch_State = !Switch_State;
	            countery = 3;           // reset position
	            settings_values(Target_Temperature, Speed, Buzzer_State, Switch_State);
	        }

	        if (ESW_Flag == 1) {// Exit on button press
	            ESW_Flag = 0;
	            edit_mode = 0;
	            counterx = 4*2;
	            countery = 4;
	 }}}

/*--------------------------------------------------------------------------------*/// Save and go back to main menu
	if(ESW_Flag==1 && countery==4 && page==2){  //Save & Back
	   ESW_Flag=0;
	   settings_melody();
	   main_menu();
	   __HAL_TIM_SET_COUNTER(&htim2, 0);  // Reset encoder hardware counter
	   previous_counter = 0;
	   counterx = 0;
	   countery = 0;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ST7789_CS_Pin|Buzzer_Pin|ST7789_RESET_Pin|ST7789_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_Pin|DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SW_Pin */
  GPIO_InitStruct.Pin = SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EndSW_Pin */
  GPIO_InitStruct.Pin = EndSW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EndSW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ST7789_CS_Pin Buzzer_Pin ST7789_RESET_Pin ST7789_DC_Pin */
  GPIO_InitStruct.Pin = ST7789_CS_Pin|Buzzer_Pin|ST7789_RESET_Pin|ST7789_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_Pin DIR_Pin */
  GPIO_InitStruct.Pin = EN_Pin|DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

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
