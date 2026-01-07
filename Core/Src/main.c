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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "pid_loop.h"
#include "stm32f4xx_hal.h"
#include "ITM_Printf.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AVG_WINDOW_SIZE 5   // Number of samples for rolling average
#define NUM_MOTORS 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for Reading_Task */
osThreadId_t Reading_TaskHandle;
const osThreadAttr_t Reading_Task_attributes = {
  .name = "Reading_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PID_Task */
osThreadId_t PID_TaskHandle;
const osThreadAttr_t PID_Task_attributes = {
  .name = "PID_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Init_Task */
osThreadId_t Init_TaskHandle;
const osThreadAttr_t Init_Task_attributes = {
  .name = "Init_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal6,
};
/* Definitions for MotorTx_Task */
osThreadId_t MotorTx_TaskHandle;
const osThreadAttr_t MotorTx_Task_attributes = {
  .name = "MotorTx_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for Set_Task */
osThreadId_t Set_TaskHandle;
const osThreadAttr_t Set_Task_attributes = {
  .name = "Set_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Encoder_Queue */
osMessageQueueId_t Encoder_QueueHandle;
const osMessageQueueAttr_t Encoder_Queue_attributes = {
  .name = "Encoder_Queue"
};
/* Definitions for SetQueue */
osMessageQueueId_t SetQueueHandle;
const osMessageQueueAttr_t SetQueue_attributes = {
  .name = "SetQueue"
};
/* Definitions for EncoderTimer */
osTimerId_t EncoderTimerHandle;
const osTimerAttr_t EncoderTimer_attributes = {
  .name = "EncoderTimer"
};
/* Definitions for setMutex */
osMutexId_t setMutexHandle;
const osMutexAttr_t setMutex_attributes = {
  .name = "setMutex"
};
/* Definitions for TxMutex */
osMutexId_t TxMutexHandle;
const osMutexAttr_t TxMutex_attributes = {
  .name = "TxMutex"
};
/* Definitions for setPointEvent */
osEventFlagsId_t setPointEventHandle;
const osEventFlagsAttr_t setPointEvent_attributes = {
  .name = "setPointEvent"
};
/* Definitions for speedChangeEvent */
osEventFlagsId_t speedChangeEventHandle;
const osEventFlagsAttr_t speedChangeEvent_attributes = {
  .name = "speedChangeEvent"
};
/* USER CODE BEGIN PV */
//static float SAMPLE_TIME;
//static int16_t err_prev = 0;
//static int16_t err_prev2 = 0;
//static float integral_sum = 0.0f;
//static float pid_output = 0.0f;

float motor_setpoint = 0; // shared
float pwm_outputs[4];  // updated in PID task or PWM task

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void Start_Sensor_Reading_Task(void *argument);
void Start_PID_Task(void *argument);
void Start_Init_Task(void *argument);
void Start_MotorTx_Task(void *argument);
void Start_set_Task(void *argument);
void EncoderTimerCallback(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct{
	uint8_t motorId; //speed of motor 1-4
	uint16_t speed;
} EncoderData;

typedef struct {
    float samples[AVG_WINDOW_SIZE];
    float sum;
    uint8_t index;
    uint8_t count;
} RollingAvg;

RollingAvg_t rpm_averagers[NUM_MOTORS];  // One for each motor
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of setMutex */
  setMutexHandle = osMutexNew(&setMutex_attributes);

  /* creation of TxMutex */
  TxMutexHandle = osMutexNew(&TxMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of EncoderTimer */
  EncoderTimerHandle = osTimerNew(EncoderTimerCallback, osTimerPeriodic, NULL, &EncoderTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Encoder_Queue */
  Encoder_QueueHandle = osMessageQueueNew (16, sizeof(EncoderData), &Encoder_Queue_attributes);

  /* creation of SetQueue */
  SetQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &SetQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Reading_Task */
  Reading_TaskHandle = osThreadNew(Start_Sensor_Reading_Task, NULL, &Reading_Task_attributes);

  /* creation of PID_Task */
  PID_TaskHandle = osThreadNew(Start_PID_Task, NULL, &PID_Task_attributes);

  /* creation of Init_Task */
  Init_TaskHandle = osThreadNew(Start_Init_Task, NULL, &Init_Task_attributes);

  /* creation of MotorTx_Task */
  MotorTx_TaskHandle = osThreadNew(Start_MotorTx_Task, NULL, &MotorTx_Task_attributes);

  /* creation of Set_Task */
  Set_TaskHandle = osThreadNew(Start_set_Task, NULL, &Set_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of setPointEvent */
  setPointEventHandle = osEventFlagsNew(&setPointEvent_attributes);

  /* creation of speedChangeEvent */
  speedChangeEventHandle = osEventFlagsNew(&speedChangeEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int16_t setValues [8];

void setnewPWM(EncoderData data){
	// convert the new pid output to pwm signal and communicate via CAN
	ITM_SendChar(data.motorId);
	TaskFunction(data.speed + 48); // 48 is for the ASCII equivalent of the number
}

void TaskFunction(char number){
	printf("%c\n",number);
}

float updateRollingAverage(uint8_t motor_id, float new_rpm)
{
    RollingAvg *avg = &rpm_averagers[motor_id];

    // Subtract oldest value, add new value
    avg->sum -= avg->samples[avg->index];
    avg->samples[avg->index] = new_rpm;
    avg->sum += new_rpm;

    // Advance index
    avg->index = (avg->index + 1) % AVG_WINDOW_SIZE;

    // Track sample count (only needed during initial fill)
    if (avg->count < AVG_WINDOW_SIZE)
        avg->count++;

    return avg->sum / avg->count;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_Sensor_Reading_Task */
/**
  * @brief  Function implementing the Sensor_Reading_ thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Sensor_Reading_Task */
void Start_Sensor_Reading_Task(void *argument)
{
  /* USER CODE BEGIN 5 */
	EncoderData data;
  /* Infinite loop */
  for(;;)
  {
	if(osThreadFlagsWait(0x51,	osFlagsWaitAny, osWaitForever)){ // Wait for signal to start running
		TaskFunction('W');
	}

	for(uint8_t i=0; i < 4; i++){
		// Get PWM value of the encoders and convert them.

		float filtered_rpm = updateRollingAverage(i, raw_rpm);
		EncoderData msg = { .motorId = i, .speed = filtered_rpm };

		if(osMessageQueuePut(Encoder_QueueHandle, &i, 0, 0) == osOK){ // add converted data to the queue
			TaskFunction('A');
		}
	}
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_PID_Task */
/**
* @brief Function implementing the PID_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_PID_Task */
void Start_PID_Task(void *argument)
{
  /* USER CODE BEGIN Start_PID_Task */
	EncoderData encData;
	float velocity_ideal;
	float output;

	//osMessageQueueGet(SetQueueHandle, &velocity_ideal, NULL, 30); // get the new set value from the queue within 30ms. If not gotten within allowed time, continue.

  /* Infinite loop */
  for(;;)
  {

	if(osMessageQueueGet(Encoder_QueueHandle, &encData, NULL, osWaitForever) == osOK){
		osMutexAcquire(setMutexHandle, osWaitForever);
		velocity_ideal = motor_setpoints[encData.motorId]; // it should be a single setpoint
		osMutexRelease(setMutexHandle);

		TaskFunction('D');

		//output = setnewPWM(encData);
	}else{
		TaskFunction('N');
	}

	osMutexAcquire(TxMutexHandle, osWaitForever);
	pwm_outputs[encData.motorId] = output;
	osMutexRelease(TxMutexHandle);

	osDelay(10);

  }
  /* USER CODE END Start_PID_Task */
}

/* USER CODE BEGIN Header_Start_Init_Task */
/**
* @brief Function implementing the Init_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Init_Task */
void Start_Init_Task(void *argument)
{
  /* USER CODE BEGIN Start_Init_Task */
	osTimerStart(EncoderTimerHandle, 50);
  /* Infinite loop */
  for(;;)
  {
    TaskFunction('!');
    osThreadTerminate(Init_TaskHandle);
  }
  /* USER CODE END Start_Init_Task */
}

/* USER CODE BEGIN Header_Start_MotorTx_Task */
/**
* @brief Function implementing the MotorTx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_MotorTx_Task */
void Start_MotorTx_Task(void *argument)
{
  /* USER CODE BEGIN Start_MotorTx_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_MotorTx_Task */
}

/* USER CODE BEGIN Header_Start_set_Task */
/**
* @brief Function implementing the Set_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_set_Task */
void Start_set_Task(void *argument)
{
  /* USER CODE BEGIN Start_set_Task */
	uint8_t setCount = 0;
	uint8_t totalValue = 0;
	uint8_t size = 0;
	uint8_t avgSetpoint = 0;
	int value = 0;
  /* Infinite loop */
  for(;;)
  {
    // rolling average code
	if(setCount<8){
		setValues[setCount] = value;
		setCount++;
		HAL_Delay(50);
	}
	else if(setCount == 8){
		setValues[size%8] = value;
		size++;

		for(int i=0; i<8; i++){
			totalValue += setValues[i];
		}

		avgSetpoint = totalValue/8;
	}

	osMutexAcquire(setMutexHandle, osWaitForever);
	motor_setpoint = avgSetpoint;
	osMutexRelease(setMutexHandle);

	totalValue = 0;
  }
  /* USER CODE END Start_set_Task */
}

/* EncoderTimerCallback function */
void EncoderTimerCallback(void *argument)
{
  /* USER CODE BEGIN EncoderTimerCallback */
	TaskFunction('S');
	osThreadFlagsSet(Reading_TaskHandle, 0x51);
  /* USER CODE END EncoderTimerCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
