/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID_MODEL.h"
#include "rtwtypes.h"
#include <math.h>
#include "bsp_fdcan.h"
#include "motorctrl.h"
#include "stdio.h"
#include "math.h"
#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"
#include "stdio.h"
#include "string.h"
#include "mpu6050.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern motor_t motor[num];
extern uint8_t CAN_RECEIVE[3];
extern double deadband;
extern motor_measure_t *motor_data_can1[8];
extern motor_measure_t *motor_data_can2[8];
extern motor_measure_t *motor_data_can3[8];

extern IWDG_HandleTypeDef hiwdg1;
extern int Vel_Deadband[3];
extern double tor_output[3*8];
#define CUR      0
#define VEL      1
#define ANG      2

#define PI 3.1415926
#define m6020_2_1_init 5848
#define m6020_2_2_init 3021
#define m6020_2_3_init	3055
uint8_t active_can[10];
uint8_t shoot_flag[10]={0};
uint8_t dri_flag=0;
float tar[2];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

int theta_test[10];

uint8_t uart_index[2] = {0};
uint8_t buffer[100];
char UART2_TX_BUF[100];
uint8_t UART7_RX_BUF[100];
uint8_t UART2_RX_BUF[100];
uint8_t rx_datatemp[8];
int cnt[10];
int PinState=0;
uint8_t usart2_rec=0;
float float1, float2;
uint8_t upper_cnt=0;
float diff[2];
int m6020_init=0;
int flag=0;
int ecd_count=0;
MPU6050_t MPU6050;
float gz_sum,gz_aver;
int cnt_gz=0;
float yaw;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM6_Init();
  MX_FDCAN2_Init();
  MX_USART2_UART_Init();
  MX_FDCAN3_Init();
  MX_USART3_UART_Init();
  MX_FDCAN1_Init();
  MX_UART7_Init();
  MX_IWDG1_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);

	HAL_UART_Receive_IT(&huart2, buffer+1, 1);

//  can_filter_init();
//	bsp_can_init();
  fdcan_filter_init(&hfdcan1);
	fdcan_filter_init(&hfdcan2);
  fdcan_filter_init(&hfdcan3);

	PID_MODEL_initialize();
	
	PID_Angle_S_Para_Init(2, 1 , 10 , 3 , 0.01);
  PID_Angle_A_Para_Init(2, 1 , 0.3 , 0 , 0);
	PID_Angle_S_Para_Init(2, 5 , 70 , 30 , 0.1);
	PID_Angle_A_Para_Init(2, 5 , 0.3 , 0 , 0);

	rtP.DX_P=0.005;//6020
	rtP.DX_I=0;
	rtP.DX_D=0;
	rtP.DY_P=0.01;
	rtP.DY_I=0;
	rtP.DY_D=0;
	
	Set_6020_Mode( 0 );
	set_mode( VEL, VEL, ANG, VEL, VEL, ANG, ANG,
            ANG, VEL, VEL, VEL, CUR, ANG, ANG,
					  ANG, VEL, VEL, VEL, ANG, ANG, ANG ); 

	memset(UART2_TX_BUF,0,sizeof(UART2_TX_BUF));
//	while (MPU6050_Init(&hi2c1) == 1) ;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//		MPU6050_Read_All(&hi2c1, &MPU6050);
//		if(cnt_gz<10000){
//			gz_sum+=MPU6050.Gz;
//			gz_aver=gz_sum/cnt_gz;
//			cnt_gz++;
//		}
		if (motor_data_can2[4]->activate) {
				if (motor_data_can2[4]->ecd == motor_data_can2[4]->last_ecd) {
						ecd_count++;               
				} else {
						ecd_count = 0;            
						motor_data_can2[4]->last_ecd = motor_data_can2[4]->ecd;
				}
\
				if (ecd_count >= 10 && flag == 0) {
						m6020_init = motor_data_can2[4]->ecd + motor_data_can2[4]->circle * 8191;
						rtU.status_CH2_5=ANG;
						flag = 1;                 
				}
		}
		HAL_Delay (1);
		
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//2*F9 2*F0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {

        if (uart_index[1] == 0)
        {
            if (buffer[1] == 0xF9)
            {
                UART2_RX_BUF[uart_index[1]++] = buffer[1];
            }

        }
        else if (uart_index[1] == 1)
        {
            if (buffer[1] == 0xF9)
            {
                UART2_RX_BUF[uart_index[1]++] = buffer[1];
            }
            else
            {
                uart_index[1] = 0;
            }
        }
        else
        {
            UART2_RX_BUF[uart_index[1]++] = buffer[1];
            if (uart_index[1] == 13) 
            {

                uart_index[1] = 0;

                if (UART2_RX_BUF[0] == 0xF9 && UART2_RX_BUF[1] == 0xF9 &&    
                    UART2_RX_BUF[11] == 0xF0 && UART2_RX_BUF[12] == 0xF0) 
                {
										upper_cnt=UART2_RX_BUF[10];
                    memcpy(&float1, &UART2_RX_BUF[2], sizeof(float));
                    memcpy(&float2, &UART2_RX_BUF[6], sizeof(float));
									if(float1!=999)
										rtU.x_diff=float1;
									else
										rtU.x_diff=0;
									if(float2!=999)
										rtU.y_diff=float2;
									else
										rtU.y_diff=0;

									HAL_UART_Transmit_IT(&huart2,UART2_RX_BUF,13);
                }

            }
        }

        HAL_UART_Receive_IT(&huart2, buffer + 1, 1);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

  if (huart->Instance == USART2)
  {
    HAL_UART_Receive_IT(&huart2, buffer, 1);
  }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM6)
  {		
		if(	rtU.status_CH2_5==ANG){
			set_target(2,1,tar[0]);
			set_target(2,5,tar[1]+m6020_init);//6020
		}
    cnt[0]++;
		HAL_IWDG_Refresh(&hiwdg1);//feed the IWG
		diff[0]=-rtY.Y_OUT;
		diff[1]=+rtY.X_OUT;
		
		tar[0]-=rtY.Y_OUT;
		tar[1]+=rtY.X_OUT;

		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,(GPIO_PinState)1);

		
		get_msgn();
		assign_output();
    motor_state_update();
		set_reset_status();
		PID_MODEL_step();
		cnt[1]++;
		if(cnt[1]==500){
			HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_7);
			cnt[1]=0;
		}

  }
	
	 if (htim->Instance == TIM7)
	{
		
		}
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
