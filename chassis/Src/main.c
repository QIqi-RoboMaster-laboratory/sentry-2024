/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"


#include "stdbool.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "bsp_delay.h"
#include "bsp_usart.h"
#include "remote_control.h"

#include "chassis_task.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "INS_task.h"
#include "led_flow_task.h"
#include "oled_task.h"
#include "referee_usart_task.h"
#include "voltage_task.h"
#include "stm32.h"
#include "stm32_private.h"
#include "BMI088driver.h"
#include "bsp_dwt.h"

#include "SEGGER_RTT.h" 
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

uint8_t arxbuffer[24];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t upbuffers[SEGGER_RTT_MAX_NUM_UP_BUFFERS-1][BUFFER_SIZE_UP];	//up buffers
uint8_t downbuffers[SEGGER_RTT_MAX_NUM_DOWN_BUFFERS-1][BUFFER_SIZE_DOWN]; //down buffers
char    upnames[SEGGER_RTT_MAX_NUM_UP_BUFFERS-1][64] = {
	"Up channel 1     ",
	"JScope_t4b1f4i2u4",
};

char downnames[SEGGER_RTT_MAX_NUM_UP_BUFFERS-1][64] = {
	"down channel 1   ",
	"JScope_t4b1f4i2u4",
};

char * terimal_up_name = "LoggerR";
char * terimal_down_name = "LoggerW";

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
void RTTGetKeyTest()
{
	int ch = SEGGER_RTT_WaitKey();
	SEGGER_RTT_Write(0,&ch,1);
}


//COMMAND 0 ~ 33
#define COMMAND_COUNT	34
const char * COMMAND[COMMAND_COUNT] = {
 RTT_CTRL_RESET,               // "\x1B[0m"      	0  
 RTT_CTRL_CLEAR,               // "\x1B[2J"       1  

 RTT_CTRL_TEXT_BLACK,          // "\x1B[2;30m"		2
 RTT_CTRL_TEXT_RED,            // "\x1B[2;31m"		3
 RTT_CTRL_TEXT_GREEN,          // "\x1B[2;32m"		4
 RTT_CTRL_TEXT_YELLOW,         // "\x1B[2;33m"		5
 RTT_CTRL_TEXT_BLUE,           // "\x1B[2;34m"		6
 RTT_CTRL_TEXT_MAGENTA,        // "\x1B[2;35m"		7
 RTT_CTRL_TEXT_CYAN,           // "\x1B[2;36m"		8
 RTT_CTRL_TEXT_WHITE,          // "\x1B[2;37m"		9

 RTT_CTRL_TEXT_BRIGHT_BLACK,   // "\x1B[1;30m"		A
 RTT_CTRL_TEXT_BRIGHT_RED,     // "\x1B[1;31m"		B
 RTT_CTRL_TEXT_BRIGHT_GREEN,   // "\x1B[1;32m"		C
 RTT_CTRL_TEXT_BRIGHT_YELLOW,  // "\x1B[1;33m"		D
 RTT_CTRL_TEXT_BRIGHT_BLUE,    // "\x1B[1;34m"		E
 RTT_CTRL_TEXT_BRIGHT_MAGENTA, // "\x1B[1;35m"		F
 RTT_CTRL_TEXT_BRIGHT_CYAN,    // "\x1B[1;36m"		G
 RTT_CTRL_TEXT_BRIGHT_WHITE,   // "\x1B[1;37m"		H

 RTT_CTRL_BG_BLACK,            // "\x1B[24;40m"		I
 RTT_CTRL_BG_RED,              // "\x1B[24;41m"		J
 RTT_CTRL_BG_GREEN,            // "\x1B[24;42m"		K
 RTT_CTRL_BG_YELLOW,           // "\x1B[24;43m"		L
 RTT_CTRL_BG_BLUE,             // "\x1B[24;44m"		M
 RTT_CTRL_BG_MAGENTA,          // "\x1B[24;45m"		N
 RTT_CTRL_BG_CYAN,             // "\x1B[24;46m"		O
 RTT_CTRL_BG_WHITE,            // "\x1B[24;47m"		P

 RTT_CTRL_BG_BRIGHT_BLACK,     // "\x1B[4;40m"		Q
 RTT_CTRL_BG_BRIGHT_RED,       // "\x1B[4;41m"		R
 RTT_CTRL_BG_BRIGHT_GREEN,     // "\x1B[4;42m"		S
 RTT_CTRL_BG_BRIGHT_YELLOW,    // "\x1B[4;43m"		T
 RTT_CTRL_BG_BRIGHT_BLUE,      // "\x1B[4;44m"		U
 RTT_CTRL_BG_BRIGHT_MAGENTA,   // "\x1B[4;45m"		V
 RTT_CTRL_BG_BRIGHT_CYAN,      // "\x1B[4;46m"		W
 RTT_CTRL_BG_BRIGHT_WHITE,     // "\x1B[4;47m"		X
};


void RTTCommandAuto(int ch){
	static int index = 0;
	index++;
	if(index >= COMMAND_COUNT){
		index = 0;
	}
	SEGGER_RTT_TerminalOut(ch,COMMAND[index]);
				
}

void RTTStatusPrint(bool terimal_only)
{
	SEGGER_RTT_printf(0,"\r\nConrtrol Name   : %s",_SEGGER_RTT.acID);
	SEGGER_RTT_printf(0,"\r\nUpbuffer Count  : %d",_SEGGER_RTT.MaxNumUpBuffers);
	SEGGER_RTT_printf(0,"\r\nDownBuffer Count: %d",_SEGGER_RTT.MaxNumDownBuffers);
	//upbuffer
	SEGGER_RTT_printf(0,"\r\n------------up------------");
	int count = terimal_only?1:_SEGGER_RTT.MaxNumUpBuffers;
	for(int i = 0;i < count;i++){
		SEGGER_RTT_printf(0,"\r\nsName: %s\tFlags:%d\tpBuffer:0x%08x\tSizeOfBuffer:%d\tRdOff:%d\tWrOff:%d",
		_SEGGER_RTT.aUp[i].sName,_SEGGER_RTT.aUp[i].Flags,_SEGGER_RTT.aUp[i].pBuffer,_SEGGER_RTT.aUp[i].SizeOfBuffer,_SEGGER_RTT.aUp[i].RdOff,_SEGGER_RTT.aUp[i].WrOff);
	}
	//down buffer
	SEGGER_RTT_printf(0,"\r\n------------down------------");
	count = terimal_only?1:_SEGGER_RTT.MaxNumDownBuffers;
	for(int i = 0;i < count;i++){
		SEGGER_RTT_printf(0,"\r\nsName: %s\tFlags:%d\tpBuffer:0x%08x\tSizeOfBuffer:%d\tRdOff:%d\tWrOff:%d",
		_SEGGER_RTT.aDown[i].sName,_SEGGER_RTT.aDown[i].Flags,_SEGGER_RTT.aDown[i].pBuffer,_SEGGER_RTT.aDown[i].SizeOfBuffer,_SEGGER_RTT.aDown[i].RdOff,_SEGGER_RTT.aDown[i].WrOff);
	}
}







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
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  MX_CRC_Init();
  MX_RNG_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
	  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	
    can_filter_init();
    delay_init();
    remote_control_init();

    DWT_Init(168);
    while (BMI088_init(&hspi1, 1) != BMI088_NO_ERROR)
    {
       ;
    }
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	SEGGER_RTT_Init();
	SEGGER_RTT_CB * cb = &_SEGGER_RTT;
	for(int i = 1; i < cb->MaxNumUpBuffers;i++){
		SEGGER_RTT_AllocUpBuffer(upnames[i-1],upbuffers[i-1],BUFFER_SIZE_UP,SEGGER_RTT_MODE_DEFAULT);
	}
	for(int i = 1; i < cb->MaxNumDownBuffers;i++){
		SEGGER_RTT_AllocDownBuffer(downnames[i-1],downbuffers[i-1],BUFFER_SIZE_DOWN,SEGGER_RTT_MODE_DEFAULT);
	}
	//Set terimal name (up & down)
	//SEGGER_RTT_SetNameUpBuffer(0,terimal_up_name);
	//SEGGER_RTT_SetNameDownBuffer(0,terimal_down_name);
	
	SEGGER_RTT_TerminalOut(0,"/*");
	SEGGER_RTT_TerminalOut(0," * PowerDebugger RTT Demo, this demo is used to demonstrate the RTT function of the verification device,");
	SEGGER_RTT_TerminalOut(0," * including the parallel channel data read and write Settings, and waveform diagram, etc.");
	SEGGER_RTT_TerminalOut(0," * Copyright: ICWorkshop  ");
	SEGGER_RTT_TerminalOut(0," * -(RTT Ch0)");
	SEGGER_RTT_TerminalOut(0," * -end");
	SEGGER_RTT_TerminalOut(0," */"); 
	RTTStatusPrint(true);
			uint32_t Tick = HAL_GetTick();
    while (1)
    {
    /* USER CODE END WHILE */
 	//get key test
		//RTTGetKeyTest();	
		
		//command test		
    if(HAL_GetTick() - Tick >= 1000){
			//print RTT status
			RTTStatusPrint(true);
			
			Tick += 1000;
			
			RTTCommandAuto(0);
			
			SEGGER_RTT_TerminalOut(0,"Terminal 0 Out\r\n");
			SEGGER_RTT_TerminalOut(1,"Terminal 1 Out\r\n");
			SEGGER_RTT_TerminalOut(2,"Terminal 2 Out\r\n");
			SEGGER_RTT_TerminalOut(3,"Terminal 3 Out\r\n");
			SEGGER_RTT_TerminalOut(4,"Terminal 4 Out\r\n");
			SEGGER_RTT_TerminalOut(5,"Terminal 5 Out\r\n");
			SEGGER_RTT_TerminalOut(6,"Terminal 6 Out\r\n");
			SEGGER_RTT_TerminalOut(7,"Terminal 7 Out\r\n");
			SEGGER_RTT_TerminalOut(8,"Terminal 8 Out\r\n");
			SEGGER_RTT_TerminalOut(9,"Terminal 9 Out\r\n");
			SEGGER_RTT_TerminalOut(10,"Terminal 10 Out\r\n");
			SEGGER_RTT_TerminalOut(11,"Terminal 11 Out\r\n");
			SEGGER_RTT_TerminalOut(12,"Terminal 12 Out\r\n");
			SEGGER_RTT_TerminalOut(13,"Terminal 13 Out\r\n");
			SEGGER_RTT_TerminalOut(14,"Terminal 14 Out\r\n");
			SEGGER_RTT_TerminalOut(15,"Terminal 15 Out\r\n");
		
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV30;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
