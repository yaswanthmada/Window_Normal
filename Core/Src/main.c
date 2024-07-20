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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void delay_ms(int );
int CURRENT(void);
int DCURRENT(void);
void dsw_lsw_rl_gpio_init(void);
void anti_pinch(void);
void motor_up(void);
void motor_down(void);
void motor_stopup(void);
void motor_stopdown(void);
void adc_init(void);
uint16_t ADC_READ();
void adc_clock(void);
void UART_INIT();
void SEND_UART(char );
void string_uart(char *);
void int_uart(int );
void float_uart(float,int);


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define THRESHOLD_CURRENT 4.170
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float adc_vol,adc_cur,k;
int adc_val,cnt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  dsw_lsw_rl_gpio_init();
  adc_init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  up_down();
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void dsw_lsw_rl_gpio_init(void)
{
  	RCC->AHBENR|=(1<<17);//CLK FOR GPIOA


  	GPIOA->MODER&=(~(1<<2))&(~(1<<3));//PA1 INPUT(DOOR DOWN SWITCH)
  	GPIOA->MODER&=(~(1<<4))&(~(1<<5));//PA2 INPUT(DOOR UP SWITCH)


  	GPIOA->MODER&=(~(1<<6))&(~(1<<7));//PA3 INPUT(LIMIT DOWN SWITCH)
  	GPIOA->MODER&=(~(1<<8))&(~(1<<9));//PA4 INPUT(LIMIT UP SWITCH)

  	GPIOA->MODER|=(1<<10);//PA5 OUTPUT(RELAY R1)
  	GPIOA->MODER|=(1<<12);//PA6 OUTPUT(RELAY R2)

  	GPIOA->PUPDR|=1<<3;//PA1 PULL-DOWN
  	GPIOA->PUPDR|=1<<5;//PA2 PULL-DOWN
	GPIOA->PUPDR|=1<<7;//PA3 PULL-DOWN
	GPIOA->PUPDR|=1<<9;//PA4 PULL-DOWN

	GPIOA->PUPDR|=1<<10;//PA5 PULL-UP(Relay 1)
	GPIOA->PUPDR|=1<<12;//PA6 PULL-UP(Relay 2)

	GPIOA->ODR|=(1<<5)|(1<<6);//Initially realay should on

}
/************************* SOFTWARE DELAY **************************************/
void delay_ms(int ms)
{
 ms*=12000;
 while(ms--);
}
/************************* ADC INIT ********************************************/
void adc_init()
{

	RCC->APB2ENR|=(1<<9);
	RCC->AHBENR|=(1<<17);
	GPIOA->MODER|=0X3;
	ADC1->CR |= 1<<31; /* (4) */
	while ((ADC1->CR>>31 & 1) != 0);
	ADC1->CFGR2&=(~(1<<31))&(~(1<<30));//ASYN CLK
	ADC1->CFGR1&=(~(1<<0));//SCAN CHNL FROM 0 TO 18
	ADC1->CFGR1|=(1<<13);
	ADC1->CR|=(1<<0);
	ADC1->CHSELR|=(1<<0);
	while(!(ADC1->ISR>>0 &1));
}
/*****************************ADC READ ****************************************/
uint16_t ADC_READ()
{
	    ADC1->CR |= (1<<2); // START COVR
	    while (!(ADC1->ISR>>2&1)); // END CONVER
	    return ADC1->DR; // READ COVERT VAL
}
/*-------------------------------------MOTOR_UP--------------------------------*/
void motor_down(void)
{
   GPIOA->ODR|=(1<<6);//RELAY R2 OFF
   delay_ms(1);
   GPIOA->BSRR|=(1<<21);//RELAY R1 ON
}


/*------------------------------------MOTOR_DOWN-----------------------------*/
void motor_up(void)
{
   GPIOA->ODR|=(1<<5);//RELAY R1 0FF
   delay_ms(1);
   GPIOA->BSRR|=(1<<22);//RELAY R2 ON
}


/*------------------------------------MOTOR_STOP-----------------------------*/
void motor_stop(void)
{
	GPIOA->ODR|=(1<<5)|(1<<6);//Stop Motor,RELAY PINS HIGH
	cnt=0;
}
/************************************ANTI-PINCH*********************************/
void anti_pinch(void)
{
motor_stop();
delay_ms(1);
motor_down();
delay_ms(50);
motor_stop();
while(GPIOA->IDR>>2 &1);
}
/****************************** UP_DOWN_CHECK************************************/
void up_down(void)
{
/*------------------------------------------------------------------------------*
*                    CHECK FOR DOWN SWITCH                                     *
*------------------------------------------------------------------------------*/
label1:if(((GPIOA->IDR>>1)&1)==1)
	 {
		 delay_ms(5);
		 if(((GPIOA->IDR>>1)&1)==0)//DEBOUNCE
		 {
			 goto label1;
		 }
		 if((GPIOA->IDR>>3 & 1)==1)//limit switch already press
		 {
			 k=2;
			 goto label1;
		 }
		 else
		 {
			  motor_down();
		 }
		 //wait until window switch release or limit switch press
		 while((GPIOA->IDR>>1 &1)&&(!(GPIOA->IDR>>3 &1)));
	     motor_stop();
	 }
     else
     {
      motor_stop();
     }
/*------------------------------------------------------------------------------*
 *                    CHECK FOR UP SWITCH                                     *
 *------------------------------------------------------------------------------*/
 label2:if(((GPIOA->IDR>>2)&1)==1)
 {
      delay_ms(5);
	if(((GPIOA->IDR>>2)&1)==0)//DEBOUNCE
	 {
		 goto label2;
	 }
	 if((GPIOA->IDR>>4 & 1)==1)//limit switch already press
	 {
		 goto label1;
	 }
	 else
	 {
		  motor_up();
	 }
	 //wait until window switch release or limit switch press or adc current is more
	 while((GPIOA->IDR>>2 & 1)&&(!(GPIOA->IDR>>4 &1)))
	 {
		 if(!CURRENT())
		 {
			 anti_pinch();
		 }
	 }
	 motor_stop();
	 while(GPIOA->IDR>>2 &1);

}
else
{
  motor_stop();
}
}
/****************************CURRENT READ ***********************************/
int CURRENT(void)
{
   adc_val=0;
   if(cnt==0)
   {
	   delay_ms(10);
	   cnt++;
   }
   for(int i=0;i<10;i++)
   {
	   adc_val=adc_val+ADC_READ();
   }
   adc_val=adc_val/10;
   adc_vol=(adc_val*(3.6/4095.0))-0.52;
   adc_cur=(adc_vol-2.5)/0.066;
   if(adc_cur<0)
   {
	   adc_cur=-adc_cur;
   }
     if(adc_cur>THRESHOLD_CURRENT)
     {
  	   return 0;
     }
  return 1;
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
