/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "motor.h"
//#include <stdio.h>
#include "RPM_Encoder.h"
#include "cmsis_os.h"
#include "uartstdio.h"
#include "ctype.h"
#include "pca9685.h"
#include "math.h"
//#include "PS2.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SS_Pin GPIO_PIN_4
#define SS_GPIO_Port GPIOA
#define DIRECTION_3_Pin GPIO_PIN_13
#define DIRECTION_3_GPIO_Port GPIOE
#define DIRECTION_2_Pin GPIO_PIN_14
#define DIRECTION_2_GPIO_Port GPIOE
#define DIRECTION_1_Pin GPIO_PIN_15
#define DIRECTION_1_GPIO_Port GPIOE
#define L298_IN1_Pin GPIO_PIN_0
#define L298_IN1_GPIO_Port GPIOD
#define L298_IN2_Pin GPIO_PIN_1
#define L298_IN2_GPIO_Port GPIOD
#define L298_IN3_Pin GPIO_PIN_2
#define L298_IN3_GPIO_Port GPIOD
#define L298_IN4_Pin GPIO_PIN_3
#define L298_IN4_GPIO_Port GPIOD
#define L298_IN5_Pin GPIO_PIN_4
#define L298_IN5_GPIO_Port GPIOD
#define L298_IN6_Pin GPIO_PIN_7
#define L298_IN6_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#define spi_enable 		HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, GPIO_PIN_RESET)
#define spi_disable  	HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, GPIO_PIN_SET)

//#define Forward 0xef
//#define Backward 0xbf
//#define Left 0x7f
//#define Right 0xdf
//#define IDLE 0xff
//#define Forward_Left 0x6f
//#define Forward_Right 0xcf
//#define Backward_Left 0x3f
//#define Backward_Right 0x9f

//#define Select_Hand_1 0xfe
//#define Select_Hand_2 0xfd
//#define Select_Hand_3 0xfb


#define Rotate_180 0xef
#define Rotate_mn180 0xbf

#define Rotate_Left 0x7f
#define Rotate_Right 0xdf

#define Lift 0xef
#define Down 0xbf
#define Close 0x7f
#define Open 0xdf

#define Fast 0xf7
#define Slow 0xfb
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim9;

//extern UART_HandleTypeDef huart5;

extern PID_Param_t pid;
extern void pid_config(void);
extern double rpm_1, rpm_2, rpm_3;
extern double out_1, out_2, out_3;

extern osThreadId_t taskReadButtonHandle;
extern osMessageQueueId_t myButtonsHandle;
extern osThreadId_t CONTROLHandle;

typedef struct {                                // object data type
  char buffer[1];
  uint8_t buffer_index;
} msgQueueObj_t;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
