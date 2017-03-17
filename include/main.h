/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define relay_4_ON_Pin GPIO_PIN_13
#define relay_4_ON_GPIO_Port GPIOC
#define relay_4_OFF_Pin GPIO_PIN_14
#define relay_4_OFF_GPIO_Port GPIOC
#define ALARM_Pin GPIO_PIN_2
#define ALARM_GPIO_Port GPIOB
#define relay_7_ON_Pin GPIO_PIN_14
#define relay_7_ON_GPIO_Port GPIOB
#define relay_7_OFF_Pin GPIO_PIN_15
#define relay_7_OFF_GPIO_Port GPIOB
#define relay_1_OFF_Pin GPIO_PIN_8
#define relay_1_OFF_GPIO_Port GPIOD
#define relay_1_ON_Pin GPIO_PIN_8
#define relay_1_ON_GPIO_Port GPIOA
#define relay_3_ON_Pin GPIO_PIN_13
#define relay_3_ON_GPIO_Port GPIOA
#define relay_8_ON_Pin GPIO_PIN_6
#define relay_8_ON_GPIO_Port GPIOF
#define relay_8_OFF_Pin GPIO_PIN_7
#define relay_8_OFF_GPIO_Port GPIOF
#define relay_2_ON_Pin GPIO_PIN_14
#define relay_2_ON_GPIO_Port GPIOA
#define relay_2_OFF_Pin GPIO_PIN_15
#define relay_2_OFF_GPIO_Port GPIOA
#define relay_3_OFF_Pin GPIO_PIN_5
#define relay_3_OFF_GPIO_Port GPIOB
#define relay_5_ON_Pin GPIO_PIN_6
#define relay_5_ON_GPIO_Port GPIOB
#define relay_5_OFF_Pin GPIO_PIN_7
#define relay_5_OFF_GPIO_Port GPIOB
#define relay_6_ON_Pin GPIO_PIN_8
#define relay_6_ON_GPIO_Port GPIOB
#define relay_6_OFF_Pin GPIO_PIN_9
#define relay_6_OFF_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
