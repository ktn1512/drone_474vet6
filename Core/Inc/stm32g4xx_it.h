/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @details Khai báo các điểm vào của bảng vector ngắt Cortex-M4 và ngoại vi.
  *            Hành vi từng handler được cài đặt trong stm32g4xx_it.c.
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#ifndef __STM32G4xx_IT_H
#define __STM32G4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief Xử lý ngắt không che được; implementation hiện tại dừng trong vòng lặp vô hạn.
 */
void NMI_Handler(void);
/**
 * @brief Xử lý HardFault; implementation hiện tại dừng trong vòng lặp vô hạn.
 */
void HardFault_Handler(void);
/**
 * @brief Xử lý lỗi bảo vệ bộ nhớ; implementation hiện tại dừng trong vòng lặp vô hạn.
 */
void MemManage_Handler(void);
/**
 * @brief Xử lý lỗi truy cập bus; implementation hiện tại dừng trong vòng lặp vô hạn.
 */
void BusFault_Handler(void);
/**
 * @brief Xử lý lỗi lệnh hoặc trạng thái CPU; hiện dừng trong vòng lặp vô hạn.
 */
void UsageFault_Handler(void);
/**
 * @brief Điểm xử lý ngoại lệ SVC; thân hàm hiện không thực hiện thao tác nào.
 */
void SVC_Handler(void);
/**
 * @brief Điểm xử lý ngoại lệ Debug Monitor; thân hàm hiện không thực hiện thao tác nào.
 */
void DebugMon_Handler(void);
/**
 * @brief Điểm xử lý ngoại lệ PendSV; thân hàm hiện không thực hiện thao tác nào.
 */
void PendSV_Handler(void);
/**
 * @brief Xử lý SysTick bằng HAL_IncTick để tăng bộ đếm thời gian HAL.
 */
void SysTick_Handler(void);
/**
 * @brief Chuyển ngắt DMA1 Channel1 cho HAL xử lý DMA RX của USART2.
 */
void DMA1_Channel1_IRQHandler(void);
/**
 * @brief Chuyển ngắt DMA1 Channel2 cho HAL xử lý DMA TX của USART2.
 */
void DMA1_Channel2_IRQHandler(void);
/**
 * @brief Chuyển ngắt USB ưu tiên thấp cho HAL_PCD_IRQHandler của USB FS.
 */
void USB_LP_IRQHandler(void);
/**
 * @brief Chuyển ngắt USART1 cho HAL_UART_IRQHandler với handle huart1.
 */
void USART1_IRQHandler(void);
/**
 * @brief Chuyển ngắt USART2 cho HAL_UART_IRQHandler với handle huart2.
 */
void USART2_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32G4xx_IT_H */
