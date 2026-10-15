/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "e220.h"
#include "icm42670.h"
#include "gps.h"
#include "mmc5983ma.h"
#include "angle_controller.h"
#include "rate_controller.h"
#include "mixer.h"
#include "mahony.h"
#include "drone_link.h"

#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  uint32_t start_ms;
  DroneLink_PitchDebug_t data;
} PitchLogWindow_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1; /**< Handle HAL của I2C1; truyền vào các driver dùng bus này. */
I2C_HandleTypeDef hi2c2; /**< Handle HAL của ngoại vi I2C2. */
I2C_HandleTypeDef hi2c3; /**< Handle HAL của ngoại vi I2C3. */

SPI_HandleTypeDef hspi1; /**< Handle SPI1 dùng trao đổi dữ liệu với ICM42670. */
SPI_HandleTypeDef hspi2; /**< Handle HAL của ngoại vi SPI2. */
SPI_HandleTypeDef hspi3; /**< Handle HAL của ngoại vi SPI3. */

TIM_HandleTypeDef htim5; /**< Handle TIM5 xuất PWM tới bốn ESC qua CH1..CH4. */

UART_HandleTypeDef huart1;        /**< USART1, 38400 baud; hiện dùng xuất log trạng thái. */
UART_HandleTypeDef huart2;        /**< USART2, 9600 baud; giao tiếp với module E220. */
UART_HandleTypeDef huart3;        /**< Handle HAL của USART3, cấu hình 115200 baud. */
DMA_HandleTypeDef hdma_usart2_rx; /**< Handle RX DMA1 Channel1 được CubeMX cấu hình; E220 hiện nhận từng byte bằng ngắt UART. */
DMA_HandleTypeDef hdma_usart2_tx; /**< Handle TX DMA1 Channel2 dùng gửi gói E220 không chặn. */

/* USER CODE BEGIN PV */
E220_t lora; /**< Trạng thái UART, bộ đệm RX/TX và bộ đếm chẩn đoán của radio. */
/* USART2 TX uses CubeMX DMA1_Channel2; E220 RX remains interrupt-driven. */
volatile uint32_t e220_uart_tx_done = 0; /**< Số callback hoàn tất TX của USART2; được tăng trong ngữ cảnh ngắt. */

DroneLink_t drone_link; /**< Trạng thái giao thức throttle/PID/telemetry giữa STM32 và ESP32. */

MMC5983MA_t mmc; /**< Đối tượng từ kế; đoạn khởi tạo hiện đang được comment. */

ICM42670_t icm;      /**< Cấu hình và bias gyro được đo trong quá trình hiệu chuẩn IMU. */
ICM42670_Data_t imu; /**< Mẫu IMU: gia tốc (g), tốc độ góc (độ/s) và nhiệt độ (độ C). */

Mahony_t mahony; /**< Quaternion, phản hồi tích phân và tham số bộ lọc tư thế Mahony. */

AngleController_t angle_ctrl; /**< PID vòng góc roll/pitch, tạo setpoint tốc độ góc. */
RateController_t rate_ctrl;   /**< PID vòng tốc độ roll/pitch/yaw, tạo hiệu chỉnh cho mixer. */

MotorMixer_t motor; /**< Độ rộng xung của bốn động cơ m1..m4, đơn vị microsecond. */

float roll_deg;  /**< Góc roll ước lượng từ Mahony, đơn vị độ. */
float pitch_deg; /**< Góc pitch ước lượng từ Mahony, đơn vị độ. */
float yaw_deg;   /**< Góc yaw tương đối, đơn vị độ; chưa dùng từ kế để hiệu chỉnh hướng. */

float roll_rate_sp;  /**< Tốc độ góc roll yêu cầu từ vòng góc, đơn vị độ/s. */
float pitch_rate_sp; /**< Tốc độ góc pitch yêu cầu từ vòng góc, đơn vị độ/s. */

float roll_pid;  /**< Hiệu chỉnh roll đưa vào mixer, cùng thang đơn vị microsecond với throttle. */
float pitch_pid; /**< Hiệu chỉnh pitch đưa vào mixer, cùng thang đơn vị microsecond với throttle. */
float yaw_pid;   /**< Hiệu chỉnh yaw đưa vào mixer, cùng thang đơn vị microsecond với throttle. */

float throttle = 1000.0f; /**< Lệnh ga trực tiếp lấy từ DroneLink, đơn vị microsecond. */

char uart_buf[128]; /**< Bộ đệm chuỗi báo lỗi khi khởi tạo IMU. */

static char tx_frame[512]; /**< Bộ đệm log tồn tại suốt chương trình để USART1 TX bằng ngắt đọc được. */
static PitchLogWindow_t pitch_log;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	if (huart->Instance == USART1) {
//		GPS_RxCallback(gps_rx_byte);
//		HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1);
//	} else if (huart->Instance == USART2) {
//		E220_RxCallback(&lora);
//	}
// }
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t last_tick; /**< Mốc HAL_GetTick() của lần cập nhật điều khiển trước, đơn vị ms. */

/* Collect every control sample, including peaks between radio status packets. */
static void PitchLog_Record(float dt, float p_term, float i_term, float d_term,
                            uint8_t pid_valid)
{
  DroneLink_PitchDebug_t *data = &pitch_log.data;
  float gyro_y = imu.gy_dps;
  float dt_ms = dt * 1000.0f;

  if (data->samples == 0U)
  {
    data->gyro_y_min = data->gyro_y_max = gyro_y;
    data->d_min = data->d_max = d_term;
    data->dt_min_ms = data->dt_max_ms = dt_ms;
  }
  else
  {
    if (gyro_y < data->gyro_y_min)
      data->gyro_y_min = gyro_y;
    if (gyro_y > data->gyro_y_max)
      data->gyro_y_max = gyro_y;
    if (d_term < data->d_min)
      data->d_min = d_term;
    if (d_term > data->d_max)
      data->d_max = d_term;
    if (dt_ms < data->dt_min_ms)
      data->dt_min_ms = dt_ms;
    if (dt_ms > data->dt_max_ms)
      data->dt_max_ms = dt_ms;
  }

  data->gyro_y = gyro_y;
  data->rate_sp = pitch_rate_sp;
  data->p_term = p_term;
  data->i_term = i_term;
  data->d_term = d_term;
  data->output = pitch_pid;
  data->dt_ms = dt_ms;
  ++data->samples;
  if (!pid_valid)
    ++data->invalid_pid_samples;
}

/*
 * One combined S + diagnostic per T request (normally about 1.5 seconds).
 * Preserve every peak until DMA accepts the packet, including busy/PID slots.
 * At throttle 1000 the sampled PID is diagnostic only, not applied to motors.
 */
static void PitchLog_TrySend(uint32_t now)
{
  pitch_log.data.window_ms = now - pitch_log.start_ms;
  if (DroneLink_SendStatusDebug(&drone_link, roll_deg, pitch_deg, yaw_deg,
                                motor.m1, motor.m2, motor.m3, motor.m4,
                                &pitch_log.data))
  {
    pitch_log = (PitchLogWindow_t){.start_ms = now};
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 * @details Khởi tạo ngoại vi, hiệu chuẩn IMU, lấy tư thế ban đầu rồi chạy vòng điều khiển góc/tốc độ và PWM.
 * @note Vòng lặp vô hạn; chương trình không trả về trong hoạt động bình thường.
 * @note DroneLink được poll trong main; UART/DMA callback chỉ xử lý hoàn tất truyền/nhận.
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_I2C3_Init();
  MX_TIM5_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 1000);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 1000);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 1000);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 1000);

  HAL_Delay(4000);

  // if (GPS_Init(&huart1) != HAL_OK)
  // {
  // 	Error_Handler();
  // }
  if (ICM42670_Init(&icm) != ICM42670_OK)
  {
    sprintf(uart_buf, "ICM init err\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf),
                      HAL_MAX_DELAY);
    Error_Handler();
  }
  /* Keep the drone stationary (it may be tilted) throughout calibration. */
  if (ICM42670_CalibrateGyro(&icm, 500, 2) != ICM42670_OK)
  {
    Error_Handler();
  }

  /* Average gravity to initialize the actual tilt instead of zero angles. */
  const uint16_t tilt_samples = 100U; /**< Số mẫu gia tốc dùng tính hướng trọng lực khi khởi động. */
  float ax_sum = 0.0f;                /**< Tổng gia tốc trục X của các mẫu khởi tạo, đơn vị g. */
  float ay_sum = 0.0f;                /**< Tổng gia tốc trục Y của các mẫu khởi tạo, đơn vị g. */
  float az_sum = 0.0f;                /**< Tổng gia tốc trục Z của các mẫu khởi tạo, đơn vị g. */
  /* i là chỉ số mẫu gia tốc đang lấy, từ 0 đến tilt_samples - 1. */
  for (uint16_t i = 0U; i < tilt_samples; ++i)
  {
    if (ICM42670_ReadAll(&imu) != ICM42670_OK)
    {
      Error_Handler();
    }
    ax_sum += imu.ax_g;
    ay_sum += imu.ay_g;
    az_sum += imu.az_g;
    HAL_Delay(2U);
  }
  if (!Mahony_InitFromAccel(&mahony, ax_sum / tilt_samples,
                            ay_sum / tilt_samples, az_sum / tilt_samples))
  {
    Error_Handler();
  }
  Mahony_GetEuler(&mahony, &roll_deg, &pitch_deg, &yaw_deg);
  AngleController_Init(&angle_ctrl);
  RateController_Init(&rate_ctrl);

  /* MX_USART2_UART_Init already links TX DMA1_Channel2 and enables USART2 IRQ. */

  /* E220-900T22D: M0/M1 tied to GND, AUX disconnected, UART 9600 8N1.
   * Radio address/channel/air rate must match the peer's stored settings.
   */
  if (E220_Init(&lora, &huart2) != HAL_OK)
  {
    Error_Handler();
  }

  DroneLink_Init(&drone_link, &lora, &angle_ctrl, &rate_ctrl);
  motor.m1 = motor.m2 = motor.m3 = motor.m4 = 1000U;
  /* Tranh dt dau tien bang toan bo thoi gian khoi tao/calibrate IMU. */
  last_tick = HAL_GetTick();
  pitch_log = (PitchLogWindow_t){.start_ms = last_tick};

  /* E220 packet TX uses DMA; S is sent on T request (no periodic S). */
  /* Apply the existing motor outputs only after stationary IMU calibration. */
  //	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 1300);
  //	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 1150);
  //	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 1200);
  //	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 1150);
  // mmc.hi2c = &hi2c1;
  // if (MMC5983_Init(&mmc) != HAL_OK)
  // {
  // 	Error_Handler();
  // }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    DroneLink_Poll(&drone_link);

    uint32_t now = HAL_GetTick(); /**< Thời gian hiện tại của vòng lặp, đơn vị ms; phép trừ unsigned xử lý tràn tick. */

    float dt = (now - last_tick) * 0.001f; /**< Thời gian giữa hai lần cập nhật điều khiển, đổi từ ms sang giây. */

    if (dt < 0.002f)
    {
      continue;
    }

    last_tick = now;

    /* ================= IMU ================= */

    if (ICM42670_ReadAll(&imu) != ICM42670_OK)
    {
      // IMU failure must not leave the previous motor PWM active.
      DroneLink_SetMinimumThrottle(&drone_link);
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 1000);
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 1000);
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 1000);
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 1000);
      motor.m1 = motor.m2 = motor.m3 = motor.m4 = 1000;
      (void)DroneLink_SendStatus(&drone_link, roll_deg, pitch_deg, yaw_deg,
                                 motor.m1, motor.m2, motor.m3, motor.m4);
      continue;
    }

    ICM42670_ApplyCalibration(&icm, &imu);

    /* ================= MAHONY ================= */

    Mahony_Update(&mahony, imu.gx_dps, imu.gy_dps, imu.gz_dps, imu.ax_g,
                  imu.ay_g, imu.az_g, dt);

    Mahony_GetEuler(&mahony, &roll_deg, &pitch_deg, &yaw_deg);

    /*
     * Nếu IMU gắn khác chiều
     * thì đổi dấu tại đây
     */

    // roll_deg  = -roll_deg;
    // pitch_deg = -pitch_deg;
    /* ================= ANGLE PID ================= */

    AngleController_Update(&angle_ctrl, 0.0f, // roll target
                           0.0f,              // pitch target

                           roll_deg, pitch_deg, dt, &roll_rate_sp, &pitch_rate_sp);

    /* ================= RATE PID ================= */

    RateController_Update(&rate_ctrl, roll_rate_sp, pitch_rate_sp, 0.0f,
                          imu.gx_dps, imu.gy_dps, imu.gz_dps, dt, &roll_pid, &pitch_pid,
                          &yaw_pid);

    /* Snapshot the terms actually used, before minimum-throttle PID_Reset. */
    uint8_t pitch_pid_valid = rate_ctrl.pitch.initialized;
    float pitch_p_term = rate_ctrl.pitch.kp * rate_ctrl.pitch.prev_error;
    float pitch_i_term = rate_ctrl.pitch.integral;
    float pitch_d_term = -rate_ctrl.pitch.kd * rate_ctrl.pitch.derivative_filtered;
    PitchLog_Record(dt, pitch_p_term, pitch_i_term, pitch_d_term, pitch_pid_valid);

    /* ================= MIXER ================= */

    throttle = (float)DroneLink_Throttle(&drone_link);
    if (throttle > 1000.0f)
    {
      Mixer_QuadX(throttle, roll_pid, pitch_pid, yaw_pid, &motor);
    }
    else
    {
      motor.m1 = motor.m2 = motor.m3 = motor.m4 = 1000U;
      PID_Reset(&angle_ctrl.roll);
      PID_Reset(&angle_ctrl.pitch);
      PID_Reset(&rate_ctrl.roll);
      PID_Reset(&rate_ctrl.pitch);
      PID_Reset(&rate_ctrl.yaw);
    }

    // Xuat PWM ra ESC
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, motor.m1);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, motor.m2);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, motor.m3);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, motor.m4);

    static uint32_t last_log_tick = 0U; /**< Mốc gửi log gần nhất, đơn vị ms; giữ giá trị giữa các vòng lặp. */

    uint32_t log_now = HAL_GetTick(); /**< Thời gian dùng kiểm tra chu kỳ log 500 ms. */

    PitchLog_TrySend(log_now);

    if ((log_now - last_log_tick >= 500U) && (huart1.gState == HAL_UART_STATE_READY))
    {
      /** @brief Số ký tự snprintf cần ghi, không tính NUL; chỉ gửi khi bộ đệm đủ chỗ. */
      int len =
          snprintf(tx_frame, sizeof(tx_frame),
                   "thr=%u angle=%.1f,%.1f,%.1f pid=%.1f,%.1f m=%u,%u,%u,%u\r\n"
                   "LORA T_RX=%lu P_WR=%lu P_RD=%lu TIMEOUT=%lu BAD=%lu"
                   " RX_BYTE=%lu S_REQ=%lu TX_DMA_START=%lu TX_DMA_DONE=%lu"
                   " TX_BUSY=%lu TX_ERR=%lu UART_ERR=%lu\r\n",
                   (unsigned)DroneLink_Throttle(&drone_link),
                   roll_deg, pitch_deg, yaw_deg, roll_pid, pitch_pid,
                   (unsigned)motor.m1, (unsigned)motor.m2,
                   (unsigned)motor.m3, (unsigned)motor.m4,
                   (unsigned long)drone_link.throttle_rx_count,
                   (unsigned long)drone_link.pid_write_count,
                   (unsigned long)drone_link.pid_read_count,
                   (unsigned long)drone_link.throttle_timeout_count,
                   (unsigned long)drone_link.rx_invalid_frames,
                   (unsigned long)drone_link.rx_byte_count,
                   (unsigned long)drone_link.status_request_count,
                   (unsigned long)lora.tx_start_count,
                   (unsigned long)lora.tx_done_count,
                   (unsigned long)lora.tx_busy_count,
                   (unsigned long)lora.tx_error_count,
                   (unsigned long)lora.uart_error_count);

      if ((len > 0) && ((size_t)len < sizeof(tx_frame)))
      {
        if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)tx_frame,
                                 (uint16_t)len) == HAL_OK)
        {
          last_log_tick = log_now;
        }
      }
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 * @details Cấu hình SYSCLK 144 MHz từ HSI/PLL, AHB và APB chia 1; lỗi HAL dẫn tới Error_Handler().
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; /**< Cấu hình HSI, HSI48 và PLL; khởi tạo các trường về 0. */
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; /**< Cấu hình nguồn clock CPU và các bộ chia bus AHB/APB. */

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 * @details Thiết lập I2C1 địa chỉ 7 bit, timing 0x60715075, bật lọc analog và tắt lọc digital.
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x60715075;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 * @details Thiết lập I2C2 địa chỉ 7 bit, timing 0x60715075, bật lọc analog và tắt lọc digital.
 */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x60715075;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 * @details Thiết lập I2C3 địa chỉ 7 bit, timing 0x60715075, bật lọc analog và tắt lọc digital.
 */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x60715075;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 * @details Thiết lập SPI1 master, dữ liệu 8 bit, CPOL thấp/CPHA cạnh đầu, NSS software; dùng cho IMU.
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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 * @details Thiết lập SPI2 master với độ dài dữ liệu 4 bit và bộ chia clock 16 theo cấu hình hiện tại.
 */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 * @details Thiết lập SPI3 master với độ dài dữ liệu 4 bit và bộ chia clock 16 theo cấu hình hiện tại.
 */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 * @details Thiết lập PWM TIM5 với prescaler 143 và period 2499: tick 1 microsecond, chu kỳ 2500 microsecond (400 Hz).
 * @note CH1..CH3 có compare khởi tạo 1000, CH4 là 0; main sau đó đặt cả bốn kênh về 1000.
 */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0}; /**< Chọn nguồn clock nội cho TIM5. */
  TIM_MasterConfigTypeDef sMasterConfig = {0};     /**< Cấu hình đồng bộ và tín hiệu trigger của TIM5. */
  TIM_OC_InitTypeDef sConfigOC = {0};              /**< Cấu hình chế độ PWM, polarity và giá trị compare của từng kênh. */

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 143;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 2499;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 * @details Thiết lập USART1 38400 baud, 8 bit, không parity, 1 stop bit, TX/RX; dùng xuất log.
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 * @details Thiết lập USART2 9600 baud, 8N1, TX/RX; MSP gắn DMA1 Channel2 cho TX E220.
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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 * @details Thiết lập USART3 115200 baud, 8N1, TX/RX, không flow control.
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 * @details Bật clock DMAMUX1/DMA1 và ngắt DMA1 Channel1 (RX), Channel2 (TX) ở priority 0.
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 * @details Bật clock GPIO, cấu hình ICM_CS (PA4) và PB12 là output push-pull, mức ban đầu thấp.
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0}; /**< Cấu hình GPIO dùng chung khi thiết lập các chân output. */
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : ICM_CS_Pin */
  GPIO_InitStruct.Pin = ICM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ICM_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Chuyển callback nhận USART2 tới driver E220 để lưu byte và đăng ký lần nhận tiếp theo.
 * @param[in] huart Handle UART do HAL truyền vào; chỉ USART2 được xử lý ở đây.
 * @note Được HAL gọi trong ngữ cảnh ngắt UART/DMA; không poll giao thức ở đây.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    E220_RxCallback(&lora);
  }
}

/**
 * @brief Đếm lần hoàn tất TX USART2 và giải phóng trạng thái TX DMA của driver E220.
 * @param[in] huart Handle UART do HAL truyền vào; chỉ USART2 được xử lý ở đây.
 * @note Được HAL gọi trong ngữ cảnh ngắt UART/DMA; không poll giao thức ở đây.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    ++e220_uart_tx_done;
    E220_TxCpltCallback(&lora);
  }
}

/**
 * @brief Chuyển lỗi USART2 tới driver E220 để đếm lỗi và phục hồi nhận UART.
 * @param[in] huart Handle UART do HAL truyền vào; chỉ USART2 được xử lý ở đây.
 * @note Được HAL gọi trong ngữ cảnh ngắt UART/DMA; không poll giao thức ở đây.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    E220_ErrorCallback(&lora);
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 * @details Tắt toàn bộ ngắt và dừng trong vòng lặp vô hạn khi gặp lỗi khởi tạo hoặc lỗi nghiêm trọng.
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
 * @details Hook báo assert khi USE_FULL_ASSERT được bật; phần thân hiện chưa xuất thông tin lỗi.
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
