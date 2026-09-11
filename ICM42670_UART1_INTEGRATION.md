# Đọc ICM42670 và gửi UART1

## Trạng thái mã hiện tại đã kiểm tra

- ICM42670 đã có driver tại `Core/Inc/icm42670.h` và `Core/Src/icm42670.c`.
- Driver dùng `SPI1`: PA5 = SCK, PA6 = MISO, PA7 = MOSI; chip-select là PA4 (`ICM_CS`). SPI đang ở mode 0, 8-bit, khoảng 10.625 MHz.
- `ICM42670_ReadAll(&imu)` đọc 12 byte bắt đầu tại thanh ghi `0x0B`, tương ứng ACCEL_X/Y/Z rồi GYRO_X/Y/Z, và trả về các giá trị `int16_t` raw.
- USART1 là PA9 = TX, PA10 = RX, 38 400 baud, 8-N-1. Nó đang nhận GPS bằng ngắt. Có thể truyền dữ liệu IMU trên TX/PA9 đồng thời; không thay đổi callback nhận GPS.

Không có file C/H nào bị sửa. Các thay đổi dưới đây chỉ là mã cần chèn nếu muốn tích hợp chức năng này vào firmware.

## Mã cần thêm vào `Core/Src/main.c`

### 1. Thêm vùng đệm UART1

Chèn vào `/* USER CODE BEGIN PV */`, sau các biến `imu`, `tx_frame`, `seq` có sẵn:

```c
static char imu_uart1_frame[96];
static uint32_t imu_uart1_last_tx_ms = 0U;
```

### 2. Kiểm tra cảm biến sau khi khởi tạo

Ngay sau dòng `ICM42670_Init();` trong `/* USER CODE BEGIN 2 */`, chèn:

```c
if (ICM42670_ReadReg(ICM_WHO_AM_I) != 0x67U) {
    Error_Handler();
}
```

`0x67` là giá trị `WHO_AM_I` của ICM-42670-P. Nếu chương trình dừng tại đây, cần kiểm tra nguồn 3.3 V, dây SPI1 và chân CS PA4.

### 3. Đọc và gửi một khung IMU trên UART1

Trong `while (1)`, giữ nguyên dòng đã có:

```c
ICM42670_ReadAll(&imu);
```

Sau dòng đó, chèn:

```c
if ((HAL_GetTick() - imu_uart1_last_tx_ms) >= 100U) {
    int n = snprintf(imu_uart1_frame, sizeof(imu_uart1_frame),
                     "ICM,%lu,%d,%d,%d,%d,%d,%d\r\n",
                     HAL_GetTick(),
                     imu.ax, imu.ay, imu.az,
                     imu.gx, imu.gy, imu.gz);

    if ((n > 0) && (n < (int)sizeof(imu_uart1_frame))) {
        if (HAL_UART_Transmit(&huart1, (uint8_t *)imu_uart1_frame,
                              (uint16_t)n, 50U) == HAL_OK) {
            imu_uart1_last_tx_ms = HAL_GetTick();
        }
    }
}
```

Ví dụ dữ liệu nhận được từ PA9 ở 38 400 baud:

```text
ICM,1234,-110,203,16400,15,-24,8
```

Thứ tự trường là `ICM,tick_ms,ax,ay,az,gx,gy,gz`. Các giá trị hiện là raw signed-16-bit, phù hợp để xác nhận giao tiếp và chiều trục mà không phụ thuộc cấu hình full-scale.

## Các điểm cần lưu ý trước khi áp dụng

1. Không đổi baud của USART1 nếu GPS vẫn dùng 38 400 baud. Serial terminal/USB-UART để xem IMU cũng phải đặt 38 400, 8-N-1; nối GND chung và RX của USB-UART vào PA9.
2. Không gọi lại `HAL_UART_Receive_IT()` và không sửa `HAL_UART_RxCpltCallback()` cho tính năng này. Callback hiện tại đã dành RX của USART1 cho GPS; truyền bằng `HAL_UART_Transmit()` chỉ dùng TX.
3. Vòng lặp hiện có thể chờ ACK LoRa đến 500 ms. Do đó chu kỳ thực tế của khung IMU có thể chậm hơn 100 ms; đoạn mã trên chỉ giới hạn tốc độ gửi tối đa 10 Hz. Nếu cần 10 Hz bảo đảm, phần chờ ACK LoRa phải được chuyển sang xử lý bất đồng bộ.
4. Dòng kiểm tra `WHO_AM_I` giả định đúng chip là ICM-42670-P. Nếu module dùng biến thể khác, hãy đọc và in giá trị này trước, rồi xác nhận với datasheet của biến thể đó thay vì đổi hằng số một cách đoán mò.

## File cần sửa khi muốn tích hợp

| File | Sửa đổi |
|---|---|
| `Core/Src/main.c` | Thêm hai biến tĩnh, kiểm tra `WHO_AM_I` sau `ICM42670_Init()`, và chèn khối gửi UART1 trong vòng lặp. |

Không cần sửa `Core/Src/icm42670.c`, `Core/Inc/icm42670.h`, `Core/Src/gps.c` hay cấu hình CubeMX cho phiên bản raw-data này.

## Xác nhận phần cứng

- ICM42670-P hỗ trợ SPI tới 24 MHz; cấu hình hiện tại 10.625 MHz nằm trong giới hạn.
- Nếu `WHO_AM_I` sai hoặc dữ liệu luôn `0`/`-1`, kiểm tra CS PA4 ở mức cao khi rỗi, mode SPI 0, và MISO PA6.
