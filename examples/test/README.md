# Test E220 đơn giản: STM32 gửi PING, ESP32 trả PONG

1. Nạp firmware STM32 hiện tại. `Core/Src/main.c` đã bật
   `#define LORA_SIMPLE_TEST 1`; chế độ này chạy trước phần IMU/PID/PWM.
   Driver `e22.h/c` giữ nguyên, dùng UART transparent để giao tiếp với E220.
2. Mở **`examples/test/test.ino`** trong Arduino IDE. Dùng sketch trong thư
   mục riêng này, không đặt chung với sketch web có `setup()` và `loop()`.
3. Chọn ESP32 Dev Module hoặc ESP32S3 Dev Module rồi nạp sketch.
4. Mở Serial Monitor ESP32 ở **115200 baud**. Xem log STM32 qua USB-UART nối
   **PA9 / USART1_TX → RX USB-UART**, GND chung, ở **38400 baud, 8N1**.
   Log STM32 dùng USART1, không phải cổng USB CDC của board.

| E220 | STM32 | ESP32 DevKit | ESP32-S3 |
| --- | --- | --- | --- |
| RXD | PD5 / USART2_TX | GPIO17 | GPIO17 |
| TXD | PD6 / USART2_RX | GPIO16 | GPIO18 |
| M0, M1 | GND | GND | GND |
| AUX | Không nối | Không nối | Không nối |
| GND | GND chung | GND chung | GND chung |

Cấp nguồn cho module và gắn anten. Hai module cần được cấu hình sẵn:
UART 9600 8N1, transparent, cùng address/channel/air rate/key, tắt byte RSSI
đính kèm. Code không đổi cấu hình radio vì M0/M1 đang nối GND.

STM32 gửi một chuỗi ngắn mỗi 2 giây. Kết quả mong đợi:

```text
STM32 (38400):
[TX UART OK] PING STM32
[RX] PONG ESP32

ESP32 (115200):
[RX] PING STM32
[TX UART] PONG ESP32
```

Chỉ có `[TX UART OK]` nghĩa là STM32 đã gửi qua UART. Nhìn thấy
`[RX] PONG ESP32` trên STM32 mới xác nhận trao đổi thành công cả hai chiều.
Nếu ESP32 không thấy PING, kiểm tra nguồn, GND, RX/TX và cấu hình hai radio.
Nếu ESP32 thấy PING nhưng STM32 không thấy PONG, kiểm tra đường TXD E220
về PD6 và ngắt USART2.

Chương trình drone cũ vẫn được giữ trong nhánh `#else`. Bản firmware đã build
cho bài test dùng `LORA_SIMPLE_TEST = 1`.
