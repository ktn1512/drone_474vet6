# E220-900T22D trong project STM32G474

Driver: `Core/Inc/e220.h`, `Core/Src/e220.c`.

## Cách nối hiện tại

| Chân E220 | Kết nối |
| --- | --- |
| RXD | PD5 / USART2_TX |
| TXD | PD6 / USART2_RX |
| M0, M1 | GND |
| AUX | Không nối STM32 |
| GND | GND chung với STM32 |

USART2 dùng **9600 baud, 8N1**, đã được khai báo trong `main.c` và file `.ioc`.
M0/M1 ở mức thấp chọn normal mode. Module phải được cấu hình sẵn để truyền
trong suốt (transparent), tắt byte RSSI đính kèm; driver chuyển nguyên byte
UART vào bộ đệm nhận, không tự thêm địa chỉ hay tách byte RSSI.

Với M0/M1 nối GND, STM32 không thể đổi địa chỉ, kênh, tốc độ vô tuyến hoặc
khóa qua UART. Muốn cấu hình bằng công cụ Ebyte/USB-UART, cần ngắt hai chân
này khỏi GND rồi đưa cả hai lên mức cao; UART cấu hình là 9600 8N1. Lưu cấu
hình vào module và đưa M0/M1 về GND để chạy lại.

## Khớp cấu hình với ESP32

Ví dụ `examples/esp32_e220/esp32_e220.ino` đang đặt:

| Tham số | Giá trị |
| --- | --- |
| Address | `0x1234` |
| Channel | `23` thập phân / `0x17` |
| Frequency | `873.125 MHz` |
| Air data rate | `2.4 kbps` |
| UART | `9600`, `8N1` |
| Transmission | Transparent |
| RSSI byte / LBT | Tắt |
| Key | `0x0000` |

Cấu hình sẵn E220 phía STM32 cho khớp các giá trị này. Module mặc định có
address `0x0000`, nên không khớp address `0x1234` của ví dụ ESP32. Nếu muốn
giữ address mặc định trên cả hai module, đổi `RADIO_ADDRESS` trong ví dụ
ESP32 thành `0x0000` và kiểm tra các tham số còn lại. Ví dụ ESP32 dùng cấu
hình tạm thời `C2` và áp dụng lại mỗi lần khởi động.

`Core/esp32_drone_web.ino` giữ web điều khiển ga, ARM và PID, dùng UART
transparent với cấu hình đã lưu trong module. Sketch web **không gửi lệnh
C2**, nên không tự đổi address thành `0x1234` như ví dụ ở trên. Cấu hình
hai module cho khớp nhau trước khi dùng; UART ESP32 là 9600 8N1.

## ESP32 web gửi/nhận E220

| Chân E220 | ESP32 DevKit WROOM-32 | ESP32-S3 |
| --- | --- | --- |
| RXD | GPIO17 / TX2 | GPIO17 / TX2 |
| TXD | GPIO16 / RX2 | GPIO18 / RX2 |
| M0 | GPIO26 | GPIO4 |
| M1 | GPIO27 | GPIO5 |
| AUX | GPIO25 | GPIO6 |
| VCC | Nguồn 5 V ổn định | Nguồn 5 V ổn định |
| GND | GND chung với ESP32 | GND chung với ESP32-S3 |

Sketch đặt M0/M1 LOW để gửi và nhận ở chế độ thường. Nếu AUX không nối,
đặt `E220_AUX = -1` trong nhánh loại bo tương ứng. Khi có AUX, sketch chờ
HIGH ổn định ít nhất 3 ms trước khi gửi; mọi lần gửi vẫn có khoảng nghỉ
`MIN_RADIO_GAP_MS`. AUX và bộ đếm TX chỉ phản ánh trạng thái module/UART,
không xác nhận module bên kia đã nhận gói.

1. Sao chép `Core/esp32_drone_web.ino` và `Core/index_html.h` vào cùng thư
   mục Arduino tên `esp32_drone_web`, rồi mở file `.ino` trong thư mục đó.
2. Chỉnh `WIFI_SSID`, `WIFI_PASS` và GPIO theo bo; chọn **ESP32 Dev Module**
   hoặc **ESP32S3 Dev Module** trong Arduino IDE rồi nạp.
3. Mở Serial Monitor ở 115200 baud. Khi Wi-Fi kết nối, mở địa chỉ
   `Dashboard: http://...` được in ra.
4. Mục **Gửi nhận E220** hiển thị dòng RX/TX gần nhất, bộ đếm và lỗi.
   Khi DISARM, bấm **Gửi gói test E220** để gửi `ESP32 -> STM32\r\n`.
   Gói test được xếp hàng tới lượt gửi gần nhất, có timeout 2 giây nếu
   module vẫn bận; theo dõi TX và bộ đếm lỗi để biết kết quả gửi UART.

Web gửi lệnh `T` (ga/ARM), `P` (ghi PID), `G` (đọc PID) theo giao thức
`DroneLink`: ASCII kèm XOR checksum `*HH` và ký tự `\n`. ESP32 nhận và xử
lý `A`/`N` (ACK/từ chối), `R` (PID), `S` (telemetry), đồng thời hiển thị
cả văn bản test không có checksum. Dòng quá dài, chứa NUL, hoặc bị ngắt
giữa chừng được bỏ tới ký tự xuống dòng tiếp theo.

Vòng `main.c` hiện đang test văn bản: gửi `STM32 -> ESP32\r\n` mỗi giây
và đưa dữ liệu nhận ra USART1 (38400 baud). Vì `DroneLink_Poll` và vòng
điều khiển đang được comment, chế độ này chưa xử lý lệnh ARM/ga/PID hay
gửi status drone. Web sẽ hiển thị RX test nhưng vẫn khóa ARM khi chưa có
status hợp lệ. Muốn điều khiển drone cần chạy lại vòng `DroneLink` phía
STM32; RX văn bản thô không được dùng để bỏ qua điều kiện này.

Sketch web đã được biên dịch thành object với Arduino-ESP32 3.3.12 và
Xtensa GCC 14.2.0 cho ESP32, ESP32-S3 UART console và ESP32-S3 USB CDC.
JavaScript của dashboard đã được kiểm tra với trạng thái API giả lập,
gồm RX test, telemetry và khóa điều khiển khi mất kết nối. Chưa kiểm tra
liên kết firmware hoàn chỉnh, nạp bo hoặc truyền nhận trên phần cứng.

## Dùng driver

```c
E220_t lora;

if (E220_Init(&lora, &huart2) != HAL_OK) {
    Error_Handler();
}

HAL_StatusTypeDef tx_status = E220_SendString(&lora, "hello\r\n");
uint8_t data[128];
uint16_t received = E220_Read(&lora, data, sizeof(data));
```

`main.c` đã chuyển callback USART2 sang `E220_RxCallback` và
`E220_ErrorCallback`. `DroneLink` dùng `E220_Send_IT` với bộ đệm tồn tại đến
khi UART gửi xong, để vòng điều khiển không phải chờ truyền UART.

- Mỗi lần gửi tối đa 200 byte; dài hơn trả `HAL_ERROR`, không tự cắt dữ liệu.
  Đây là giới hạn của driver, không phải giới hạn tổng dữ liệu của module.
- Timeout gửi tính theo độ dài và baud UART; 200 byte ở 9600 baud dùng
  229 ms, tránh timeout 100 ms của thư viện cũ.
- Bộ đệm RX có 255 byte sử dụng được. Khi đầy, bỏ byte mới nhất và tăng
  `rx_overflow_count`; `uart_error_count` đếm lỗi UART. `rx_status` lưu kết
  quả gần nhất khi khởi động/rearm nhận bằng ngắt.
- AUX không nối nên `HAL_OK` chỉ xác nhận việc gửi qua UART, không xác nhận
  module rảnh hoặc bên kia nhận được. Ứng dụng cần giãn thời gian gửi theo
  tốc độ vô tuyến; API gửi bằng ngắt cũng không thay thế điều khiển lưu lượng.
  Vòng test hiện tại gửi chuỗi ngắn mỗi giây. Với dữ liệu dài 200 byte và air
  rate 2.4 kbps, nên dành thêm thời gian cho overhead vô tuyến giữa các lần
  gửi; không dùng chu kỳ test một giây cho mọi loại gói.

Thông số được đối chiếu với [tài liệu Ebyte E220-900T22D](https://www.ebyte.com/Uploadfiles/Files/2021-7-6/202176188184448.pdf),
phần 5–7. Chưa kiểm tra truyền/nhận trên phần cứng thực tế.

## Kiểm thử driver trên máy tính

```text
gcc -std=c11 -Wall -Wextra -Werror -Itests/e220_hal -ICore/Inc Core/Src/e220.c tests/test_e220.c -o test_e220.exe
test_e220.exe
```

Các test giả lập HAL kiểm tra timeout, lỗi/busy UART, giới hạn dữ liệu, gửi
bằng ngắt, thứ tự nhận khi bộ đệm quay vòng/đầy và khôi phục nhận sau lỗi.
