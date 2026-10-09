# ESP32 DevKit / ESP32-S3 gửi nhận với E220-900T22D

Dùng cùng file `esp32_e220.ino` cho cả hai bo. Mỗi bo cần một module E220-900T22D.
Sketch dùng Arduino và UART phần cứng, không cần cài thư viện LoRa riêng.
Khi chọn đúng loại bo trong Arduino IDE, code tự chọn bộ chân tương ứng.

## Nối dây

| Chân E220 | ESP32 DevKit WROOM-32 | ESP32-S3 |
|---|---|---|
| RXD | GPIO17 (TX của ESP32) | GPIO17 (TX của ESP32-S3) |
| TXD | GPIO16 (RX của ESP32) | GPIO18 (RX của ESP32-S3) |
| M0 | GPIO26 | GPIO4 |
| M1 | GPIO27 | GPIO5 |
| AUX | GPIO25 | GPIO6 |
| VCC | Nguồn 5 V ổn định | Nguồn 5 V ổn định |
| GND | GND chung với ESP32 | GND chung với ESP32-S3 |

- Gắn anten phù hợp vào từng module trước khi thử phát.
- Cấp nguồn đủ dòng cho module; nên dành ít nhất 200 mA cho mỗi E220.
  Có thể dùng chân 5 V của bo nếu đường nguồn trên bo đáp ứng được.
- Tín hiệu RXD, TXD, M0, M1 và AUX dùng mức logic 3,3 V.
  Nguồn VCC 5 V không có nghĩa UART sử dụng tín hiệu 5 V.
- Với ESP32 WROVER hoặc bo tích hợp ngoại vi khác, kiểm tra sơ đồ bo rồi đổi
  `LORA_RX`, `LORA_TX`, `LORA_M0`, `LORA_M1`, `LORA_AUX` nếu có chân bị chiếm.
- M0/M1 nối GPIO theo bảng vì sketch cần điều khiển chế độ cấu hình;
  không nối cố định hai chân này xuống GND khi dùng sketch này.

## Nạp và thử

1. Trong Arduino IDE, cài gói bo **esp32 by Espressif Systems** nếu chưa có.
2. Mở `esp32_e220.ino`.
3. Với ESP32 DevKit, chọn **ESP32 Dev Module**, chọn đúng cổng rồi Upload.
4. Với ESP32-S3, chọn **ESP32S3 Dev Module** và cấu hình flash/PSRAM đúng bo.
   Nếu dùng cổng USB trực tiếp của chip, đặt **USB CDC On Boot → Enabled**.
   Nếu dùng cổng USB qua chip chuyển UART, dùng Serial UART và tắt CDC On Boot.
5. Nạp cùng sketch lên cả hai bo.
6. Mở Serial Monitor của từng bo ở **115200 baud**, chọn **Newline**
   hoặc **Both NL & CR**. Nhấn RESET khi cần xem lại thông báo khởi động.
7. Chờ cả hai bo hiện `READY`. Nhập `Xin chao tu DevKit` trên bo DevKit:
   bo S3 sẽ hiện `[RX ESP32-S3] Xin chao tu DevKit`.
8. Nhập `S3 da nhan` trên bo S3 để thử gửi theo chiều ngược lại.

Hai bo đều có thể gửi và nhận. Gửi luân phiên khi thử; mỗi dòng tối đa 120 byte,
hai lần gửi trên cùng bo cách nhau ít nhất 1,5 giây. Ký tự tiếng Việt có thể
chiếm nhiều byte. Nếu hiện `BUSY`, chờ rồi gửi lại dòng đó.

## Cấu hình radio trong code

Mỗi lần ESP32 khởi động, sketch gửi lệnh `C2` để đặt cấu hình tạm thời giống
nhau trên cả hai module, kiểm tra phản hồi, rồi chuyển về chế độ thường
`M0 = LOW, M1 = LOW`.

- UART: 9600 baud, 8N1; tốc độ radio: 2,4 kbps.
- Địa chỉ của cả hai module: `0x1234`.
- Kênh: `RADIO_CHANNEL = 23`, tương ứng 873,125 MHz.
- Truyền transparent, công suất 22 dBm, khóa giao tiếp bằng 0, tắt byte RSSI.

Muốn đổi tần số, sửa `RADIO_CHANNEL` ở cả hai bo:
`f = 850.125 + RADIO_CHANNEL` MHz.
Ví dụ kênh 18 là 868,125 MHz; kênh 65 là 915,125 MHz.
Địa chỉ và kênh cần giống nhau ở hai đầu.

Cấu hình C2 không lưu qua mất nguồn E220. Nếu E220 được cấp nguồn lại riêng
trong khi ESP32 vẫn chạy, nhấn RESET ESP32 để cấu hình lại module.

## Dùng trong chương trình của bạn

Gọi `sendText("Nhiet do: 28.5 C");` để gửi một dòng.
Giữ `receiveRadio();` trong `loop()` để xử lý dữ liệu nhận.
Đổi phần `Serial.printf(...)` trong `receiveRadio()` nếu muốn xử lý lệnh
thay vì chỉ hiển thị nội dung. Hai đầu dùng ký tự xuống dòng `\n` làm dấu
kết thúc thông điệp; đây là ví dụ gửi văn bản, không phải giao thức dữ liệu nhị phân.

`[TX]` chỉ xác nhận đã đưa dữ liệu sang UART của E220. Sketch chưa có ACK
hoặc gửi lại gói mất; việc hai bo phát cùng lúc có thể gây mất dữ liệu.

Nếu không hiện READY, kiểm tra nguồn, GND chung, RX/TX nối chéo và ba chân
M0/M1/AUX. Nếu hiện READY nhưng không có RX, kiểm tra anten và kênh ở hai đầu.

## Kiểm tra

Đã biên dịch riêng file sketch thành object cho ESP32 và ESP32-S3 với bộ
biên dịch Xtensa và header Arduino ESP32 3.3.12, gồm ESP32-S3 dùng USB CDC
và UART console. Không có lỗi hay cảnh báo trong các lần kiểm tra này.
Chưa kiểm tra liên kết toàn bộ firmware hoặc thử truyền trên phần cứng.

## Tài liệu

- [EBYTE: E220-900T22D User Manual](https://www.ebyte.com/Uploadfiles/Files/2021-7-6/202176188184448.pdf)
- [EBYTE: thông số và chân E220-900T22D](https://www.ebyte.com/product/1076.html)
- [Espressif: Arduino UART / HardwareSerial](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/serial.html)
- [Espressif: USB CDC cho ESP32-S3](https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/cdc_dfu_flash.html)
