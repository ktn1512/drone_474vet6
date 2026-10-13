# Giao tiếp STM32 – ESP32

Firmware STM32: `Core/Src/drone_link.c`, tích hợp trong `Core/Src/main.c`.
Sketch web ESP32: `Core/esp32_drone_web/esp32_drone_web.ino`, giao diện `index_html.h` cùng thư mục.

Mỗi gói là một dòng ASCII: `payload*HH\n`. `HH` là XOR của tất cả byte trong payload, viết thành hai ký tự hex. ESP32 tự tạo checksum khi gửi qua web. STM32 và ESP32 đều kiểm tra checksum trước khi parse. Dòng thiếu/sai trường, quá dài hoặc có giá trị ngoài giới hạn bị bỏ qua.

| Chiều | Payload | Xử lý |
| --- | --- | --- |
| ESP32 → STM32 | `T,seq,throttle` | Cập nhật ga 1000–1700 µs, áp dụng trực tiếp |
| ESP32 → STM32 | `P,seq,id,kp,ki,kd` | Cập nhật bộ PID và reset trạng thái tích phân/đạo hàm của bộ đó |
| ESP32 → STM32 | `G,seq,id` | Đọc bộ PID |
| STM32 → ESP32 | `R,seq,id,kp,ki,kd` | Giá trị PID thực tế, phản hồi cả lệnh đọc và ghi |
| STM32 → ESP32 | `N,seq,id` | Từ chối yêu cầu PID không hợp lệ |
| STM32 → ESP32 | `S,throttle,roll,pitch,yaw,m1,m2,m3,m4` | Ga đã nhận, góc nghiêng (độ), xung motor (µs), định kỳ 1500 ms khi radio có lượt gửi |

`id`: 0 = Angle Roll, 1 = Angle Pitch, 2 = Rate Roll, 3 = Rate Pitch, 4 = Rate Yaw. Kp/Ki/Kd phải hữu hạn và trong khoảng 0–100. `seq` là số thứ tự để ESP32 khớp phản hồi với yêu cầu.

Không còn trạng thái, nút, API hoặc trường ARM/DISARM. Đọc/ghi PID không phụ thuộc mức ga và không đổi ga. Khởi động STM32 ở 1000 µs; ESP32 tự đọc 5 bộ PID sau khởi động. Web hiển thị riêng PID đã nhận từ STM32 và chỉ xác nhận ghi khi nhận đúng gói `R`.

STM32 giữ ga hợp lệ gần nhất khi mất radio hoặc không nhận thêm lệnh; đóng web/mất Wi-Fi không tự đưa ga về 1000 µs. Nút **Ga 1000 µs** gửi lệnh ga tối thiểu. Ở 1000 µs, cả bốn motor được đặt 1000 µs và reset PID. Lỗi đọc IMU vẫn đưa ga và xung motor về 1000 µs. Trên 1000 µs, mixer PID hiện có tính xung riêng cho từng motor.

Phản hồi PID được xếp hàng, chờ UART rảnh và có ưu tiên trước telemetry. STM32 giữ buffer truyền bất đồng bộ và cách các lần gửi ít nhất 350 ms. ESP32 gửi ga mới ở lượt radio tiếp theo; ngoài tác vụ PID, nhắc lại ga mỗi 1900 ms. UART vẫn dùng 9600 baud 8N1; cấu hình và chân radio hiện có được giữ nguyên.

Giao thức mới cần nạp cả STM32 và ESP32; gói cũ có trường ARM không tương thích.

Kiểm tra parser STM32 trên máy:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -Itests/drone_link_hal -ICore/Inc tests/test_drone_link.c Core/Src/drone_link.c Core/Src/pid.c -lm -o Debug/test_drone_link.exe
.\Debug\test_drone_link.exe
```

Kiểm tra trên board: gửi ga 1000 µs, đọc 5 PID, ghi và đọc lại PID, thay đổi ga rồi kiểm tra ga/góc/xung motor trả về web. Build và kiểm tra parser trên máy không xác nhận đường truyền radio hoặc PWM thực tế.
