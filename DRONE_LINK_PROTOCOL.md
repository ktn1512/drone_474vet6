# Giao tiếp STM32 – ESP32

Firmware STM32: `Core/Src/drone_link.c`, tích hợp trong `Core/Src/main.c`.
Sketch web ESP32: `Core/esp32_drone_web/esp32_drone_web.ino`, giao diện `index_html.h` cùng thư mục.

Mỗi gói là một dòng ASCII: `payload*HH\n`. `HH` là XOR của tất cả byte trong payload, viết thành hai ký tự hex. ESP32 tự tạo checksum khi gửi qua web. STM32 và ESP32 đều kiểm tra checksum trước khi parse. Dòng thiếu/sai trường, quá dài hoặc có giá trị ngoài giới hạn bị bỏ qua.

| Chiều | Payload | Xử lý |
| --- | --- | --- |
| ESP32 → STM32 | `T,seq,throttle` | Cập nhật ga 1000–1700 µs, áp dụng trực tiếp |
| ESP32 → STM32 | `P,seq,id,kp,ki,kd` | Cập nhật bộ PID và reset trạng thái tích phân/đạo hàm của bộ đó |
| ESP32 → STM32 | `G,seq,id` | Đọc bộ PID |
| STM32 → ESP32 | `R,seq,id,kp,ki,kd` | Giá trị PID thực tế, phản hồi lệnh đọc G |
| STM32 → ESP32 | `A,seq,id` | Xác nhận ghi PID thành công |
| STM32 → ESP32 | `N,seq,id` | Từ chối yêu cầu PID không hợp lệ |
| STM32 → ESP32 | `S,seq,throttle,roll,pitch,yaw,m1,m2,m3,m4[,D,...]` | Một phản hồi cho lệnh T, kèm log chẩn đoán pitch nếu có mẫu |

`id`: 0 = Angle Roll, 1 = Angle Pitch, 2 = Rate Roll, 3 = Rate Pitch, 4 = Rate Yaw. Kp/Ki/Kd phải hữu hạn và trong khoảng 0–100. `seq` là số thứ tự để ESP32 khớp phản hồi với yêu cầu.

Không còn trạng thái, nút, API hoặc trường ARM/DISARM. Đọc/ghi PID không phụ thuộc mức ga và không đổi ga. Khởi động STM32 ở 1000 µs; ESP32 tự đọc 5 bộ PID sau khởi động. Web hiển thị riêng PID đã nhận từ STM32 và chỉ xác nhận ghi khi nhận đúng gói `A`.

STM32 đưa ga về 1000 µs khi không nhận T hợp lệ trong hơn 4500 ms. ESP32 cũng yêu cầu ga tối thiểu khi web hoặc telemetry quá hạn theo các ngưỡng trong sketch. Nút **Ga 1000 µs** gửi lệnh ga tối thiểu. Ở 1000 µs, cả bốn motor được đặt 1000 µs và reset PID. Lỗi đọc IMU đưa ga và xung motor về 1000 µs. Trên 1000 µs, mixer PID tính xung riêng cho từng motor.

Phản hồi PID có một ô chờ, chờ UART rảnh và có ưu tiên trước telemetry. STM32 giữ bản sao gói trong buffer DMA cho đến khi UART truyền xong. ESP32 gửi ga mới ở lượt radio tiếp theo; ngoài tác vụ PID, nhắc lại ga mỗi 1500 ms. Khoảng nghỉ tối thiểu phía ESP32 là 900 ms. UART radio dùng 9600 baud 8N1.

Log chẩn đoán qua E220:

```text
S,seq,throttle,roll,pitch,yaw,m1,m2,m3,m4,D,w,n,bad,gy,gyMin,gyMax,sp,P,I,D,DMin,DMax,out,dt_ms,dtMin_ms,dtMax_ms*HH
```

- `w`: thời gian tích lũy thực tế, ms; `n`: số vòng điều khiển được lấy mẫu.
- `bad`: số lần PID pitch bị reset vì dữ liệu hoặc dt không hợp lệ, không tính reset ở ga 1000.
- `gy`, `gyMin`, `gyMax`: gyro Y sau trừ bias, hiện tại/nhỏ nhất/lớn nhất, độ/s.
- `sp`: tốc độ pitch yêu cầu từ vòng angle, độ/s.
- `P`, `I`, `D`: từng thành phần của rate pitch; `DMin`, `DMax`: đỉnh D trong cửa sổ; đơn vị µs.
- `out`: tổng hiệu chỉnh pitch sau giới hạn đầu ra, µs; có thể khác tổng P+I+D nếu bão hòa.
- `dt_ms`, `dtMin_ms`, `dtMax_ms`: chu kỳ điều khiển hiện tại/nhỏ nhất/lớn nhất, ms.

STM32 ghi nhận từng vòng điều khiển và chỉ xóa cửa sổ khi gói S được HAL chấp nhận gửi DMA. Nếu radio bận hoặc đang ưu tiên PID, thống kê tiếp tục tích lũy. Việc HAL chấp nhận không chứng minh ESP32 đã nhận RF. Gửi chung S và log thành một gói theo nhịp T khoảng 1,5 giây; không phát log 10 Hz qua radio. Số thực chẩn đoán có ba chữ số có nghĩa và có thể dùng dạng số mũ để giữ gói ngắn, tối đa 256 byte kể cả checksum/newline.

Nạp cả STM32 và sketch ESP32 mới để nhận phần mở rộng. ESP32 vẫn chấp nhận S cũ không có phần D; S mở rộng lỗi bị từ chối trước khi cập nhật góc/trạng thái kết nối. Serial Monitor ESP32 ở 115200 baud sẽ in dòng `RX < S,...` và dòng `PITCH ...` có tên từng đại lượng. Dashboard tiếp tục cập nhật ga/góc/motor, và ô dòng RX cuối cùng chứa toàn bộ gói.

Ở ga 1000, P/I/D được lấy mẫu trước bước reset để chẩn đoán; chúng không được áp dụng lên motor. Thời gian cửa sổ có thể dài hơn chu kỳ T nếu gửi bị trì hoãn. Lỗi IMU vẫn có thể trả S thường với motor 1000.

Giao thức mới cần nạp cả STM32 và ESP32; gói cũ có trường ARM không tương thích.

Build STM32 trong môi trường đã có `make` và `arm-none-eabi-gcc`:

```powershell
make -C Debug -j4 all
```

Kiểm tra trên board: gửi ga 1000 µs, đọc 5 PID, ghi và đọc lại PID, thay đổi ga rồi kiểm tra ga/góc/xung motor trả về web. Build và kiểm tra parser trên máy không xác nhận đường truyền radio hoặc PWM thực tế.
