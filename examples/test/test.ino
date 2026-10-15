/**
 * @file test.ino
 * @brief Ví dụ ESP32/E220 transparent nhận dòng và trả PONG cho tiền tố hello.
 * @details Chương trình hiện so sánh tiền tố "hello" phân biệt hoa/thường,
 * không tự phát PING và không dùng checksum hay lệnh throttle/PID của DroneLink.
 * Thống kê pong_sent chỉ đếm lần UART nhận đủ byte ghi, không xác nhận nhận RF.
 */
#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>

/* ================= ĐỊNH NGHĨA CHÂN NỐI ================= */
// UART nối với E220
/** @brief GPIO16 là đầu RX ESP32, nối TXD của E220. */
static const int LORA_RX = 16;  // Nối vào TXD của E220
/** @brief GPIO17 là đầu TX ESP32, nối RXD của E220. */
static const int LORA_TX = 17;  // Nối vào RXD của E220

// Chân điều khiển chế độ E220
/** @brief GPIO25 điều khiển M0; setup đặt LOW cho chế độ normal. */
static const int LORA_M0 = 25;   // Nối vào M0 của E220
/** @brief GPIO26 điều khiển M1; setup đặt LOW cho chế độ normal. */
static const int LORA_M1 = 26;   // Nối vào M1 của E220
/** @brief GPIO27 đọc AUX; HIGH báo rảnh, LOW báo bận theo chân module. */
static const int LORA_AUX = 27;  // Nối vào AUX của E220

/* ================= KHỞI TẠO BIẾN ================= */
/** @brief Đối tượng UART phần cứng số 1 dành cho E220, cấu hình 9600 baud 8N1. */
HardwareSerial LoRa(1);  // Sử dụng UART1 (hoặc UART2) của ESP32

/** @brief Bộ đệm dòng RX tối đa 127 byte trước NUL; CR bị bỏ, LF kết thúc dòng. */
static char line[128];
/** @brief Số byte hiện lưu trong line, chưa gồm NUL kết thúc. */
static size_t used = 0;
/** @brief Cờ dòng vượt bộ đệm; bỏ phần còn lại đến LF, không gọi OnLine. */
static bool overflowed = false;
/** @brief Tổng byte LoRa.read lấy được, gồm CR/LF và byte của dòng bỏ do overflow. */
static uint32_t received_bytes = 0;
/** @brief Số lần LoRa.write chấp nhận đủ byte PONG; không phải số ACK hay gói RF nhận. */
static uint32_t pong_count = 0;
/** @brief Tick millis lần in thống kê gần nhất, đơn vị ms. */
static uint32_t last_info = 0;

// Chờ module rảnh (AUX = HIGH là rảnh, LOW là bận)
/**
 * @brief Chờ AUX lên HIGH hoặc thoát khi hết thời gian cho phép.
 * @param[in] timeout_ms Thời gian chờ tối đa, ms; mặc định 1000 ms.
 * @note Timeout chỉ in cảnh báo rồi trả về; hàm không báo kết quả và không bảo đảm
 * AUX đã HIGH sau khi trở về. Mỗi lượt chờ có delay(1) ms.
 */
void waitAux(uint32_t timeout_ms = 1000) {
  uint32_t start = millis(); /**< Mốc bắt đầu chờ AUX, ms; dùng hiệu tick để xét timeout. */
  while (digitalRead(LORA_AUX) == LOW) {
    if (millis() - start >= timeout_ms) {
      Serial.println("[WARN] AUX timeout!");
      break;
    }
    delay(1);
  }
}

// Xử lý gói tin khi nhận đủ 1 dòng hoàn chỉnh
/**
 * @brief In dòng RX và thử gửi PONG nếu dòng bắt đầu đúng bằng hello.
 * @param[in] s Chuỗi hợp lệ có NUL kết thúc, không gồm CR/LF.
 * @details PONG gồm "PONG ESP32 ", phần còn lại sau 5 byte "hello" và CRLF.
 * Tiền tố không yêu cầu dấu phân cách, nên "helloworld" cũng được phản hồi.
 * @note Chờ AUX, delay 20 ms rồi LoRa.write; chỉ tăng pong_count nếu write nhận đủ
 * byte. Không flush để đợi TX hoàn tất và không xác nhận radio đầu xa đã nhận.
 */
static void OnLine(const char *s) {
  Serial.print("[RX RADIO] ");
  Serial.println(s);

  const char prefix[] = "hello"; /**< Tiền tố RX đúng theo ví dụ hiện tại, 5 byte phân biệt hoa/thường. */
  if (strncmp(s, prefix, sizeof(prefix) - 1) == 0) {
    char reply[80]; /**< Bộ đệm PONG, gồm suffix của RX, CRLF và NUL định dạng. */
    int n = snprintf(reply, sizeof(reply), "PONG ESP32 %s\r\n", s + sizeof(prefix) - 1); /**< Số byte PONG cần tạo, chưa gồm NUL; không gửi nếu bị cắt. */

    if (n > 0 && n < (int)sizeof(reply)) {
      waitAux();  // Đảm bảo module đã nhận xong hoàn toàn và sẵn sàng phát
      delay(20);  // Guard time nhỏ

      size_t sent = LoRa.write((const uint8_t *)reply, (size_t)n); /**< Số byte UART chấp nhận từ buffer; chưa chứng minh nhận qua RF. */
      Serial.printf("[TX UART] PONG ESP32, bytes=%u/%d\n", (unsigned)sent, n);
      if (sent == (size_t)n) {
        ++pong_count;
      }
    }
  }
}

/**
 * @brief Khởi tạo cổng debug, chân mode/AUX và UART E220, rồi in cấu hình.
 * @details Debug dùng 115200 baud, E220 dùng 9600 baud 8N1. M0/M1 được đặt LOW;
 * waitAux có thể hết hạn mà setup vẫn tiếp tục. Hàm không đọc/ghi thanh ghi RF.
 */
void setup() {
  Serial.begin(115200);

  // 1. Cấu hình chân M0, M1 và AUX
  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  pinMode(LORA_AUX, INPUT);

  // Đưa module vào Mode 0: Normal / Transparent (M0 = 0, M1 = 0)
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);

  // Chờ module ổn định sau khi chuyển chế độ
  delay(100);
  waitAux();

  // 2. Khởi tạo UART LoRa ở tốc độ 9600 8N1
  LoRa.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);
  delay(500);

  Serial.println("==========================================");
  Serial.println(" ESP32 DEVKIT - E220 TRANSCEIVER READY");
  Serial.printf(" Pins: RX=%d, TX=%d, M0=%d, M1=%d, AUX=%d\n",
                LORA_RX, LORA_TX, LORA_M0, LORA_M1, LORA_AUX);
  Serial.println(" Mode: Normal (M0=0, M1=0), Baud=9600");
  Serial.println("==========================================");

  last_info = millis();
}

/**
 * @brief Lấy byte UART, ghép dòng, xử lý hello/PONG và in thống kê mỗi 3000 ms.
 * @details CR bị bỏ; LF kết thúc dòng, dòng rỗng không được xử lý. Dòng dài quá
 * 127 byte bị bỏ đến LF. Byte NUL được lưu như byte thường nhưng khi OnLine dùng
 * chuỗi C, chuỗi sẽ kết thúc tại NUL đầu tiên. Cuối vòng có delay 1 ms.
 */
void loop() {
  // Đọc byte nhận từ module LoRa
  while (LoRa.available() > 0) {
    int b = LoRa.read(); /**< Byte UART 0..255; âm nếu không đọc được byte. */
    if (b < 0) break;

    ++received_bytes;
    char c = (char)b; /**< Byte nhận để phân loại CR/LF hoặc ghép vào line. */

    if (c == '\r') continue;

    if (c == '\n') {
      if (overflowed) {
        Serial.println("[RX] Buffer overflow, bo qua dong!");
      } else if (used > 0) {
        line[used] = '\0';
        OnLine(line);
      }
      used = 0;
      overflowed = false;
    } else {
      if (!overflowed && used < sizeof(line) - 1) {
        line[used++] = c;
      } else {
        overflowed = true;
      }
    }
  }

  // In thống kê định kỳ mỗi 3 giây
  uint32_t now = millis(); /**< Tick hiện tại, ms; dùng hiệu tick để in thống kê mỗi 3 giây. */
  if (now - last_info >= 3000U) {
    last_info = now;
    Serial.printf("[STAT] rx_bytes=%lu, pong_sent=%lu, aux_pin=%d\n",
                  (unsigned long)received_bytes,
                  (unsigned long)pong_count,
                  digitalRead(LORA_AUX));
  }

  delay(1);
}