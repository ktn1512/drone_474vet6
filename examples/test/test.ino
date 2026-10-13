#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>

/* ================= ĐỊNH NGHĨA CHÂN NỐI ================= */
// UART nối với E220
static const int LORA_RX = 16;  // Nối vào TXD của E220
static const int LORA_TX = 17;  // Nối vào RXD của E220

// Chân điều khiển chế độ E220
static const int LORA_M0 = 25;   // Nối vào M0 của E220
static const int LORA_M1 = 26;   // Nối vào M1 của E220
static const int LORA_AUX = 27;  // Nối vào AUX của E220

/* ================= KHỞI TẠO BIẾN ================= */
HardwareSerial LoRa(1);  // Sử dụng UART1 (hoặc UART2) của ESP32

static char line[128];
static size_t used = 0;
static bool overflowed = false;
static uint32_t received_bytes = 0;
static uint32_t pong_count = 0;
static uint32_t last_info = 0;

// Chờ module rảnh (AUX = HIGH là rảnh, LOW là bận)
void waitAux(uint32_t timeout_ms = 1000) {
  uint32_t start = millis();
  while (digitalRead(LORA_AUX) == LOW) {
    if (millis() - start >= timeout_ms) {
      Serial.println("[WARN] AUX timeout!");
      break;
    }
    delay(1);
  }
}

// Xử lý gói tin khi nhận đủ 1 dòng hoàn chỉnh
static void OnLine(const char *s) {
  Serial.print("[RX RADIO] ");
  Serial.println(s);

  const char prefix[] = "hello";
  if (strncmp(s, prefix, sizeof(prefix) - 1) == 0) {
    char reply[80];
    int n = snprintf(reply, sizeof(reply), "PONG ESP32 %s\r\n", s + sizeof(prefix) - 1);

    if (n > 0 && n < (int)sizeof(reply)) {
      waitAux();  // Đảm bảo module đã nhận xong hoàn toàn và sẵn sàng phát
      delay(20);  // Guard time nhỏ

      size_t sent = LoRa.write((const uint8_t *)reply, (size_t)n);
      Serial.printf("[TX UART] PONG ESP32, bytes=%u/%d\n", (unsigned)sent, n);
      if (sent == (size_t)n) {
        ++pong_count;
      }
    }
  }
}

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

void loop() {
  // Đọc byte nhận từ module LoRa
  while (LoRa.available() > 0) {
    int b = LoRa.read();
    if (b < 0) break;

    ++received_bytes;
    char c = (char)b;

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
  uint32_t now = millis();
  if (now - last_info >= 3000U) {
    last_info = now;
    Serial.printf("[STAT] rx_bytes=%lu, pong_sent=%lu, aux_pin=%d\n",
                  (unsigned long)received_bytes,
                  (unsigned long)pong_count,
                  digitalRead(LORA_AUX));
  }

  delay(1);
}