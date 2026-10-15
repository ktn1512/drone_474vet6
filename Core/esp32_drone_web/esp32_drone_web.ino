/**
 * @file esp32_drone_web.ino
 * @brief Cầu nối dashboard HTTP với giao thức DroneLink qua E220 UART.
 * @details Lệnh T đặt ga; P/G ghi/đọc PID; S là telemetry, A/N/R xác nhận hoặc phản hồi PID.
 */
#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include "index_html.h"

// Arduino-ESP32: ESP32 DevKit / ESP32-S3, E220-900T22D in transparent mode.
// Keep the module's stored settings. Address, channel, air rate and key must
// match the STM32-side E220; UART must be 9600 8N1, appended RSSI disabled.
constexpr char WIFI_SSID[] = "DESKTOP-7F2FABB 9674"; /**< Tên mạng Wi-Fi để ESP32 kết nối ở chế độ station. */
constexpr char WIFI_PASS[] = "khanhkhanh"; /**< Mật khẩu mạng Wi-Fi của cấu hình hiện tại. */
constexpr uint32_t WIFI_RETRY_MS = 10000; /**< Khoảng cách tối thiểu giữa các lần yêu cầu kết nối lại Wi-Fi, ms. */
#if defined(CONFIG_IDF_TARGET_ESP32S3)
constexpr int E220_RX = 18; /**< GPIO RX của ESP32, nối tới TXD của E220; giá trị tùy dòng chip. */  // ESP32-S3 RX <- E220 TXD
constexpr int E220_TX = 17; /**< GPIO TX của ESP32, nối tới RXD của E220; giá trị tùy dòng chip. */  // ESP32-S3 TX -> E220 RXD
constexpr int E220_M0 = 4; /**< GPIO điều khiển M0 của E220, được đặt LOW trong setup(). */
constexpr int E220_M1 = 5; /**< GPIO điều khiển M1 của E220, được đặt LOW trong setup(). */
constexpr int E220_AUX = 6; /**< GPIO đọc AUX; giá trị âm bỏ qua kiểm tra AUX. */  // Set to -1 if AUX is not connected.
#elif defined(CONFIG_IDF_TARGET_ESP32)
constexpr int E220_RX = 16; /**< GPIO RX của ESP32, nối tới TXD của E220; giá trị tùy dòng chip. */  // ESP32 RX2 <- E220 TXD
constexpr int E220_TX = 17; /**< GPIO TX của ESP32, nối tới RXD của E220; giá trị tùy dòng chip. */  // ESP32 TX2 -> E220 RXD
constexpr int E220_M0 = 25; /**< GPIO điều khiển M0 của E220, được đặt LOW trong setup(). */  // Same wiring as successful ESP32 E220 test
constexpr int E220_M1 = 26; /**< GPIO điều khiển M1 của E220, được đặt LOW trong setup(). */
constexpr int E220_AUX = 27; /**< GPIO đọc AUX; giá trị âm bỏ qua kiểm tra AUX. */  // Set to -1 if AUX is not connected.
#else
#error "Select ESP32 Dev Module or ESP32S3 Dev Module."
#endif
constexpr uint32_t UART_BAUD = 9600; /**< Tốc độ UART tới E220, đơn vị baud; định dạng 8N1. */
constexpr uint16_t MIN_THR = 1000; /**< Giới hạn dưới lệnh ga, đơn vị microsecond. */
constexpr uint16_t MAX_THR = 1700; /**< Giới hạn trên lệnh ga, đơn vị microsecond. */
constexpr uint16_t MAX_MOTOR_STATUS_US = 2000; /**< Giới hạn trên của giá trị motor được chấp nhận trong gói S, microsecond. */
/* Half-duplex E220 2.4kbps: nho de packet R/S co cua so phan hoi. */
constexpr uint32_t THROTTLE_PERIOD_MS = 1500; /**< Chu kỳ yêu cầu T khi không có giao dịch PID, ms. */
constexpr uint32_t PID_HEARTBEAT_MS = 2500; /**< Khoảng thời gian chèn T giữa các giao dịch PID, ms. */
constexpr uint32_t CLIENT_TIMEOUT_MS = 3000; /**< Thời gian không có cập nhật web trước khi đặt ga tối thiểu, ms. */
constexpr uint32_t STATUS_VALID_MS = 6500; /**< Tuổi tối đa của status S hợp lệ để freshStatus() trả true, ms. */
constexpr uint32_t MIN_RADIO_GAP_MS = 900; /**< Khoảng cách tối thiểu giữa hai lần gửi UART E220, ms. */
constexpr uint32_t AUX_SETTLE_MS = 3; /**< Thời gian AUX cần giữ HIGH trước khi cho phép gửi, ms. */
constexpr uint32_t RX_PARTIAL_TIMEOUT_MS = 1200; /**< Thời gian tối đa chờ thêm byte của dòng chưa hoàn tất, ms. */
constexpr size_t MAX_RADIO_PACKET_BYTES = 200; /**< Số byte lớn nhất sendRadioBytes() cho phép gửi trong một lần. */
constexpr uint32_t ACK_TIMEOUT_MS = 2200; /**< Thời gian chờ A/N/R trước khi thử lại giao dịch PID, ms. */
constexpr uint8_t MAX_ATTEMPTS = 4; /**< Tổng số lần gửi tối đa cho mỗi PID, gồm lần gửi đầu tiên. */

WebServer web(80); /**< HTTP server phục vụ dashboard và API tại cổng 80. */
HardwareSerial LoRa(2); /**< UART phần cứng số 2 dùng giao tiếp E220, không phải thư viện LoRa RF. */

/** @brief Bộ gain kp/ki/kd của một PID; ID 0/1 là vòng góc, 2/3/4 là vòng tốc độ. */
struct PidGains {
  float kp /**< Gain tỉ lệ. */, ki /**< Gain tích phân. */, kd /**< Gain đạo hàm. */;
};
/** @brief Gain hiển thị trên dashboard; cập nhật từ R hoặc sau A xác nhận ghi PID. */
PidGains pids[5] = {
  { 10.0f, 0.0f, 0.0f },  // Angle Roll
  { 10.0f, 0.0f, 0.0f },  // Angle Pitch
  { 0.5f, 1.4f, 0.0f },   // Rate Roll
  { 0.5f, 1.4f, 0.0f },   // Rate Pitch
  { 0.3f, 0.0f, 0.0f }    // Rate Yaw
};

/* Throttle truc tiep, khong co ARM/DISARM. */
uint16_t throttle = MIN_THR; /**< Lệnh ga đang yêu cầu, đơn vị microsecond. */
uint32_t lastWebMs = 0; /**< Mốc hoạt động web gần nhất, ms từ millis(). */
uint32_t lastTxMs = 0; /**< Mốc UART hoàn tất lần gửi E220 gần nhất, ms. */
uint32_t lastThrottleTxMs = 0; /**< Mốc gửi gói T thành công qua UART gần nhất, ms. */
uint32_t nextSeq = 1; /**< Sequence cấp cho lệnh mới; lệnh PID thử lại giữ nguyên waitingSeq. */

bool statusSeen = false; /**< Đã nhận ít nhất một gói S hợp lệ. */
uint32_t lastStatusMs = 0; /**< Mốc nhận gói S hợp lệ gần nhất, ms. */
uint32_t validStatusCount = 0; /**< Tổng số gói S vượt qua kiểm tra checksum và nội dung. */
uint32_t invalidStatusCount = 0; /**< Số gói S sai checksum hoặc sai các trường dữ liệu. */
String statusDiagnostic = "Chua nhan goi status S hop le"; /**< Thông báo chẩn đoán gói status mới nhất cho dashboard. */
uint16_t rxThrottle = MIN_THR; /**< Ga báo về trong gói S gần nhất, microsecond. */
float rxRoll = 0, rxPitch = 0, rxYaw = 0; /**< Góc roll/pitch/yaw báo về từ STM32, đơn vị độ. */
uint16_t rxMotors[4] = {1000, 1000, 1000, 1000}; /**< PWM m1..m4 báo về trong gói S, đơn vị microsecond. */

/** @brief Trạng thái tác vụ PID: OP_NONE rảnh, OP_APPLY ghi, OP_READ đọc từ STM32. */
enum OpType { OP_NONE, OP_APPLY, OP_READ };
OpType operation = OP_NONE; /**< Loại giao dịch PID hiện đang thực hiện. */
PidGains pendingPids[5]; /**< Gain nhập từ web, chờ STM32 xác nhận trước khi cập nhật pids. */
uint8_t opIndex = 0; /**< ID PID đang xử lý, từ 0 đến 4; 5 đánh dấu đã xong cả bộ. */
uint8_t attempts = 0; /**< Số lần đã gửi yêu cầu cho PID hiện tại. */
uint32_t waitingSeq = 0; /**< Sequence của yêu cầu PID đang đợi A/N/R phản hồi. */
uint32_t lastOpMs = 0; /**< Mốc gửi yêu cầu PID gần nhất, ms; dùng tính timeout. */
bool waiting = false; /**< Đang chờ phản hồi của yêu cầu PID hiện tại. */
String opText = "Không có yêu cầu PID"; /**< Thông báo tiến độ hoặc lỗi của giao dịch PID. */

char rxLine[256]; /**< Bộ đệm gom một dòng radio, chừa một byte NUL cuối. */
size_t rxLength = 0; /**< Số byte payload đang có trong rxLine, chưa tính NUL. */
bool rxDiscard = false; /**< Bỏ các byte tới newline sau khi dòng lỗi hoặc quá dài. */
uint32_t lastRxByteMs = 0; /**< Mốc nhận byte UART gần nhất, ms. */
bool radioStarted = false; /**< UART LoRa đã khởi tạo thành công. */
bool auxWasHigh = false; /**< Trạng thái AUX HIGH đã được quan sát ở lần kiểm tra trước. */
uint32_t auxHighSinceMs = 0; /**< Mốc đầu tiên AUX chuyển HIGH để tính thời gian ổn định, ms. */
uint32_t radioTxCount = 0; /**< Số lần ghi đủ gói vào UART; không chứng minh phía STM32 đã nhận RF. */
uint32_t radioRxCount = 0; /**< Số dòng UART không rỗng nhận hoàn chỉnh, gồm cả dòng chưa hợp lệ. */
uint32_t radioRxByteCount = 0; /**< Tổng số byte đọc được từ UART E220. */
uint32_t radioRxErrors = 0; /**< Số lỗi checksum/định dạng dòng, tràn bộ đệm hoặc dòng bị timeout. */
uint32_t radioTxErrors = 0; /**< Số lỗi khởi tạo UART, kích thước gói hoặc ghi thiếu byte. */
uint32_t lastRadioRxMs = 0; /**< Mốc hoàn tất dòng radio gần nhất, ms. */
String lastRadioRx; /**< Nội dung dòng radio nhận gần nhất để hiển thị chẩn đoán. */
String lastRadioTx; /**< Mô tả payload gửi UART gần nhất để hiển thị chẩn đoán. */

/**
 * @brief Chuyển chuỗi thành literal JSON, escape ký tự điều khiển và byte UTF-8 không hợp lệ.
 * @param[in] value Chuỗi đầu vào cần mã hóa.
 * @return Chuỗi JSON đã có dấu nháy kép bao ngoài.
 */
String jsonString(const String &value) {
  String escaped; /**< Chuỗi JSON có dấu nháy bao ngoài và các ký tự đã được escape. */
  escaped.reserve(value.length() + 2);
  escaped += '"';
  for (size_t i = 0; i < value.length(); ++i) {
    const uint8_t c = static_cast<uint8_t>(value[i]); /**< Byte đầu của ký tự đang xét; có thể là ASCII hoặc byte đầu UTF-8. */
    if (c == '"' || c == '\\') {
      escaped += '\\';
      escaped += static_cast<char>(c);
    } else if (c >= 0x80) {
      // Preserve valid UTF-8 text; escape isolated bytes from binary/noisy RX.
      /* count là số byte dự kiến của ký tự UTF-8: 2, 3, 4; bằng 0 nếu byte đầu sai. */
      const size_t count = c >= 0xC2 && c <= 0xDF ? 2 : c >= 0xE0 && c <= 0xEF ? 3
                                                      : c >= 0xF0 && c <= 0xF4 ? 4
                                                                               : 0;
      bool valid = count != 0 && i + count <= value.length(); /**< Chuỗi byte hiện tại có khả năng là một ký tự UTF-8 đầy đủ và hợp lệ. */
      for (size_t j = 1; valid && j < count; ++j) {
        const uint8_t next = static_cast<uint8_t>(value[i + j]); /**< Byte tiếp theo của ký tự UTF-8 đang kiểm tra. */
        valid = next >= 0x80 && next <= 0xBF;
        if (j == 1)
          valid = valid && !(c == 0xE0 && next < 0xA0) && !(c == 0xED && next > 0x9F) && !(c == 0xF0 && next < 0x90) && !(c == 0xF4 && next > 0x8F);
      }
      if (valid) {
        for (size_t j = 0; j < count; ++j)
          escaped += value[i + j];
        i += count - 1;
      } else {
        char hex[7]; /**< Bộ đệm escape JSON dạng \u00HH, bao gồm NUL kết thúc. */
        snprintf(hex, sizeof(hex), "\\u%04X", static_cast<unsigned>(c));
        escaped += hex;
      }
    } else if (c < 0x20 || c == 0x7F) {
      // Radio input may contain arbitrary bytes; always emit valid JSON.
      char hex[7]; /**< Bộ đệm escape JSON dạng \u00HH, bao gồm NUL kết thúc. */
      snprintf(hex, sizeof(hex), "\\u%04X", static_cast<unsigned>(c));
      escaped += hex;
    } else {
      escaped += static_cast<char>(c);
    }
  }
  escaped += '"';
  return escaped;
}

/**
 * @brief Tính checksum XOR 8 bit của toàn bộ byte trong chuỗi.
 * @param[in] s Payload chưa có hậu tố checksum.
 * @return Giá trị XOR; chuỗi rỗng trả 0.
 */
uint8_t xorChecksum(const String &s) {
  uint8_t v = 0; /**< Giá trị XOR tích lũy trên các byte của chuỗi. */
  for (size_t i = 0; i < s.length(); i++)
    v ^= uint8_t(s[i]);
  return v;
}

/**
 * @brief Tạo frame ASCII dạng payload*HH và kết thúc bằng newline.
 * @param[in] payload Payload DroneLink chưa có checksum/newline.
 * @return Frame sẵn sàng để ghi UART.
 */
String frameOf(const String &payload) {
  char hex[5]; /**< Hậu tố gồm dấu *, hai ký tự checksum hex, newline và NUL. */
  snprintf(hex, sizeof(hex), "*%02X\n", xorChecksum(payload));
  return payload + hex;
}

/**
 * @brief Kiểm tra hậu tố checksum và bỏ hậu tố khỏi chuỗi.
 * @param[in,out] raw Dòng đã bỏ CR/LF; hậu tố bị xóa khi strtoul đọc hết hai ký tự cuối và giá trị <=255, kể cả khi checksum sai.
 * @return true khi định dạng hậu tố và giá trị XOR khớp; false khi sai.
 */
bool decodeFrame(String &raw) {
  if (raw.length() < 4 || raw[raw.length() - 3] != '*')
    return false;
  String hex = raw.substring(raw.length() - 2); /**< Hai ký tự cuối chứa checksum cần đọc ở cơ số 16. */
  char *end = nullptr; /**< Con trỏ tới ký tự đầu tiên chưa được hàm chuyển số đọc. */
  unsigned long want = strtoul(hex.c_str(), &end, 16); /**< Checksum mong đợi đọc từ hai ký tự hex cuối dòng. */
  if (end != hex.c_str() + 2 || want > 255)
    return false;
  raw.remove(raw.length() - 3);
  return xorChecksum(raw) == want;
}

/**
 * @brief Kiểm tra AUX HIGH đủ thời gian ổn định; cập nhật trạng thái theo dõi AUX.
 * @param[in] now Mốc millis() hiện tại, đơn vị ms.
 * @return true nếu bỏ qua AUX hoặc AUX HIGH ít nhất AUX_SETTLE_MS.
 */
bool radioAuxReady(uint32_t now) {
  if (E220_AUX < 0)
    return true;
  if (digitalRead(E220_AUX) == LOW) {
    auxWasHigh = false;
    return false;
  }
  if (!auxWasHigh) {
    auxWasHigh = true;
    auxHighSinceMs = now;
  }
  return (uint32_t)(now - auxHighSinceMs) >= AUX_SETTLE_MS;
}

/**
 * @brief Kiểm tra UART đã khởi tạo, AUX sẵn sàng và đủ khoảng cách gửi.
 * @param[in] now Mốc millis() hiện tại, đơn vị ms.
 * @return true khi có thể bắt đầu lần gửi UART tiếp theo.
 */
bool radioReady(uint32_t now) {
  return radioStarted && radioAuxReady(now) && (uint32_t)(now - lastTxMs) >= MIN_RADIO_GAP_MS;
}

/**
 * @brief Ghi nguyên gói vào UART E220 và chờ UART TX hoàn tất.
 * @param[in] packet Chuỗi byte cần gửi, dài 1..MAX_RADIO_PACKET_BYTES.
 * @param[in] description Payload dùng lưu và in log chẩn đoán.
 * @return true khi ghi đủ byte vào UART; false nếu bận, kích thước sai hoặc ghi thiếu.
 * @note Kết quả true và AUX HIGH không phải ACK từ STM32.
 */
bool sendRadioBytes(const String &packet, const String &description) {
  uint32_t now = millis(); /**< Mốc thời gian hiện tại dùng tính các khoảng thời gian, ms. */
  if (!radioReady(now))
    return false;
  if (packet.length() == 0 || packet.length() > MAX_RADIO_PACKET_BYTES) {
    ++radioTxErrors;
    return false;
  }
  /* written là số byte UART đã chấp nhận; so với packet.length() để phát hiện ghi thiếu. */
  const size_t written = LoRa.write(
    reinterpret_cast<const uint8_t *>(packet.c_str()), packet.length());
  // Wait for UART TX only, preserving incoming RX bytes. AUX is not an ACK.
  LoRa.flush(true);
  lastTxMs = millis();
  if (written != packet.length()) {
    ++radioTxErrors;
    Serial.println("E220 UART write failed");
    return false;
  }
  ++radioTxCount;
  lastRadioTx = description;
  Serial.print("TX > ");
  Serial.println(description);
  return true;
}

/**
 * @brief Bổ sung checksum/newline rồi gửi payload DroneLink.
 * @param[in] payload Lệnh ASCII chưa có hậu tố checksum.
 * @return Kết quả sendRadioBytes(); chỉ phản ánh UART phía ESP32.
 */
bool sendRadio(const String &payload) {
  return sendRadioBytes(frameOf(payload), payload);
}

/**
 * @brief Đặt ga tối thiểu và đánh dấu cần gửi T ở slot radio tiếp theo.
 * @param[in] reason Chuỗi lý do in ra Serial khi ga thực sự thay đổi.
 */
void setMinimumThrottle(const char *reason) {
  if (throttle != MIN_THR) {
    throttle = MIN_THR;
    lastThrottleTxMs = millis() - THROTTLE_PERIOD_MS;
    Serial.print("Throttle -> 1000: ");
    Serial.println(reason);
  }
}

/**
 * @brief Kiểm tra đã nhận gói S hợp lệ và gói đó còn trong STATUS_VALID_MS.
 * @return true khi telemetry còn mới; false nếu chưa có S hợp lệ hoặc đã quá hạn.
 */
bool freshStatus() {
  return statusSeen && (uint32_t)(millis() - lastStatusMs) <= STATUS_VALID_MS;
}

/**
 * @brief Đọc gain từ chuỗi và kiểm tra giá trị hữu hạn trong khoảng 0..100.
 * @param[in] text Chuỗi biểu diễn số thực; không được có ký tự dư cuối chuỗi.
 * @param[out] v Giá trị strtof đọc được; có thể bị ghi ngay cả khi hàm trả false, trừ chuỗi rỗng.
 * @return true khi toàn bộ chuỗi là số hợp lệ trong giới hạn.
 */
bool parseGain(const String &text, float &v) {
  if (!text.length())
    return false;
  char *end = nullptr; /**< Con trỏ tới ký tự đầu tiên chưa được hàm chuyển số đọc. */
  v = strtof(text.c_str(), &end);
  return end != text.c_str() && *end == 0 && isfinite(v) && v >= 0 && v <= 100;
}

/**
 * @brief Trả HTTP 200 với JSON xác nhận yêu cầu web hợp lệ.
 */
void respondOk() {
  web.send(200, "application/json", "{\"ok\":true}");
}
/**
 * @brief Trả HTTP lỗi với thông báo đã được mã hóa JSON.
 * @param[in] reason Thông báo lỗi để dashboard hiển thị.
 * @param[in] status Mã trạng thái HTTP, mặc định 400.
 */
void respondError(const String &reason, int status = 400) {
  web.send(status, "application/json", "{\"ok\":false,\"error\":" + jsonString(reason) + "}");
}

/* Optional S suffix: ,D,w,n,bad,gy,gyMin,gyMax,sp,P,I,D,DMin,DMax,out,dt,dtMin,dtMax. */
bool parsePitchDiagnostic(const char *suffix, uint32_t counts[3], float values[13]) {
  if (strncmp(suffix, ",D,", 3) != 0) return false;
  const char *cursor = suffix + 3;
  for (size_t i = 0; i < 3; ++i) {
    const char *start = cursor;
    uint32_t value = 0;
    while (*cursor >= '0' && *cursor <= '9') {
      const uint32_t digit = static_cast<uint32_t>(*cursor++ - '0');
      if (value > (UINT32_MAX - digit) / 10U) return false;
      value = value * 10U + digit;
    }
    if (cursor == start || *cursor != ',') return false;
    counts[i] = value;
    ++cursor;
  }
  for (size_t i = 0; i < 13; ++i) {
    char *end = nullptr;
    values[i] = strtof(cursor, &end);
    const char delimiter = i == 12 ? '\0' : ',';
    if (end == cursor || *end != delimiter || !isfinite(values[i])) return false;
    cursor = end + (i == 12 ? 0 : 1);
  }
  return counts[1] > 0 && counts[2] <= counts[1] &&
         values[1] <= values[2] && values[7] <= values[8] &&
         values[11] <= values[12];
}

/**
 * @brief Kiểm tra và xử lý dòng A/N/R/S nhận từ STM32.
 * @param[in] frame Bản sao dòng radio, không gồm CR/LF; decodeFrame() có thể sửa bản sao này.
 * @note A/N/R phải khớp sequence và ID đang chờ; S phải đúng checksum và giới hạn dữ liệu.
 */
void onFrame(String frame) {
  if (frame.indexOf('*') < 0) return;  // legacy raw radio test data
  if (!decodeFrame(frame)) {
    ++radioRxErrors;
    if (frame.startsWith("S,")) {
      ++invalidStatusCount;
      statusDiagnostic = "Sai checksum status S";
    }
    return;
  }

  const char kind = frame[0]; /**< Loại phản hồi DroneLink: A, N, R hoặc S. */
  unsigned long seq = 0; /**< Sequence đọc từ frame để đối chiếu giao dịch PID. */
  unsigned id = 0; /**< ID PID đọc từ phản hồi A/N/R. */
  int pos = 0; /**< Vị trí cuối sscanf ghi qua %n; dùng phát hiện dữ liệu dư. */

  if ((kind == 'A' || kind == 'N') && operation == OP_APPLY && waiting &&
      sscanf(frame.c_str(), "%*c,%lu,%u%n", &seq, &id, &pos) == 2 &&
      pos == (int)frame.length() && seq == waitingSeq && id == opIndex) {
    waiting = false;
    if (kind == 'N') {
      operation = OP_NONE;
      lastThrottleTxMs = millis() - THROTTLE_PERIOD_MS;
      opText = "STM32 từ chối PID " + String(id);
    } else {
      pids[id] = pendingPids[id];  // ghi vao UI sau ACK that
      ++opIndex;
      attempts = 0;
      if (opIndex == 5) {
        operation = OP_NONE;
        lastThrottleTxMs = millis() - THROTTLE_PERIOD_MS;
        opText = "Đã gửi PID: 5/5 ACK";
      }
    }
  } else if (kind == 'R' && operation == OP_READ && waiting) {
    float kp = 0, ki = 0, kd = 0; /**< Gain kp/ki/kd đọc từ gói R; chỉ cập nhật pids sau khi kiểm tra đầy đủ. */
    pos = 0;
    if (sscanf(frame.c_str(), "R,%lu,%u,%f,%f,%f%n", &seq, &id, &kp, &ki, &kd, &pos) == 5 &&
        pos == (int)frame.length() && seq == waitingSeq && id == opIndex && id < 5 &&
        isfinite(kp) && isfinite(ki) && isfinite(kd) &&
        kp >= 0 && kp <= 100 && ki >= 0 && ki <= 100 && kd >= 0 && kd <= 100) {
      pids[id] = {kp, ki, kd};
      waiting = false;
      ++opIndex;
      attempts = 0;
      if (opIndex == 5) {
        operation = OP_NONE;
        lastThrottleTxMs = millis() - THROTTLE_PERIOD_MS;
        opText = "Đã đọc PID từ drone: 5/5";
      }
    }
  } else if (kind == 'S') {
    // Legacy S or S followed by the optional ,D diagnostic suffix.
    unsigned thr = 0, m1 = 0, m2 = 0, m3 = 0, m4 = 0; /**< Ga và PWM bốn động cơ đọc từ S, đơn vị microsecond. */
    float r = 0, p = 0, y = 0; /**< Góc roll/pitch/yaw đọc từ S, đơn vị độ. */
    uint32_t diagnosticCounts[3] = {}; // w, n, bad
    float diagnosticValues[13] = {}; // gy,min,max,sp,P,I,D,min,max,out,dt,min,max
    pos = 0;
    const bool baseParsed = sscanf(frame.c_str(), "S,%lu,%u,%f,%f,%f,%u,%u,%u,%u%n",
                                   &seq, &thr, &r, &p, &y, &m1, &m2, &m3, &m4, &pos) == 9;
    const bool hasDiagnostic = baseParsed && pos < (int)frame.length();
    const bool suffixValid = baseParsed &&
        (!hasDiagnostic || parsePitchDiagnostic(frame.c_str() + pos, diagnosticCounts, diagnosticValues));
    if (baseParsed && suffixValid &&
        thr >= MIN_THR && thr <= MAX_THR &&
        isfinite(r) && isfinite(p) && isfinite(y) &&
        m1 >= MIN_THR && m1 <= MAX_MOTOR_STATUS_US &&
        m2 >= MIN_THR && m2 <= MAX_MOTOR_STATUS_US &&
        m3 >= MIN_THR && m3 <= MAX_MOTOR_STATUS_US &&
        m4 >= MIN_THR && m4 <= MAX_MOTOR_STATUS_US) {
      rxThrottle = (uint16_t)thr;
      rxRoll = r;
      rxPitch = p;
      rxYaw = y;
      rxMotors[0] = m1;
      rxMotors[1] = m2;
      rxMotors[2] = m3;
      rxMotors[3] = m4;
      lastStatusMs = millis();
      statusSeen = true;
      ++validStatusCount;
      statusDiagnostic = "Status S hợp lệ";
      if (hasDiagnostic) {
        Serial.printf(
          "PITCH seq=%lu w=%lu n=%lu bad=%lu thr=%u ang=%.1f,%.1f "
          "gy=%.3g[%.3g,%.3g] sp=%.3g P=%.3g I=%.3g "
          "D=%.3g[%.3g,%.3g] out=%.3g dt_ms=%.3g[%.3g,%.3g]\n",
          seq, (unsigned long)diagnosticCounts[0],
          (unsigned long)diagnosticCounts[1], (unsigned long)diagnosticCounts[2],
          thr, r, p, diagnosticValues[0], diagnosticValues[1], diagnosticValues[2],
          diagnosticValues[3], diagnosticValues[4], diagnosticValues[5],
          diagnosticValues[6], diagnosticValues[7], diagnosticValues[8],
          diagnosticValues[9], diagnosticValues[10], diagnosticValues[11], diagnosticValues[12]);
      }
    } else {
      ++invalidStatusCount;
      statusDiagnostic = "Status S sai số trường hoặc ngoài giới hạn";
      Serial.print("S invalid: ");
      Serial.println(frame);
    }
  }
}

/**
 * @brief Đọc hết byte UART, gom frame theo newline và loại bỏ dòng lỗi/quá hạn.
 */
void pollRadio() {
  uint32_t now = millis(); /**< Mốc thời gian hiện tại dùng tính các khoảng thời gian, ms. */
  radioAuxReady(now);
  while (LoRa.available() > 0) {
    int data = LoRa.read(); /**< Giá trị byte UART đọc được; giá trị âm nghĩa là không đọc được byte. */

    if (data < 0)
      break;

    ++radioRxByteCount;
    char c = (char)data; /**< Ký tự dùng gom dòng radio; newline kết thúc frame, CR bị bỏ qua. */
    lastRxByteMs = millis();

    if (c == '\n') {
      if (!rxDiscard && rxLength > 0) {
        rxLine[rxLength] = '\0';
        lastRadioRx = String(rxLine, rxLength);
        lastRadioRxMs = millis();
        ++radioRxCount;
        Serial.print("RX < ");
        Serial.println(lastRadioRx);
        onFrame(lastRadioRx);
      }
      rxLength = 0;
      rxDiscard = false;
    } else if (c != '\r') {
      if (rxDiscard)
        continue;
      if (c != '\0' && rxLength < sizeof(rxLine) - 1) {
        rxLine[rxLength++] = c;
      } else {
        ++radioRxErrors;
        rxLength = 0;
        rxDiscard = true;
      }
    }
  }
  // Drain buffered UART data first: a slow HTTP request must not turn a
  // complete, already-received frame into an apparent radio timeout.
  if ((rxLength > 0 || rxDiscard) && (uint32_t)(millis() - lastRxByteMs) > RX_PARTIAL_TIMEOUT_MS) {
    if (!rxDiscard)
      ++radioRxErrors;
    rxLength = 0;
    // Stay out of sync until a newline ends the truncated frame.
    rxDiscard = true;
  }
}

/**
 * @brief Khởi tạo tác vụ đọc/ghi lần lượt cả năm PID.
 * @param[in] type OP_READ để đọc, OP_APPLY để ghi; call site hiện tại chỉ dùng hai giá trị này.
 */
void startOperation(OpType type) {
  operation = type;
  opIndex = 0;
  attempts = 0;
  waiting = false;
  opText = type == OP_READ ? "Đang đọc PID: 0/5" : "Đang gửi PID: 0/5";
}

/**
 * @brief Điều phối lệnh T/P/G, timeout và thử lại từng giao dịch PID.
 * @note Không gửi T khi đang chờ phản hồi; khi hết chờ, có thể chèn T trước P/G hoặc trước lần thử lại.
 */
void processRadioTx() {
  uint32_t now = millis(); /**< Mốc thời gian hiện tại dùng tính các khoảng thời gian, ms. */
  if (throttle > MIN_THR && (uint32_t)(now - lastWebMs) > CLIENT_TIMEOUT_MS)
    setMinimumThrottle("Web inactivity");
  if (throttle > MIN_THR && statusSeen && !freshStatus())
    setMinimumThrottle("STM32 telemetry timeout");

  if (!radioReady(now)) return;

  // Only ONE request at a time. T is never sent WHILE waiting for A/N/R.
  // A heartbeat is inserted BETWEEN PID transactions (not over their reply).
  // The STM32 still enforces its own timeout if the RF connection is lost.
  if (operation != OP_NONE) {
    if (waiting) {
      if ((uint32_t)(now - lastOpMs) < ACK_TIMEOUT_MS) return;
      waiting = false;
      if (attempts >= MAX_ATTEMPTS) {
        opText = "Lỗi: PID " + String(opIndex) + " không phản hồi sau 4 lần";
        operation = OP_NONE;
        lastThrottleTxMs = millis() - THROTTLE_PERIOD_MS;
        return;
      }
    }
    // Keep the throttle watchdog fresh without colliding with the A/N/R reply.
    // T gets its own 900ms reply window for S before the next P/G.
    if ((uint32_t)(now - lastThrottleTxMs) >= PID_HEARTBEAT_MS) {
      if (sendRadio("T," + String(nextSeq++) + "," + String(throttle)))
        lastThrottleTxMs = millis();
      return;
    }
    if (attempts == 0) waitingSeq = nextSeq++;
    String payload; /**< Lệnh P hoặc G cho PID hiện tại, chưa có checksum/newline. */
    if (operation == OP_APPLY) {
      const PidGains &p = pendingPids[opIndex]; /**< Tham chiếu bộ gain đang chờ ghi cho ID PID hiện tại. */
      payload = "P," + String(waitingSeq) + "," + String(opIndex) + "," +
                String(p.kp, 4) + "," + String(p.ki, 4) + "," + String(p.kd, 4);
    } else {
      payload = "G," + String(waitingSeq) + "," + String(opIndex);
    }
    if (sendRadio(payload)) {
      waiting = true;
      ++attempts;
      lastOpMs = millis();
      opText = String(operation == OP_APPLY ? "Đang gửi PID " : "Đang đọc PID ") +
               String(opIndex + 1) + "/5, lần " + String(attempts);
    }
    return;
  }

  // No independent STM32 S transmitter anymore. Every T requests exactly
  // one S; 1.5-s slots leave airtime for the S reply at low E220 air rates.
  if ((uint32_t)(now - lastThrottleTxMs) >= THROTTLE_PERIOD_MS) {
    if (sendRadio("T," + String(nextSeq++) + "," + String(throttle)))
      lastThrottleTxMs = millis();
  }
}

/**
 * @brief Đăng ký trang dashboard và các API trạng thái, ga, ghi/đọc PID.
 */
void setupWebRoutes() {
  /* GET /: trả nguyên trang HTML được lưu trong flash. */
  web.on("/", HTTP_GET, []() {
    web.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
  });
  /* GET /api/state: đọc snapshot trạng thái và cập nhật mốc hoạt động của client. */
  web.on("/api/state", HTTP_GET, []() {
    lastWebMs = millis();
    String json; /**< Chuỗi JSON snapshot trạng thái trả về cho GET /api/state. */
    json.reserve(2200);
    json = "{\"throttle\":" + String(throttle) +
           ",\"link\":" + String(freshStatus() ? "true" : "false") +
           ",\"operation\":" + jsonString(opText) + ",\"pids\":[";
    for (int i = 0; i < 5; i++) {
      if (i) json += ',';
      json += "{\"kp\":" + String(pids[i].kp, 4) +
              ",\"ki\":" + String(pids[i].ki, 4) +
              ",\"kd\":" + String(pids[i].kd, 4) + "}";
    }
    json += "],\"pidBusy\":" + String(operation != OP_NONE ? "true" : "false");
    const uint32_t now = millis(); /**< Mốc ms dùng tính tuổi status và dòng radio trong phản hồi JSON. */
    json += ",\"statusSeen\":" + String(statusSeen ? "true" : "false");
    json += ",\"statusAgeMs\":" + (statusSeen ? String((uint32_t)(now - lastStatusMs)) : String("null"));
    json += ",\"statusValidCount\":" + String(validStatusCount);
    json += ",\"statusInvalidCount\":" + String(invalidStatusCount);
    json += ",\"statusDiagnostic\":" + jsonString(statusDiagnostic);
    json += ",\"radio\":{\"ready\":" + String(radioReady(now) ? "true" : "false") +
            ",\"auxConnected\":" + String(E220_AUX >= 0 ? "true" : "false") +
            ",\"auxReady\":" + String(radioAuxReady(now) ? "true" : "false") +
            ",\"txCount\":" + String(radioTxCount) +
            ",\"rxCount\":" + String(radioRxCount) +
            ",\"rxBytes\":" + String(radioRxByteCount) +
            ",\"rxErrors\":" + String(radioRxErrors) +
            ",\"txErrors\":" + String(radioTxErrors) +
            ",\"rxAgeMs\":" + (radioRxCount ? String((uint32_t)(now - lastRadioRxMs)) : String("null")) +
            ",\"lastRx\":" + jsonString(lastRadioRx) +
            ",\"lastTx\":" + jsonString(lastRadioTx) + "}";
    if (statusSeen) {
      json += ",\"status\":{\"throttle\":" + String(rxThrottle) +
              ",\"roll\":" + String(rxRoll, 1) +
              ",\"pitch\":" + String(rxPitch, 1) +
              ",\"yaw\":" + String(rxYaw, 1) + ",\"motors\":[";
      for (int i = 0; i < 4; i++) {
        if (i) json += ',';
        json += String(rxMotors[i]);
      }
      json += "]}";
    }
    json += '}';
    web.send(200, "application/json", json);
  });
  /* POST /api/throttle: kiểm tra ga 1000..1700 microsecond và hẹn gửi T. */
  web.on("/api/throttle", HTTP_POST, []() {
    String s = web.arg("throttle"); /**< Chuỗi ga do HTTP client gửi trong tham số throttle. */
    if (!s.length()) {
      respondError("Thiếu ga");
      return;
    }
    char *end = nullptr; /**< Con trỏ tới ký tự đầu tiên chưa được hàm chuyển số đọc. */
    long value = strtol(s.c_str(), &end, 10); /**< Ga đọc ở cơ số 10, chỉ được chấp nhận trong 1000..1700 microsecond. */
    if (end == s.c_str() || *end != '\0' || value < MIN_THR || value > MAX_THR) {
      respondError("Ga phải từ 1000 đến 1700 µs");
      return;
    }
    throttle = (uint16_t)value;
    lastWebMs = millis();
    lastThrottleTxMs = millis() - THROTTLE_PERIOD_MS; // send next radio slot
    respondOk();
  });
  /* POST /api/pids: kiểm tra 15 gain rồi khởi động tác vụ ghi tuần tự năm PID. */
  web.on("/api/pids", HTTP_POST, []() {
    if (operation != OP_NONE) {
      respondError("Đang có tác vụ PID khác", 409);
      return;
    }
    for (int i = 0; i < 5; i++) {
      if (!parseGain(web.arg(String(i) + "_kp"), pendingPids[i].kp) ||
          !parseGain(web.arg(String(i) + "_ki"), pendingPids[i].ki) ||
          !parseGain(web.arg(String(i) + "_kd"), pendingPids[i].kd)) {
        respondError("PID phải là số hữu hạn từ 0 đến 100");
        return;
      }
    }
    startOperation(OP_APPLY);
    respondOk();
  });
  /* POST /api/readpid: khởi động tác vụ đọc tuần tự năm PID khi đang rảnh. */
  web.on("/api/readpid", HTTP_POST, []() {
    if (operation != OP_NONE) {
      respondError("Đang có tác vụ PID khác", 409);
      return;
    }
    startOperation(OP_READ);
    respondOk();
  });
  web.onNotFound([]() { web.send(404, "text/plain", "Not found"); });
}

/**
 * @brief Điểm khởi tạo Arduino: UART E220, chân mode, Wi-Fi station và HTTP server.
 * @note Chờ Wi-Fi tối đa khoảng 15 giây rồi tiếp tục; loop() tiếp tục yêu cầu reconnect.
 */
void setup() {
  Serial.begin(115200);
  pinMode(E220_M0, OUTPUT);
  pinMode(E220_M1, OUTPUT);
  if (E220_AUX >= 0)
    pinMode(E220_AUX, INPUT_PULLUP);
  digitalWrite(E220_M0, LOW);
  digitalWrite(E220_M1, LOW);  // Normal / transparent mode
  LoRa.setRxBufferSize(512);
  LoRa.setTxBufferSize(256);
  LoRa.begin(UART_BAUD, SERIAL_8N1, E220_RX, E220_TX);
  radioStarted = static_cast<bool>(LoRa);
  if (!radioStarted) {
    ++radioTxErrors;
    Serial.println("E220 UART initialization failed");
  }
  Serial.printf("E220: RX=%d TX=%d M0=%d M1=%d AUX=%d, %lu baud 8N1\n",
                E220_RX, E220_TX, E220_M0, E220_M1, E220_AUX,
                static_cast<unsigned long>(UART_BAUD));

  // Station mode: ket noi router Wi-Fi 2.4 GHz cua ban
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  // Web server tu dong phuc vu khi ESP32 ket noi Wi-Fi thanh cong
  setupWebRoutes();
  web.begin();
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(WIFI_SSID);
  lastTxMs = millis() - MIN_RADIO_GAP_MS;
  lastThrottleTxMs = millis() - THROTTLE_PERIOD_MS;
  lastWebMs = millis();

  Serial.println("Dang ket noi WiFi...");

  unsigned long start = millis(); /**< Mốc bắt đầu chờ kết nối Wi-Fi trong setup(), ms. */

  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection FAILED!");
    Serial.print("WiFi status: ");
    Serial.println(WiFi.status());
  }
}

/**
 * @brief Theo dõi kết nối Wi-Fi, đặt ga tối thiểu khi mất kết nối và thử reconnect theo chu kỳ.
 */
void maintainWiFi() {
  static bool wasConnected = false; /**< Trạng thái Wi-Fi của lần gọi trước, dùng phát hiện chuyển trạng thái. */
  static uint32_t lastRetry = 0; /**< Mốc yêu cầu Wi-Fi reconnect gần nhất, ms. */
  bool connected = (WiFi.status() == WL_CONNECTED); /**< Trạng thái kết nối Wi-Fi hiện tại. */
  if (connected && !wasConnected) {
    Serial.println("Wi-Fi connected");
    Serial.print("Dashboard: http://");
    Serial.println(WiFi.localIP());
  } else if (!connected && wasConnected) {
    setMinimumThrottle("Wi-Fi lost");
  }
  wasConnected = connected;
  uint32_t now = millis(); /**< Mốc thời gian hiện tại dùng tính các khoảng thời gian, ms. */
  if (!connected && (uint32_t)(now - lastRetry) >= WIFI_RETRY_MS) {
    lastRetry = now;
    Serial.println("Retrying Wi-Fi...");
    WiFi.reconnect();
  }
}

/**
 * @brief Vòng lặp Arduino xử lý Wi-Fi, HTTP, radio RX/TX và log chẩn đoán mỗi giây.
 */
void loop() {
  maintainWiFi();
  web.handleClient();
  pollRadio();
  processRadioTx();

  static uint32_t lastDebug = 0; /**< Mốc xuất log debug gần nhất, ms. */
  if (millis() - lastDebug >= 1000) {
    lastDebug = millis();
    Serial.printf(
      "E220 M0=%d M1=%d AUX=%d TX=%lu RX_FRAME=%lu RX_BYTE=%lu RX_ERR=%lu TX_ERR=%lu THR=%u STATUS_AGE=%lu S_VALID=%lu S_BAD=%lu OP=%d WAIT=%d\n",
      digitalRead(E220_M0), digitalRead(E220_M1),
      E220_AUX >= 0 ? digitalRead(E220_AUX) : -1,
      (unsigned long)radioTxCount,
      (unsigned long)radioRxCount,
      (unsigned long)radioRxByteCount,
      (unsigned long)radioRxErrors,
      (unsigned long)radioTxErrors,
      (unsigned)throttle,
      (unsigned long)(statusSeen ? millis() - lastStatusMs : 0),
      (unsigned long)validStatusCount,
      (unsigned long)invalidStatusCount,
      (int)operation, (int)waiting);
  }
  delay(1);
}
