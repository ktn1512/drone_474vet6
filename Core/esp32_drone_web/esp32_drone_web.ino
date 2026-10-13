#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include "index_html.h"

// Arduino-ESP32: ESP32 DevKit / ESP32-S3, E220-900T22D in transparent mode.
// Keep the module's stored settings. Address, channel, air rate and key must
// match the STM32-side E220; UART must be 9600 8N1, appended RSSI disabled.
constexpr char WIFI_SSID[] = "DESKTOP-7F2FABB 9674";
constexpr char WIFI_PASS[] = "khanhkhanh";
constexpr uint32_t WIFI_RETRY_MS = 10000;
#if defined(CONFIG_IDF_TARGET_ESP32S3)
constexpr int E220_RX = 18;  // ESP32-S3 RX <- E220 TXD
constexpr int E220_TX = 17;  // ESP32-S3 TX -> E220 RXD
constexpr int E220_M0 = 4;
constexpr int E220_M1 = 5;
constexpr int E220_AUX = 6;  // Set to -1 if AUX is not connected.
#elif defined(CONFIG_IDF_TARGET_ESP32)
constexpr int E220_RX = 16;  // ESP32 RX2 <- E220 TXD
constexpr int E220_TX = 17;  // ESP32 TX2 -> E220 RXD
constexpr int E220_M0 = 25;  // Same wiring as successful ESP32 E220 test
constexpr int E220_M1 = 26;
constexpr int E220_AUX = 27;  // Set to -1 if AUX is not connected.
#else
#error "Select ESP32 Dev Module or ESP32S3 Dev Module."
#endif
constexpr uint32_t UART_BAUD = 9600;
constexpr uint16_t MIN_THR = 1000;
constexpr uint16_t MAX_THR = 1700;
constexpr uint16_t MAX_MOTOR_STATUS_US = 2000;
/* Half-duplex E220 2.4kbps: nho de packet R/S co cua so phan hoi. */
constexpr uint32_t THROTTLE_PERIOD_MS = 1500;
constexpr uint32_t PID_HEARTBEAT_MS = 2500;
constexpr uint32_t CLIENT_TIMEOUT_MS = 3000;
constexpr uint32_t STATUS_VALID_MS = 6500;
constexpr uint32_t MIN_RADIO_GAP_MS = 900;
constexpr uint32_t AUX_SETTLE_MS = 3;
constexpr uint32_t RX_PARTIAL_TIMEOUT_MS = 1200;
constexpr size_t MAX_RADIO_PACKET_BYTES = 200;
constexpr uint32_t ACK_TIMEOUT_MS = 2200;
constexpr uint8_t MAX_ATTEMPTS = 4;

WebServer web(80);
HardwareSerial LoRa(2);

struct PidGains {
  float kp, ki, kd;
};
PidGains pids[5] = {
  { 10.0f, 0.0f, 0.0f },  // Angle Roll
  { 10.0f, 0.0f, 0.0f },  // Angle Pitch
  { 0.5f, 1.4f, 0.0f },   // Rate Roll
  { 0.5f, 1.4f, 0.0f },   // Rate Pitch
  { 0.3f, 0.0f, 0.0f }    // Rate Yaw
};

/* Throttle truc tiep, khong co ARM/DISARM. */
uint16_t throttle = MIN_THR;
uint32_t lastWebMs = 0;
uint32_t lastTxMs = 0;
uint32_t lastThrottleTxMs = 0;
uint32_t nextSeq = 1;

bool statusSeen = false;
uint32_t lastStatusMs = 0;
uint32_t validStatusCount = 0;
uint32_t invalidStatusCount = 0;
String statusDiagnostic = "Chua nhan goi status S hop le";
uint16_t rxThrottle = MIN_THR;
float rxRoll = 0, rxPitch = 0, rxYaw = 0;
uint16_t rxMotors[4] = {1000, 1000, 1000, 1000};

enum OpType { OP_NONE, OP_APPLY, OP_READ };
OpType operation = OP_NONE;
PidGains pendingPids[5];
uint8_t opIndex = 0;
uint8_t attempts = 0;
uint32_t waitingSeq = 0;
uint32_t lastOpMs = 0;
bool waiting = false;
String opText = "Không có yêu cầu PID";

char rxLine[176];
size_t rxLength = 0;
bool rxDiscard = false;
uint32_t lastRxByteMs = 0;
bool radioStarted = false;
bool auxWasHigh = false;
uint32_t auxHighSinceMs = 0;
uint32_t radioTxCount = 0;
uint32_t radioRxCount = 0;
uint32_t radioRxByteCount = 0;
uint32_t radioRxErrors = 0;
uint32_t radioTxErrors = 0;
uint32_t lastRadioRxMs = 0;
String lastRadioRx;
String lastRadioTx;

String jsonString(const String &value) {
  String escaped;
  escaped.reserve(value.length() + 2);
  escaped += '"';
  for (size_t i = 0; i < value.length(); ++i) {
    const uint8_t c = static_cast<uint8_t>(value[i]);
    if (c == '"' || c == '\\') {
      escaped += '\\';
      escaped += static_cast<char>(c);
    } else if (c >= 0x80) {
      // Preserve valid UTF-8 text; escape isolated bytes from binary/noisy RX.
      const size_t count = c >= 0xC2 && c <= 0xDF ? 2 : c >= 0xE0 && c <= 0xEF ? 3
                                                      : c >= 0xF0 && c <= 0xF4 ? 4
                                                                               : 0;
      bool valid = count != 0 && i + count <= value.length();
      for (size_t j = 1; valid && j < count; ++j) {
        const uint8_t next = static_cast<uint8_t>(value[i + j]);
        valid = next >= 0x80 && next <= 0xBF;
        if (j == 1)
          valid = valid && !(c == 0xE0 && next < 0xA0) && !(c == 0xED && next > 0x9F) && !(c == 0xF0 && next < 0x90) && !(c == 0xF4 && next > 0x8F);
      }
      if (valid) {
        for (size_t j = 0; j < count; ++j)
          escaped += value[i + j];
        i += count - 1;
      } else {
        char hex[7];
        snprintf(hex, sizeof(hex), "\\u%04X", static_cast<unsigned>(c));
        escaped += hex;
      }
    } else if (c < 0x20 || c == 0x7F) {
      // Radio input may contain arbitrary bytes; always emit valid JSON.
      char hex[7];
      snprintf(hex, sizeof(hex), "\\u%04X", static_cast<unsigned>(c));
      escaped += hex;
    } else {
      escaped += static_cast<char>(c);
    }
  }
  escaped += '"';
  return escaped;
}

uint8_t xorChecksum(const String &s) {
  uint8_t v = 0;
  for (size_t i = 0; i < s.length(); i++)
    v ^= uint8_t(s[i]);
  return v;
}

String frameOf(const String &payload) {
  char hex[5];
  snprintf(hex, sizeof(hex), "*%02X\n", xorChecksum(payload));
  return payload + hex;
}

bool decodeFrame(String &raw) {
  if (raw.length() < 4 || raw[raw.length() - 3] != '*')
    return false;
  String hex = raw.substring(raw.length() - 2);
  char *end = nullptr;
  unsigned long want = strtoul(hex.c_str(), &end, 16);
  if (end != hex.c_str() + 2 || want > 255)
    return false;
  raw.remove(raw.length() - 3);
  return xorChecksum(raw) == want;
}

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

bool radioReady(uint32_t now) {
  return radioStarted && radioAuxReady(now) && (uint32_t)(now - lastTxMs) >= MIN_RADIO_GAP_MS;
}

bool sendRadioBytes(const String &packet, const String &description) {
  uint32_t now = millis();
  if (!radioReady(now))
    return false;
  if (packet.length() == 0 || packet.length() > MAX_RADIO_PACKET_BYTES) {
    ++radioTxErrors;
    return false;
  }
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

bool sendRadio(const String &payload) {
  return sendRadioBytes(frameOf(payload), payload);
}

void setMinimumThrottle(const char *reason) {
  if (throttle != MIN_THR) {
    throttle = MIN_THR;
    lastThrottleTxMs = millis() - THROTTLE_PERIOD_MS;
    Serial.print("Throttle -> 1000: ");
    Serial.println(reason);
  }
}

bool freshStatus() {
  return statusSeen && (uint32_t)(millis() - lastStatusMs) <= STATUS_VALID_MS;
}

bool parseGain(const String &text, float &v) {
  if (!text.length())
    return false;
  char *end = nullptr;
  v = strtof(text.c_str(), &end);
  return end != text.c_str() && *end == 0 && isfinite(v) && v >= 0 && v <= 100;
}

void respondOk() {
  web.send(200, "application/json", "{\"ok\":true}");
}
void respondError(const String &reason, int status = 400) {
  web.send(status, "application/json", "{\"ok\":false,\"error\":" + jsonString(reason) + "}");
}

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

  const char kind = frame[0];
  unsigned long seq = 0;
  unsigned id = 0;
  int pos = 0;

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
    float kp = 0, ki = 0, kd = 0;
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
    // S,seq,throttle,roll,pitch,yaw,m1,m2,m3,m4 (one response to T).
    unsigned thr = 0, m1 = 0, m2 = 0, m3 = 0, m4 = 0;
    float r = 0, p = 0, y = 0;
    pos = 0;
    if (sscanf(frame.c_str(), "S,%lu,%u,%f,%f,%f,%u,%u,%u,%u%n",
               &seq, &thr, &r, &p, &y, &m1, &m2, &m3, &m4, &pos) == 9 &&
        pos == (int)frame.length() &&
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
    } else {
      ++invalidStatusCount;
      statusDiagnostic = "Status S sai số trường hoặc ngoài giới hạn";
      Serial.print("S invalid: ");
      Serial.println(frame);
    }
  }
}

void pollRadio() {
  uint32_t now = millis();
  radioAuxReady(now);
  while (LoRa.available() > 0) {
    int data = LoRa.read();

    if (data < 0)
      break;

    ++radioRxByteCount;
    char c = (char)data;
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

void startOperation(OpType type) {
  operation = type;
  opIndex = 0;
  attempts = 0;
  waiting = false;
  opText = type == OP_READ ? "Đang đọc PID: 0/5" : "Đang gửi PID: 0/5";
}

void processRadioTx() {
  uint32_t now = millis();
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
    String payload;
    if (operation == OP_APPLY) {
      const PidGains &p = pendingPids[opIndex];
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

void setupWebRoutes() {
  web.on("/", HTTP_GET, []() {
    web.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
  });
  web.on("/api/state", HTTP_GET, []() {
    lastWebMs = millis();
    String json;
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
    const uint32_t now = millis();
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
  web.on("/api/throttle", HTTP_POST, []() {
    String s = web.arg("throttle");
    if (!s.length()) {
      respondError("Thiếu ga");
      return;
    }
    char *end = nullptr;
    long value = strtol(s.c_str(), &end, 10);
    if (end == s.c_str() || *end != '\0' || value < MIN_THR || value > MAX_THR) {
      respondError("Ga phải từ 1000 đến 1700 µs");
      return;
    }
    throttle = (uint16_t)value;
    lastWebMs = millis();
    lastThrottleTxMs = millis() - THROTTLE_PERIOD_MS; // send next radio slot
    respondOk();
  });
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

  unsigned long start = millis();

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

void maintainWiFi() {
  static bool wasConnected = false;
  static uint32_t lastRetry = 0;
  bool connected = (WiFi.status() == WL_CONNECTED);
  if (connected && !wasConnected) {
    Serial.println("Wi-Fi connected");
    Serial.print("Dashboard: http://");
    Serial.println(WiFi.localIP());
  } else if (!connected && wasConnected) {
    setMinimumThrottle("Wi-Fi lost");
  }
  wasConnected = connected;
  uint32_t now = millis();
  if (!connected && (uint32_t)(now - lastRetry) >= WIFI_RETRY_MS) {
    lastRetry = now;
    Serial.println("Retrying Wi-Fi...");
    WiFi.reconnect();
  }
}

void loop() {
  maintainWiFi();
  web.handleClient();
  pollRadio();
  processRadioTx();

  static uint32_t lastDebug = 0;
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
