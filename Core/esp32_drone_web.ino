#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include "index_html.h"

// Target: ESP32 DevKit V1 (classic ESP32), Arduino-ESP32 board package.
// E220 must be configured with the same channel, address, UART 9600 8N1 and air rate as the STM32-side E220.
constexpr char WIFI_SSID[] = "Mi 9T Pro";
constexpr char WIFI_PASS[] = "khanhkhanh";
constexpr uint32_t WIFI_RETRY_MS = 10000;
constexpr int E220_RX = 16; // ESP32 RX2 <- E220 TXD
constexpr int E220_TX = 17; // ESP32 TX2 -> E220 RXD
constexpr int E220_M0 = 26;
constexpr int E220_M1 = 27;
constexpr int E220_AUX = 25;
constexpr uint32_t UART_BAUD = 9600;
constexpr uint16_t MIN_THR = 1000;
constexpr uint16_t MAX_THR = 1700;
constexpr uint32_t THROTTLE_PERIOD_MS = 250;
constexpr uint32_t CLIENT_TIMEOUT_MS = 2500;
constexpr uint32_t STATUS_VALID_MS = 4500;
constexpr uint32_t MIN_RADIO_GAP_MS = 230;
constexpr uint32_t ACK_TIMEOUT_MS = 1600;
constexpr uint8_t MAX_ATTEMPTS = 3;

WebServer web(80);
HardwareSerial LoRa(2);

struct PidGains
{
  float kp, ki, kd;
};
PidGains pids[5] = {
    {10.0f, 0.0f, 0.0f}, // Angle Roll
    {10.0f, 0.0f, 0.0f}, // Angle Pitch
    {0.5f, 1.4f, 0.0f},  // Rate Roll
    {0.5f, 1.4f, 0.0f},  // Rate Pitch
    {0.3f, 0.0f, 0.0f}   // Rate Yaw
};

bool armed = false;
uint16_t throttle = MIN_THR;
uint32_t lastWebMs = 0;
uint32_t lastTxMs = 0;
uint32_t lastThrottleTxMs = 0;
uint32_t nextSeq = 1;

bool statusSeen = false;
uint32_t lastStatusMs = 0;
uint16_t rxThrottle = 1000;
bool rxArmed = false;
float rxRoll = 0, rxPitch = 0, rxYaw = 0;
uint16_t rxMotors[4] = {1000, 1000, 1000, 1000};

// Only one operation at a time; always disarmed during remote PID tuning.
enum OpType
{
  OP_NONE,
  OP_APPLY,
  OP_READ
};
OpType operation = OP_NONE;
uint8_t opIndex = 0;
uint8_t attempts = 0;
uint32_t waitingSeq = 0;
uint32_t lastOpMs = 0;
bool waiting = false;
String opText = "Không có yêu cầu PID";

char rxLine[176];
size_t rxLength = 0;

uint8_t xorChecksum(const String &s)
{
  uint8_t v = 0;
  for (size_t i = 0; i < s.length(); i++)
    v ^= uint8_t(s[i]);
  return v;
}

String frameOf(const String &payload)
{
  char hex[5];
  snprintf(hex, sizeof(hex), "*%02X\n", xorChecksum(payload));
  return payload + hex;
}

bool decodeFrame(String &raw)
{
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

bool radioReady(uint32_t now)
{
  return digitalRead(E220_AUX) == HIGH && (uint32_t)(now - lastTxMs) >= MIN_RADIO_GAP_MS;
}

bool sendRadio(const String &payload)
{
  uint32_t now = millis();
  if (!radioReady(now))
    return false;
  LoRa.print(frameOf(payload));
  lastTxMs = now;
  Serial.print("TX > ");
  Serial.println(payload);
  return true;
}

void stopMotors()
{
  armed = false;
  throttle = MIN_THR;
}

bool freshStatus()
{
  return statusSeen && (uint32_t)(millis() - lastStatusMs) <= STATUS_VALID_MS;
}

bool parseGain(const String &text, float &v)
{
  if (!text.length())
    return false;
  char *end = nullptr;
  v = strtof(text.c_str(), &end);
  return end != text.c_str() && *end == 0 && isfinite(v) && v >= 0 && v <= 100;
}

void respondOk()
{
  web.send(200, "application/json", "{\"ok\":true}");
}
void respondError(const String &reason, int status = 400)
{
  web.send(status, "application/json", "{\"ok\":false,\"error\":\"" + reason + "\"}");
}

void onFrame(String frame)
{
  if (!decodeFrame(frame))
    return;
  Serial.print("RX < ");
  Serial.println(frame);
  unsigned long seq = 0;
  unsigned id = 0;
  char kind = frame[0];

  if ((kind == 'A' || kind == 'N') && sscanf(frame.c_str(), "%*c,%lu,%u", &seq, &id) == 2 && operation == OP_APPLY && waiting && seq == waitingSeq && id == opIndex)
  {
    waiting = false;
    if (kind == 'N')
    {
      operation = OP_NONE;
      opText = "Drone từ chối thay đổi PID";
    }
    else
    {
      opIndex++;
      attempts = 0;
      if (opIndex == 5)
      {
        operation = OP_NONE;
        opText = "Đã gửi PID: 5/5 ACK";
      }
    }
  }
  else if (kind == 'R')
  {
    float kp, ki, kd;
    if (sscanf(frame.c_str(), "R,%lu,%u,%f,%f,%f", &seq, &id, &kp, &ki, &kd) == 5 && operation == OP_READ && waiting && seq == waitingSeq && id == opIndex && isfinite(kp) && isfinite(ki) && isfinite(kd) && kp >= 0 && kp <= 100 && ki >= 0 && ki <= 100 && kd >= 0 && kd <= 100)
    {
      pids[id] = {kp, ki, kd};
      waiting = false;
      opIndex++;
      attempts = 0;
      if (opIndex == 5)
      {
        operation = OP_NONE;
        opText = "Đã đọc PID từ drone: 5/5";
      }
    }
  }
  else if (kind == 'S')
  {
    unsigned thr, arm, m1, m2, m3, m4;
    float r, p, y;
    if (sscanf(frame.c_str(), "S,%u,%u,%f,%f,%f,%u,%u,%u,%u",
               &thr, &arm, &r, &p, &y, &m1, &m2, &m3, &m4) == 9 &&
        thr >= MIN_THR && thr <= MAX_THR && arm <= 1 && isfinite(r) && isfinite(p) && isfinite(y) && m1 >= MIN_THR && m1 <= MAX_THR && m2 >= MIN_THR && m2 <= MAX_THR && m3 >= MIN_THR && m3 <= MAX_THR && m4 >= MIN_THR && m4 <= MAX_THR)
    {
      rxThrottle = thr;
      rxArmed = arm;
      rxRoll = r;
      rxPitch = p;
      rxYaw = y;
      rxMotors[0] = m1;
      rxMotors[1] = m2;
      rxMotors[2] = m3;
      rxMotors[3] = m4;
      lastStatusMs = millis();
      statusSeen = true;
    }
  }
}

void pollRadio()
{
  while (LoRa.available() > 0)
  {
    int data = LoRa.read();

    if (data < 0)
      break;

    char c = (char)data;

    // Xem tung byte nhan duoc
    Serial.printf("RX BYTE: 0x%02X\n",
                  (unsigned)data);

    if (c == '\n')
    {
      rxLine[rxLength] = '\0';

      Serial.print("RAW RX: [");
      Serial.print(rxLine);
      Serial.println("]");

      if (rxLength > 0)
      {
        onFrame(String(rxLine));
      }

      rxLength = 0;
    }
    else if (c != '\r')
    {
      if (rxLength < sizeof(rxLine) - 1)
      {
        rxLine[rxLength++] = c;
      }
      else
      {
        rxLength = 0;
      }
    }
  }
}

void startOperation(OpType type)
{
  operation = type;
  opIndex = 0;
  attempts = 0;
  waiting = false;
  opText = type == OP_READ ? "Đang đọc PID: 0/5" : "Đang gửi PID: 0/5";
}

void processRadioTx()
{
  uint32_t now = millis();
  if (armed && (uint32_t)(now - lastWebMs) > CLIENT_TIMEOUT_MS)
  {
    stopMotors();
    Serial.println("Client timeout -> DISARM");
  }
  // Loss of telemetry triggers safer disarm, not continued throttle.
  if (armed && !freshStatus())
  {
    stopMotors();
    Serial.println("Drone status timeout -> DISARM");
  }

  if (operation != OP_NONE)
  {
    if (waiting)
    {
      if ((uint32_t)(now - lastOpMs) < ACK_TIMEOUT_MS)
        return;
      waiting = false;
      if (attempts >= MAX_ATTEMPTS)
      {
        operation = OP_NONE;
        opText = "Lỗi: PID không có phản hồi sau 3 lần thử";
        return;
      }
    }
    if (!radioReady(now))
      return;
    String payload;
    if (operation == OP_APPLY)
    {
      const PidGains &p = pids[opIndex];
      payload = "P," + String(nextSeq) + "," + String(opIndex) + "," + String(p.kp, 4) + "," + String(p.ki, 4) + "," + String(p.kd, 4);
    }
    else
    {
      payload = "G," + String(nextSeq) + "," + String(opIndex);
    }
    if (sendRadio(payload))
    {
      waitingSeq = nextSeq++;
      waiting = true;
      attempts++;
      lastOpMs = now;
      opText = (operation == OP_APPLY ? "Đang gửi PID: " : "Đang đọc PID: ") + String(opIndex) + "/5";
    }
    return;
  }
  if ((uint32_t)(now - lastThrottleTxMs) >= THROTTLE_PERIOD_MS)
  {
    String payload = "T," + String(nextSeq++) + "," + String(armed ? throttle : MIN_THR) + "," + String(armed ? 1 : 0);
    if (sendRadio(payload))
      lastThrottleTxMs = now;
  }
}

void setupWebRoutes()
{
  web.on("/", HTTP_GET, []()
         { web.send_P(200, "text/html; charset=utf-8", INDEX_HTML); });
  web.on("/api/state", HTTP_GET, []()
         {
    lastWebMs = millis();
    String json;
    json.reserve(900);
    json = "{\"armed\":" + String(armed ? "true" : "false") + ",\"throttle\":" + String(throttle) + ",\"link\":" + String(freshStatus() ? "true" : "false") + ",\"operation\":\"" + opText + "\",\"pids\":[";
    for (int i = 0; i < 5; i++) {
      if (i) json += ',';
      json += "{\"kp\":" + String(pids[i].kp, 4) + ",\"ki\":" + String(pids[i].ki, 4) + ",\"kd\":" + String(pids[i].kd, 4) + "}";
    }
    json += "]";
    if (statusSeen) {
      json += ",\"status\":{\"throttle\":" + String(rxThrottle) + ",\"armed\":" + String(rxArmed ? "true" : "false") + ",\"roll\":" + String(rxRoll, 1) + ",\"pitch\":" + String(rxPitch, 1) + ",\"yaw\":" + String(rxYaw, 1) + ",\"motors\":[";
      for (int i = 0; i < 4; i++) {
        if (i) json += ',';
        json += String(rxMotors[i]);
      }
      json += "]}";
    }
    json += '}';
    web.send(200, "application/json", json); });
  web.on("/api/arm", HTTP_POST, []()
         {
    if (web.arg("arm") == "0") {
      stopMotors();
      respondOk();
      return;
    }
    if (web.arg("arm") != "1") {
      respondError("Gia tri ARM khong hop le");
      return;
    }
    if (armed) {
      respondOk();
      return;
    }
    if (operation != OP_NONE || throttle != MIN_THR || !freshStatus() || rxArmed) {
      respondError("Chi ARM tai 1000us khi drone dang DISARM va co status moi");
      return;
    }
    lastWebMs = millis();
    armed = true;
    throttle = MIN_THR;
    // radio heartbeat will deliver ARM; no physical control implied by HTTP response.
    respondOk(); });
  web.on("/api/throttle", HTTP_POST, []()
         {
    String s = web.arg("throttle");
    if (!s.length()) {
      respondError("Thieu ga");
      return;
    }
    char *end = nullptr;
    long value = strtol(s.c_str(), &end, 10);
    if (end == s.c_str() || *end != 0 || value < MIN_THR || value > MAX_THR) {
      respondError("Ga phai tu 1000 den 1700");
      return;
    }
    if (!armed && value != MIN_THR) {
      respondError("Phai ARM tai 1000us truoc");
      return;
    }
    if (value > MIN_THR && (!freshStatus() || !rxArmed)) {
      respondError("Drone chua xac nhan ARM");
      return;
    }
    throttle = (uint16_t)value;
    lastWebMs = millis();
    respondOk(); });
  web.on("/api/pids", HTTP_POST, []()
         {
    if (armed || operation != OP_NONE) {
      respondError("DISARM va cho tac vu cu ket thuc");
      return;
    }
    PidGains temp[5];
    for (int i = 0; i < 5; i++) {
      if (!parseGain(web.arg(String(i) + "_kp"), temp[i].kp) || !parseGain(web.arg(String(i) + "_ki"), temp[i].ki) || !parseGain(web.arg(String(i) + "_kd"), temp[i].kd)) {
        respondError("PID phai la so huu han tu 0 den 100");
        return;
      }
    }
    memcpy(pids, temp, sizeof(pids));
    stopMotors();
    startOperation(OP_APPLY);
    respondOk(); });
  web.on("/api/readpid", HTTP_POST, []()
         {
    if (armed || operation != OP_NONE) {
      respondError("Chi doc PID khi DISARM va radio ranh");
      return;
    }
    startOperation(OP_READ);
    respondOk(); });
  web.onNotFound([]()
                 { web.send(404, "text/plain", "Not found"); });
}

void setup()
{
  Serial.begin(115200);
  pinMode(E220_M0, OUTPUT);
  pinMode(E220_M1, OUTPUT);
  pinMode(E220_AUX, INPUT_PULLUP);
  digitalWrite(E220_M0, LOW);
  digitalWrite(E220_M1, LOW); // Normal / transparent mode
  LoRa.begin(UART_BAUD, SERIAL_8N1, E220_RX, E220_TX);

  // Station mode: ket noi router Wi-Fi 2.4 GHz cua ban
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWiFi Connected!");

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    Serial.print("Web Server: http://");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("\nWiFi Connection Failed!");
  }
  // Web server tu dong phuc vu khi ESP32 ket noi Wi-Fi thanh cong
  setupWebRoutes();
  web.begin();
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(WIFI_SSID);
  lastTxMs = millis() - MIN_RADIO_GAP_MS;
  lastThrottleTxMs = millis() - THROTTLE_PERIOD_MS;
}

void maintainWiFi()
{
  static bool wasConnected = false;
  static uint32_t lastRetry = 0;

  bool connected = (WiFi.status() == WL_CONNECTED);
  if (connected && !wasConnected)
  {
    Serial.println("Wi-Fi connected");
    Serial.print("Dashboard: http://");
    Serial.println(WiFi.localIP());
  }
  else if (!connected && wasConnected)
  {
    stopMotors(); // Mat Wi-Fi: dua ga ve 1000 us, DISARM
    Serial.println("Wi-Fi lost -> DISARM");
  }
  wasConnected = connected;

  uint32_t now = millis();
  if (!connected && (uint32_t)(now - lastRetry) >= WIFI_RETRY_MS)
  {
    lastRetry = now;
    Serial.println("Retrying Wi-Fi...");
    WiFi.reconnect();
  }
}

void loop()
{
  maintainWiFi();
  web.handleClient();
  pollRadio();
  processRadioTx();
  delay(1);
}
