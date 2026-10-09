#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>

// One sketch for ESP32 DevKit (WROOM-32) and ESP32-S3.
// GPIO numbers below are ESP32 pins, not E220 pin numbers.
#if defined(CONFIG_IDF_TARGET_ESP32S3)
constexpr int LORA_RX = 18;  // ESP32-S3 RX <- E220 TXD
constexpr int LORA_TX = 17;  // ESP32-S3 TX -> E220 RXD
constexpr int LORA_M0 = 4;
constexpr int LORA_M1 = 5;
constexpr int LORA_AUX = 6;
constexpr const char *NODE_NAME = "ESP32-S3";
#elif defined(CONFIG_IDF_TARGET_ESP32)
constexpr int LORA_RX = 16;  // ESP32 DevKit RX <- E220 TXD
constexpr int LORA_TX = 17;  // ESP32 DevKit TX -> E220 RXD
constexpr int LORA_M0 = 26;
constexpr int LORA_M1 = 27;
constexpr int LORA_AUX = 25;
constexpr const char *NODE_NAME = "ESP32-DevKit";
#else
#error "Select ESP32 Dev Module or ESP32S3 Dev Module in Arduino IDE."
#endif

// Both E220 modules must use the SAME address, channel and air rate.
constexpr uint16_t RADIO_ADDRESS = 0x1234;
constexpr uint8_t RADIO_CHANNEL = 23;  // 850.125 + 23 = 873.125 MHz
static_assert(RADIO_CHANNEL <= 80, "E220-900T22D channel must be 0..80");
constexpr uint32_t RADIO_BAUD = 9600;
constexpr size_t MAX_TEXT_BYTES = 120;
constexpr uint32_t MIN_TX_GAP_MS = 1500;

HardwareSerial Radio(1);
bool radioReady = false;
bool hasSent = false;
uint32_t lastTxMs = 0;

// Wait for AUX to remain HIGH for at least 3 ms.
// AUX HIGH means buffer ready; it is NOT a delivery acknowledgment.
bool waitAuxReady(uint32_t timeoutMs = 2000) {
  const uint32_t start = millis();
  uint32_t highSince = start;
  while (static_cast<uint32_t>(millis() - start) < timeoutMs) {
    const uint32_t now = millis();
    if (digitalRead(LORA_AUX) == LOW) {
      highSince = now;
    } else if (static_cast<uint32_t>(now - highSince) >= 3) {
      return true;
    }
    delay(1);
  }
  return false;
}

bool configureRadio() {
  // Configuration mode: M0 = HIGH, M1 = HIGH; UART is always 9600 8N1.
  digitalWrite(LORA_M0, HIGH);
  digitalWrite(LORA_M1, HIGH);
  delay(50);
  if (!waitAuxReady()) {
    Serial.println("ERROR: AUX timeout. Check power and AUX wiring.");
    return false;
  }

  while (Radio.available() > 0) {
    Radio.read();
  }

  // C2: temporary configuration, reapplied on every ESP32 boot.
  // Registers: ADDH, ADDL, REG0, REG1, CHANNEL, REG3, CRYPT_H, CRYPT_L.
  const uint8_t command[] = {
    0xC2, 0x00, 0x08,
    static_cast<uint8_t>(RADIO_ADDRESS >> 8),
    static_cast<uint8_t>(RADIO_ADDRESS & 0xFF),
    0x62,  // UART 9600, 8N1; air data rate 2.4 kbps
    0x00,  // 200-byte subpackets; 22 dBm; ambient RSSI disabled
    RADIO_CHANNEL,
    0x00,  // Transparent transmission; RSSI byte and LBT disabled
    0x00, 0x00  // Communication key = 0
  };

  Radio.write(command, sizeof(command));
  Radio.flush();  // Wait for UART output, not for wireless delivery.
  uint8_t response[sizeof(command)] = {};
  const size_t received = Radio.readBytes(response, sizeof(response));
  if (received != sizeof(response) || response[0] != 0xC1 ||
      memcmp(response + 1, command + 1, sizeof(command) - 1) != 0) {
    Serial.println("ERROR: E220 config response invalid. Check RX/TX, M0/M1.");
    return false;
  }

  if (!waitAuxReady()) {
    Serial.println("ERROR: AUX timeout after configuration.");
    return false;
  }

  // Normal mode: both nodes can transmit and receive.
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  delay(50);
  return waitAuxReady();
}

// Text protocol: UTF-8 bytes followed by '\n', up to 120 bytes per line.
bool sendText(const char *text) {
  const size_t length = strlen(text);
  if (!radioReady || length == 0 || length > MAX_TEXT_BYTES) {
    return false;
  }
  if (hasSent && static_cast<uint32_t>(millis() - lastTxMs) < MIN_TX_GAP_MS) {
    Serial.println("BUSY: wait 1.5 seconds between messages, then resend.");
    return false;
  }
  if (!waitAuxReady()) {
    Serial.println("ERROR: E220 busy. Message was not sent.");
    return false;
  }

  // Submit one complete line to the module.
  uint8_t packet[MAX_TEXT_BYTES + 1];
  memcpy(packet, text, length);
  packet[length] = '\n';
  if (Radio.write(packet, length + 1) != length + 1) {
    Serial.println("ERROR: UART write failed.");
    return false;
  }
  Radio.flush();
  lastTxMs = millis();
  hasSent = true;
  Serial.printf("[TX %s] %s\n", NODE_NAME, text);
  return true;
}

void receiveRadio() {
  static char line[MAX_TEXT_BYTES + 1];
  static size_t used = 0;
  static bool overflow = false;

  while (Radio.available() > 0) {
    const char c = static_cast<char>(Radio.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      if (overflow) {
        Serial.println("ERROR: received line exceeds 120 bytes; discarded.");
      } else if (used > 0) {
        line[used] = '\0';
        Serial.printf("[RX %s] %s\n", NODE_NAME, line);
      }
      used = 0;
      overflow = false;
    } else if (used < MAX_TEXT_BYTES && !overflow) {
      line[used++] = c;
    } else {
      overflow = true;  // Discard the remainder until the next newline.
    }
  }
}

void readSerialMonitor() {
  static char line[MAX_TEXT_BYTES + 1];
  static size_t used = 0;
  static bool overflow = false;

  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      if (overflow) {
        Serial.println("ERROR: input exceeds 120 bytes; discarded.");
      } else if (used > 0) {
        line[used] = '\0';
        sendText(line);
      }
      used = 0;
      overflow = false;
    } else if (used < MAX_TEXT_BYTES && !overflow) {
      line[used++] = c;
    } else {
      overflow = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  const uint32_t start = millis();
  while (!Serial && static_cast<uint32_t>(millis() - start) < 1500) {
    delay(10);  // Bounded wait for native USB on ESP32-S3.
  }

  // Configure GPIO first; Arduino-ESP32 ignores writes to unconfigured pins.
  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  digitalWrite(LORA_M0, HIGH);
  digitalWrite(LORA_M1, HIGH);
  pinMode(LORA_AUX, INPUT_PULLUP);

  Radio.setRxBufferSize(512);  // Call before begin().
  Radio.setTimeout(1000);
  Radio.begin(RADIO_BAUD, SERIAL_8N1, LORA_RX, LORA_TX);
  Serial.printf("\n%s + E220-900T22D\n", NODE_NAME);

  radioReady = configureRadio();
  if (!radioReady) {
    Serial.println("Radio stopped. Check wiring, then press RESET.");
    return;
  }

  Serial.printf("READY: address=0x%04X, channel=%u, frequency=%.3f MHz\n",
                static_cast<unsigned int>(RADIO_ADDRESS),
                static_cast<unsigned int>(RADIO_CHANNEL),
                850.125 + RADIO_CHANNEL);
  Serial.println("Serial Monitor: 115200 baud, Newline or Both NL & CR.");
  Serial.println("Type a message and press Send. Either board can send.");
}

void loop() {
  if (radioReady) {
    receiveRadio();
    readSerialMonitor();
  }
  delay(1);
}
