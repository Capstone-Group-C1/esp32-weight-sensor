#include <Arduino.h>
#include <SPI.h>
#include <HX711.h>
#include <mcp2515.h>
#include <cstring>

constexpr size_t kSensorCount = 4;
constexpr bool kReceiverOnline = false;

// HX711 wiring for each channel: {DOUT, SCK}
// Hardwired for 4 HX711 modules on one ESP32.
// DOUT pins use input-capable GPIOs; SCK pins use output-capable GPIOs.
constexpr uint8_t HX_PINS[kSensorCount][2] = {
  {34, 32},
  {35, 33},
  {36, 25},
  {39, 26},
};

// Per-sensor scale factors for HX711::set_scale().
// Replace these with your calibrated values.
float kScaleFactors[kSensorCount] = {
    4242.00f,
    4242.00f,
    4242.00f,
    4242.00f,
};

// MCP2515 CS pin on ESP32. SPI pins use board defaults.
constexpr uint8_t MCP2515_CS_PIN = 5;
constexpr uint8_t MCP2515_SCK_PIN = 18;
constexpr uint8_t MCP2515_MISO_PIN = 19;
constexpr uint8_t MCP2515_MOSI_PIN = 23;
constexpr uint8_t MCP2515_INT_PIN = 4;

// CAN message IDs used to publish each load cell.
// Matches Python driver receive filter: 0x100 - 0x1FF (0x100 + bin_id).
constexpr uint16_t kCanBaseId = 0x100;
constexpr uint8_t kBinId = 1;

// Status / tare flags expected by Python decoder.
constexpr uint8_t kStatusOk = 0x00;
constexpr uint8_t kTareSuccess = 0x01;

// Publish rate in milliseconds.
constexpr uint32_t kPublishPeriodMs = 1000;

HX711 scales[kSensorCount];
MCP2515 mcp2515(MCP2515_CS_PIN);

uint32_t lastPublishMs = 0;

void initializeHx711() {
  for (size_t i = 0; i < kSensorCount; ++i) {
    const uint8_t dout = HX_PINS[i][0];
    const uint8_t sck = HX_PINS[i][1];

    scales[i].begin(dout, sck);
    scales[i].set_scale(kScaleFactors[i]);
    scales[i].tare();
  }
}

bool initializeCan() {
  // Match the ESP-IDF wiring used in your previous project.
  SPI.begin(MCP2515_SCK_PIN, MCP2515_MISO_PIN, MCP2515_MOSI_PIN, MCP2515_CS_PIN);
  pinMode(MCP2515_INT_PIN, INPUT_PULLUP);

  mcp2515.reset();

  // Same timing as prior config (CNF1=0x00, CNF2=0x88, CNF3=0x03): 500 kbps @ 8 MHz.
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    return false;
  }

  // Prior driver enabled one-shot mode to tolerate a missing receiver ACK.
  if (mcp2515.setNormalOneShotMode() != MCP2515::ERROR_OK) {
    return false;
  }

  return true;
}

void sendWeightFrame(float rawWeight) {
  struct can_frame frame;
  frame.can_id = kCanBaseId + kBinId;
  frame.can_dlc = 8;

  // Layout expected by Python decoder:
  // Byte 0: bin_id
  // Bytes 1-4: float weight (little-endian)
  // Byte 5: status
  // Byte 6: tare flag
  // Byte 7: reserved
  frame.data[0] = kBinId;
  std::memcpy(&frame.data[1], &rawWeight, sizeof(float));
  frame.data[5] = kStatusOk;
  frame.data[6] = kTareSuccess;
  frame.data[7] = 0;

  const MCP2515::ERROR txResult = mcp2515.sendMessage(&frame);
  if (txResult == MCP2515::ERROR_OK) {
    return;
  }

  if (!kReceiverOnline) {
    Serial.printf("TX id=0x%03X no ACK expected yet, err=%d\n", frame.can_id, static_cast<int>(txResult));
    return;
  }

  Serial.printf("TX id=0x%03X failed, err=%d\n", frame.can_id, static_cast<int>(txResult));
}

void publishSensor1Weight() {
  float sensor1Weight = 0.0f;
  if (scales[0].is_ready()) {
    // Sensor 1 raw weight only for decoder integration.
    sensor1Weight = scales[0].get_units(1);
  }

  sendWeightFrame(sensor1Weight);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  initializeHx711();

  if (!initializeCan()) {
    Serial.println("CAN init failed. Check MCP2515 wiring/clock/bitrate settings.");
  } else {
    Serial.println("CAN init OK");
  }
}

void loop() {
  const uint32_t nowMs = millis();
  if (nowMs - lastPublishMs >= kPublishPeriodMs) {
    lastPublishMs = nowMs;
    publishSensor1Weight();
  }

  delay(1);
}