#include <Arduino.h>
#include <SPI.h>
#include <HX711.h>
#include <mcp2515.h>

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
    1000.0f,
    1000.0f,
    1000.0f,
    1000.0f,
};

// MCP2515 CS pin on ESP32. SPI pins use board defaults.
constexpr uint8_t MCP2515_CS_PIN = 5;
constexpr uint8_t MCP2515_SCK_PIN = 18;
constexpr uint8_t MCP2515_MISO_PIN = 19;
constexpr uint8_t MCP2515_MOSI_PIN = 23;
constexpr uint8_t MCP2515_INT_PIN = 4;

// CAN message IDs used to publish each load cell.
// Averaged weight is sent on this standard ID.
constexpr uint16_t kCanBaseId = 0x200;

// Publish rate in milliseconds.
constexpr uint32_t kPublishPeriodMs = 50;

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

void sendWeightFrame(uint8_t contributingSensors, int32_t weightMilliUnits, bool sensorReady) {
  struct can_frame frame;
  frame.can_id = kCanBaseId;
  frame.can_dlc = 8;

  frame.data[0] = contributingSensors;
  frame.data[1] = sensorReady ? 1 : 0;
  frame.data[2] = static_cast<uint8_t>(weightMilliUnits & 0xFF);
  frame.data[3] = static_cast<uint8_t>((weightMilliUnits >> 8) & 0xFF);
  frame.data[4] = static_cast<uint8_t>((weightMilliUnits >> 16) & 0xFF);
  frame.data[5] = static_cast<uint8_t>((weightMilliUnits >> 24) & 0xFF);
  frame.data[6] = 0;
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

void publishAverageWeight() {
  float sumWeight = 0.0f;
  uint8_t readyCount = 0;

  for (uint8_t i = 0; i < kSensorCount; ++i) {
    if (!scales[i].is_ready()) {
      continue;
    }

    // Uses HX711 calibration factor to output engineering units.
    sumWeight += scales[i].get_units(1);
    ++readyCount;
  }

  if (readyCount == 0) {
    sendWeightFrame(0, 0, false);
    return;
  }

  const float averageWeight = sumWeight / static_cast<float>(readyCount);
  const int32_t avgWeightMilliUnits = static_cast<int32_t>(averageWeight * 1000.0f);
  sendWeightFrame(readyCount, avgWeightMilliUnits, true);
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
    publishAverageWeight();
  }

  delay(1);
}