#include <Arduino.h>
#include <SPI.h>
#include <HX711.h>
#include <mcp2515.h>
#include <cstring>

constexpr size_t kSensorCount = 2;

// HX711 wiring for each channel: {DOUT, SCK}
// Sensor 1: DOUT=35, SCK=33
// Sensor 2: DOUT=34, SCK=32 (avoids SPI pins: 18, 19, 23, 5)
// DOUT pins use input-capable GPIOs; SCK pins use output-capable GPIOs.
constexpr uint8_t HX_PINS[kSensorCount][2] = {
    {35, 33},
    {34, 32},
};

// Per-sensor scale factors for HX711::set_scale().
// Replace these with your calibrated values.
float kScaleFactors[kSensorCount] = {
    4228.0f,
    4228.0f,
};

// MCP2515 CS pin on ESP32. SPI pins use board defaults.
constexpr uint8_t MCP2515_CS_PIN = 5;
constexpr uint8_t MCP2515_SCK_PIN = 18;
constexpr uint8_t MCP2515_MISO_PIN = 19;
constexpr uint8_t MCP2515_MOSI_PIN = 23;

// CAN message IDs used to publish each load cell.
// Matches Python driver receive filter: 0x100 - 0x1FF (0x100 + bin_id).
constexpr uint16_t kCanBaseId = 0x100;
constexpr uint8_t kBinId = 1;
constexpr bool kAutoSwitchClockOnFailTx = true;
constexpr uint8_t kFailTxThresholdForClockSwitch = 10;

// Status / tare flags expected by Python decoder.
constexpr uint8_t kStatusOk = 0x00;
constexpr uint8_t kTareSuccess = 0x01;

// Sampling/transmit cadence in milliseconds.
// This yields a 20 Hz publish rate (every 50 ms) with ~5 samples per average,
// which improves UI responsiveness while keeping CAN load very low at 500 kbps.
constexpr uint32_t kSamplePeriodMs = 10;
constexpr uint32_t kAverageWindowMs = 50;
constexpr bool kLogEveryPublish = false;

HX711 scales[kSensorCount];
MCP2515 mcp2515(MCP2515_CS_PIN);

uint32_t lastSampleMs = 0;
uint32_t windowStartMs = 0;
float windowSum = 0.0f;
uint32_t windowSampleCount = 0;
uint8_t failTxStreak = 0;
bool using16MHzClock = false;
bool clockFallbackAttempted = false;

enum class CanInitStage
{
  Ok,
  Bitrate,
  NormalOneShotMode,
};

CAN_CLOCK activeClock()
{
  return using16MHzClock ? MCP_16MHZ : MCP_8MHZ;
}

const char *activeClockName()
{
  return using16MHzClock ? "16MHz" : "8MHz";
}

void initializeHx711()
{
  for (size_t i = 0; i < kSensorCount; ++i)
  {
    const uint8_t dout = HX_PINS[i][0];
    const uint8_t sck = HX_PINS[i][1];

    scales[i].begin(dout, sck);
    scales[i].set_scale(kScaleFactors[i]);
  }
}

void printCanErrorRegs()
{
  const uint8_t eflg = mcp2515.getErrorFlags();
  const uint8_t tec = mcp2515.errorCountTX();
  const uint8_t rec = mcp2515.errorCountRX();
  Serial.printf("EFLG=0x%02X TEC=%u REC=%u\n", eflg, tec, rec);
}

bool initializeCan(CanInitStage *failedStage = nullptr)
{
  if (failedStage != nullptr)
  {
    *failedStage = CanInitStage::Ok;
  }

  // Match the ESP-IDF wiring used in your previous project.
  SPI.begin(MCP2515_SCK_PIN, MCP2515_MISO_PIN, MCP2515_MOSI_PIN, MCP2515_CS_PIN);

  mcp2515.reset();

  if (mcp2515.setBitrate(CAN_500KBPS, activeClock()) != MCP2515::ERROR_OK)
  {
    if (failedStage != nullptr)
    {
      *failedStage = CanInitStage::Bitrate;
    }
    return false;
  }

  if (mcp2515.setNormalOneShotMode() != MCP2515::ERROR_OK)
  {
    if (failedStage != nullptr)
    {
      *failedStage = CanInitStage::NormalOneShotMode;
    }
    return false;
  }

  return true;
}

bool switchClockProfileAndReinitialize()
{
  if (clockFallbackAttempted)
  {
    Serial.println("Clock profile fallback already attempted once; keeping current profile.");
    return false;
  }

  clockFallbackAttempted = true;
  const bool previousUsing16MHz = using16MHzClock;
  using16MHzClock = !using16MHzClock;
  Serial.printf("Switching CAN clock profile to %s\n", activeClockName());
  CanInitStage failedStage = CanInitStage::Ok;
  const bool ok = initializeCan(&failedStage);
  if (!ok)
  {
    using16MHzClock = previousUsing16MHz;
    Serial.printf("Reinit failed after clock switch (stage=%d). Reverting to %s profile.\n",
                  static_cast<int>(failedStage),
                  activeClockName());
    return false;
  }

  failTxStreak = 0;
  Serial.printf("Reinit OK with %s profile\n", activeClockName());
  return true;
}

void sendWeightFrame(float rawWeight)
{
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
  if (txResult != MCP2515::ERROR_OK)
  {
    Serial.printf("TX id=0x%03X failed, err=%d\n", frame.can_id, static_cast<int>(txResult));
    printCanErrorRegs();

    if (txResult == MCP2515::ERROR_FAILTX)
    {
      ++failTxStreak;
      Serial.printf("FAILTX streak=%u\n", failTxStreak);
      Serial.println("FAILTX usually means no ACK: verify bitrate/clock, wiring, and 120R termination.");

      if (kAutoSwitchClockOnFailTx && failTxStreak >= kFailTxThresholdForClockSwitch)
      {
        switchClockProfileAndReinitialize();
      }
    }
    else
    {
      failTxStreak = 0;
    }

    return;
  }

  failTxStreak = 0;
  if (kLogEveryPublish)
  {
    Serial.printf("TX id=0x%03X weight=%.3f\n", frame.can_id, rawWeight);
  }
}

bool readCombinedSample(float *outCombinedWeight)
{
  if (outCombinedWeight == nullptr)
  {
    return false;
  }

  if (!scales[0].is_ready() || !scales[1].is_ready())
  {
    return false;
  }

  const float sensor1Weight = scales[0].get_units(1);
  const float sensor2Weight = scales[1].get_units(1);

  // Current calibration/wiring yields negative values for load, so invert both.
  *outCombinedWeight = (-sensor1Weight) + (-sensor2Weight);
  return true;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  initializeHx711();

  CanInitStage failedStage = CanInitStage::Ok;
  if (!initializeCan(&failedStage))
  {
    Serial.printf("CAN init failed at stage=%d. Check MCP2515 wiring/clock/bitrate settings.\n",
                  static_cast<int>(failedStage));
  }
  else
  {
    Serial.printf("CAN init OK (normal one-shot mode, clock=%s)\n", activeClockName());
  }
}

void loop()
{
  const uint32_t nowMs = millis();

  if (nowMs - lastSampleMs >= kSamplePeriodMs)
  {
    lastSampleMs = nowMs;

    float combinedSample = 0.0f;
    if (readCombinedSample(&combinedSample))
    {
      windowSum += combinedSample;
      ++windowSampleCount;
    }
  }

  if (nowMs - windowStartMs >= kAverageWindowMs)
  {
    if (windowSampleCount > 0)
    {
      const float averageWeight = windowSum / static_cast<float>(windowSampleCount);
      if (kLogEveryPublish)
      {
        Serial.printf("TX window avg=%.3f from %lu samples over %lu ms\n",
                      averageWeight,
                      static_cast<unsigned long>(windowSampleCount),
                      static_cast<unsigned long>(kAverageWindowMs));
      }
      sendWeightFrame(averageWeight);
    }
    else
    {
      if (kLogEveryPublish)
      {
        Serial.printf("No valid samples in %lu ms window; skipping TX\n",
                      static_cast<unsigned long>(kAverageWindowMs));
      }
    }

    windowStartMs = nowMs;
    windowSum = 0.0f;
    windowSampleCount = 0;
  }

  delay(1);
}