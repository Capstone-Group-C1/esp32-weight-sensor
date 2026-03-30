#include <Arduino.h>
#include <SPI.h>
#include <HX711.h>
#include <mcp2515.h>
#include <cstring>

constexpr size_t kSensorCount = 2;
const bool kReceiverOnline = true;

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
constexpr bool kUseLoopbackWhenReceiverOffline = false;
constexpr bool kAutoSwitchClockOnFailTx = true;
constexpr uint8_t kFailTxThresholdForClockSwitch = 10;

// Status / tare flags expected by Python decoder.
constexpr uint8_t kStatusOk = 0x00;
constexpr uint8_t kTareSuccess = 0x01;

// Publish rate in milliseconds.
constexpr uint32_t kPublishPeriodMs = 1000;

HX711 scales[kSensorCount];
MCP2515 mcp2515(MCP2515_CS_PIN);

uint32_t lastPublishMs = 0;
uint8_t failTxStreak = 0;
bool using16MHzClock = false;
bool clockFallbackAttempted = false;

enum class CanInitStage
{
  Ok,
  Bitrate,
  LoopbackMode,
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
    scales[i].tare();
  }
}

bool waitForLoopbackFrame(struct can_frame *outFrame, uint32_t timeoutMs)
{
  const uint32_t startMs = millis();
  while ((millis() - startMs) < timeoutMs)
  {
    if (mcp2515.checkReceive())
    {
      if (mcp2515.readMessage(outFrame) == MCP2515::ERROR_OK)
      {
        return true;
      }
    }

    delay(1);
  }

  return false;
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
  pinMode(MCP2515_INT_PIN, INPUT_PULLUP);

  mcp2515.reset();

  if (mcp2515.setBitrate(CAN_500KBPS, activeClock()) != MCP2515::ERROR_OK)
  {
    if (failedStage != nullptr)
    {
      *failedStage = CanInitStage::Bitrate;
    }
    return false;
  }

  if (!kReceiverOnline && kUseLoopbackWhenReceiverOffline)
  {
    if (mcp2515.setLoopbackMode() != MCP2515::ERROR_OK)
    {
      if (failedStage != nullptr)
      {
        *failedStage = CanInitStage::LoopbackMode;
      }
      return false;
    }

    return true;
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
  if (!kReceiverOnline && kUseLoopbackWhenReceiverOffline)
  {
    struct can_frame staleFrame;
    while (mcp2515.readMessage(&staleFrame) == MCP2515::ERROR_OK)
    {
      // Drain stale loopback frames so next read corresponds to this TX.
    }
  }

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

    if (!kReceiverOnline && txResult == MCP2515::ERROR_FAILTX)
    {
      Serial.println("No ACK expected yet (receiver offline). err=4 is expected in one-shot mode.");
      return;
    }

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
  Serial.printf("TX id=0x%03X weight=%.3f\n", frame.can_id, rawWeight);

  if (!kReceiverOnline && kUseLoopbackWhenReceiverOffline)
  {
    struct can_frame rxFrame;
    if (waitForLoopbackFrame(&rxFrame, 20))
    {
      float loopbackWeight = 0.0f;
      std::memcpy(&loopbackWeight, &rxFrame.data[1], sizeof(float));
      if (rxFrame.can_id == frame.can_id && rxFrame.data[0] == kBinId)
      {
        Serial.printf("RX loopback id=0x%03X weight=%.3f\n", rxFrame.can_id, loopbackWeight);
      }
      else
      {
        Serial.printf("RX loopback mismatch: id=0x%03X bin_id=%u expected=0x%03X/%u\n",
                      rxFrame.can_id,
                      rxFrame.data[0],
                      frame.can_id,
                      kBinId);
      }
    }
    else
    {
      Serial.println("Loopback RX frame not available.");
    }
  }
}

void publishAverageWeight()
{
  float sensor1Weight = 0.0f;
  float sensor2Weight = 0.0f;

  if (scales[0].is_ready())
  {
    sensor1Weight = scales[0].get_units(1);
  }

  if (scales[1].is_ready())
  {
    sensor2Weight = scales[1].get_units(1);
  }

  // Calculate and send average of both sensors
  float averageWeight = (sensor1Weight + sensor2Weight) / 2.0f;
  Serial.printf("Sensor1=%.3f Sensor2=%.3f Average=%.3f\n", sensor1Weight, sensor2Weight, averageWeight);
  sendWeightFrame(averageWeight);
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
    if (!kReceiverOnline && kUseLoopbackWhenReceiverOffline)
    {
      Serial.printf("CAN init OK (loopback mode, no external ACK required, clock=%s)\n", activeClockName());
    }
    else
    {
      Serial.printf("CAN init OK (normal one-shot mode, clock=%s)\n", activeClockName());
    }
  }
}

void loop()
{
  const uint32_t nowMs = millis();
  if (nowMs - lastPublishMs >= kPublishPeriodMs)
  {
    lastPublishMs = nowMs;
    publishAverageWeight();
  }

  delay(1);
}