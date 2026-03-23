#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

// MCP2515 CS pin on ESP32. SPI pins use board defaults.
constexpr uint8_t MCP2515_CS_PIN = 5;
constexpr uint8_t MCP2515_SCK_PIN = 18;
constexpr uint8_t MCP2515_MISO_PIN = 19;
constexpr uint8_t MCP2515_MOSI_PIN = 23;
constexpr uint8_t MCP2515_INT_PIN = 4;

constexpr uint32_t kCanId = 0x100;
constexpr uint32_t kPublishPeriodMs = 1000;
constexpr bool kReceiverOnline = true;
constexpr bool kUseLoopbackWhenReceiverOffline = false;
constexpr bool kAutoSwitchClockOnFailTx = true;
constexpr uint8_t kFailTxThresholdForClockSwitch = 10;

MCP2515 mcp2515(MCP2515_CS_PIN);

uint32_t lastPublishMs = 0;
uint32_t txCounter = 0;
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

void sendCounterFrame()
{
  if (!kReceiverOnline && kUseLoopbackWhenReceiverOffline)
  {
    struct can_frame staleFrame;
    while (mcp2515.readMessage(&staleFrame) == MCP2515::ERROR_OK)
    {
      // Drain pending loopback frames so the next read corresponds to this TX.
    }
  }

  struct can_frame frame;
  frame.can_id = kCanId;
  frame.can_dlc = 8;
  frame.data[0] = static_cast<uint8_t>(txCounter & 0xFF);
  frame.data[1] = static_cast<uint8_t>((txCounter >> 8) & 0xFF);
  frame.data[2] = static_cast<uint8_t>((txCounter >> 16) & 0xFF);
  frame.data[3] = static_cast<uint8_t>((txCounter >> 24) & 0xFF);
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;

  const MCP2515::ERROR txResult = mcp2515.sendMessage(&frame);
  if (txResult != MCP2515::ERROR_OK)
  {
    Serial.printf("TX failed, err=%d\n", static_cast<int>(txResult));
    printCanErrorRegs();

    if (!kReceiverOnline && txResult == MCP2515::ERROR_FAILTX)
    {
      Serial.println("No ACK expected yet (receiver offline). err=4 is expected in one-shot mode.");
      ++txCounter;
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
  Serial.printf("TX id=0x%03lX counter=%lu\n", kCanId, txCounter);

  if (!kReceiverOnline && kUseLoopbackWhenReceiverOffline)
  {
    struct can_frame rxFrame;
    if (waitForLoopbackFrame(&rxFrame, 20))
    {
      const uint32_t rxCounter =
          static_cast<uint32_t>(rxFrame.data[0]) |
          (static_cast<uint32_t>(rxFrame.data[1]) << 8) |
          (static_cast<uint32_t>(rxFrame.data[2]) << 16) |
          (static_cast<uint32_t>(rxFrame.data[3]) << 24);
      if (rxFrame.can_id == kCanId && rxCounter == txCounter)
      {
        Serial.printf("RX loopback id=0x%03lX counter=%lu\n", rxFrame.can_id, rxCounter);
      }
      else
      {
        Serial.printf("RX loopback mismatch: id=0x%03lX counter=%lu expected=0x%03lX/%lu\n",
                      rxFrame.can_id,
                      rxCounter,
                      kCanId,
                      txCounter);
      }
    }
    else
    {
      Serial.println("Loopback RX frame not available.");
    }
  }

  ++txCounter;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

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
    sendCounterFrame();
  }

  delay(1);
}