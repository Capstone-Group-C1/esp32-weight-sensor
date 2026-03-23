#include <Arduino.h>
#include <HX711.h>

constexpr size_t kSensorCount = 2;

// HX711 wiring for first two sensors only
constexpr uint8_t HX_PINS[kSensorCount][2] = {
  {34, 32},
  {35, 33},
};

// Calibration factors (update after calibration)
float kScaleFactors[kSensorCount] = {
  4200.62f,
  4242.00f,
};

HX711 scales[kSensorCount];

constexpr uint32_t kReadPeriodMs = 200;
uint32_t lastReadMs = 0;

void initializeHx711() {
  for (size_t i = 0; i < kSensorCount; ++i) {
    scales[i].begin(HX_PINS[i][0], HX_PINS[i][1]);
    scales[i].set_scale(kScaleFactors[i]);
    scales[i].tare();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  initializeHx711();

  Serial.println("HX711 Initialized (2 sensors)");
}

void loop() {
  const uint32_t nowMs = millis();

  if (nowMs - lastReadMs >= kReadPeriodMs) {
    lastReadMs = nowMs;

    for (size_t i = 0; i < kSensorCount; ++i) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(": ");

      if (scales[i].is_ready()) {
        float weight = scales[i].get_units(1);
        Serial.print(weight, 3); // 3 decimal places
        Serial.print(" g");
      } else {
        Serial.print("NOT READY");
      }

      Serial.print("\t");
    }

    Serial.println();
  }

  delay(100);
}