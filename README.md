# ESP32 Weight Sensor Over CAN

This project reads four HX711 load cell amplifiers on an ESP32, averages the four weight readings, and transmits the averaged value over CAN using an MCP2515 module.

## Hardware

- ESP32 Dev Module
- 4x HX711 boards
- 4x load cells (one per HX711)
- MCP2515 CAN controller + transceiver module (8 MHz crystal expected by current firmware)

## Firmware Summary

The application in src/main.cpp does the following:

- Initializes 4 HX711 instances.
- Tares all channels at startup.
- Reads all channels periodically.
- Averages all ready sensor readings.
- Sends one CAN frame containing the averaged weight.

Publish period is 50 ms by default.

## Pin Mapping

### MCP2515 to ESP32

- SCK: GPIO18
- MISO: GPIO19
- MOSI: GPIO23
- CS: GPIO5
- INT: GPIO4

### HX711 to ESP32

Each HX711 uses one DOUT and one SCK pin:

- HX711 #1: DOUT GPIO34, SCK GPIO32
- HX711 #2: DOUT GPIO35, SCK GPIO33
- HX711 #3: DOUT GPIO36, SCK GPIO25
- HX711 #4: DOUT GPIO39, SCK GPIO26

Note:

- GPIO34, GPIO35, GPIO36, GPIO39 are input-only on ESP32 and are used here for HX711 DOUT.
- All grounds must be common across ESP32, HX711 modules, and MCP2515 module.

## CAN Settings

Current CAN settings in firmware:

- Bitrate: 500 kbps
- MCP2515 oscillator: 8 MHz
- Mode: Normal one-shot mode
- CAN ID: 0x200
- DLC: 8 bytes

### CAN Payload Format (ID 0x200)

- Byte 0: Number of contributing sensors (0 to 4)
- Byte 1: Ready flag (1 if at least one sensor contributed, otherwise 0)
- Byte 2: Weight int32, bits 7:0 (little-endian)
- Byte 3: Weight int32, bits 15:8
- Byte 4: Weight int32, bits 23:16
- Byte 5: Weight int32, bits 31:24
- Byte 6: Reserved (0)
- Byte 7: Reserved (0)

Weight is transmitted as signed int32 in milli-units.
If your engineering unit is kilograms, the value is milligrams-equivalent in that unit system (example: 1.234 -> 1234).

## Build and Upload

This project is intended to be built and uploaded from VS Code using the PlatformIO extension.

### 1) Install PlatformIO in VS Code

1. Open VS Code.
2. Open the Extensions view.
3. Search for PlatformIO IDE.
4. Install the extension by PlatformIO.
5. Restart VS Code if prompted.

### 2) Open and Build the Project

1. Open this repository folder in VS Code.
2. Wait for PlatformIO to finish indexing dependencies.
3. Use the PlatformIO toolbar at the bottom of VS Code:
   - Checkmark icon: Build
   - Right-arrow icon: Upload

You can also open the PlatformIO sidebar and use:

- Project Tasks > esp32dev > General > Build
- Project Tasks > esp32dev > General > Upload

### 3) Serial Monitor Setup

Firmware serial logging is configured at 115200 baud in setup().

To open serial output in VS Code:

1. Connect the ESP32 over USB.
2. Click the plug icon in the PlatformIO toolbar (Monitor).
3. Set the monitor baud rate to 115200.

If needed, add this to platformio.ini to lock monitor speed:

monitor_speed = 115200

## Calibration

Per-channel scale factors are defined in src/main.cpp:

- kScaleFactors[0]
- kScaleFactors[1]
- kScaleFactors[2]
- kScaleFactors[3]

To calibrate:

1. Set all scale factors to 1.0 temporarily.
2. Upload firmware and tare with no load.
3. Place a known reference weight on each load cell individually.
4. Compute each scale factor from raw reading versus known mass.
5. Update kScaleFactors and rebuild.

## Notes

- If no CAN receiver is online, one-shot mode may report TX errors due to missing ACK; this is expected behavior in that condition.
- If your MCP2515 module uses a 16 MHz crystal, update the oscillator setting in code from MCP_8MHZ to MCP_16MHZ.
