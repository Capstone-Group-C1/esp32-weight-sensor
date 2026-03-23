# ESP32 CAN Counter Transmit Test

This project is a simple CAN transmission test on ESP32 + MCP2515. It transmits an incrementing counter once per second.

## Hardware

- ESP32 Dev Module
- MCP2515 CAN controller + transceiver module (8 MHz crystal expected by current firmware)

## Firmware Summary

The application in src/main.cpp does the following:

- Initializes MCP2515 at 500 kbps with 8 MHz oscillator setting.
- Sends one CAN frame every 1000 ms.
- Stores the counter in bytes 0-3 (little-endian).
- Uses bytes 4-7 as zeros.

## Pin Mapping

### MCP2515 to ESP32

- SCK: GPIO18
- MISO: GPIO19
- MOSI: GPIO23
- CS: GPIO5
- INT: GPIO4

## CAN Settings

Current CAN settings in firmware:

- Bitrate: 500 kbps
- MCP2515 oscillator: 8 MHz
- Mode: Normal one-shot mode
- CAN ID: 0x100
- DLC: 8 bytes

### CAN Payload Format (ID 0x100)

- Byte 0: Counter bits 7:0
- Byte 1: Counter bits 15:8
- Byte 2: Counter bits 23:16
- Byte 3: Counter bits 31:24
- Byte 4: Reserved (0)
- Byte 5: Reserved (0)
- Byte 6: Reserved (0)
- Byte 7: Reserved (0)

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

## Notes

- If no CAN receiver is online, one-shot mode may report TX errors due to missing ACK; this is expected behavior.
- If your MCP2515 module uses a 16 MHz crystal, update the oscillator setting in code from MCP_8MHZ to MCP_16MHZ.

## Raspberry Pi Receive Test (MCP2515)

Use this section to confirm the ESP32 counter is visible on a Raspberry Pi CAN interface.

### 1) Bring up `can0` on Raspberry Pi

Install CAN tools:

```bash
sudo apt update
sudo apt install -y can-utils python3-pip
python3 -m pip install --user python-can
```

Bring interface down/up at 500 kbps:

```bash
sudo ip link set can0 down || true
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

### 2) Quick raw frame check

```bash
candump can0,100:7FF
```

You should see one frame per second with ID `100`.

### 3) Decode counter with Python script

From this repository:

```bash
python3 tools/pi_can_counter_rx.py --channel can0 --can-id 0x100
```

Expected output pattern:

```text
[12:34:56] id=0x100 dlc=8 counter=123 first
[12:34:57] id=0x100 dlc=8 counter=124 delta=1
[12:34:58] id=0x100 dlc=8 counter=125 delta=1
```

### 4) Wiring sanity

- CANH to CANH, CANL to CANL.
- Common ground between ESP32 CAN module and Pi CAN module.
- 120 ohm termination at each end of the CAN bus.
