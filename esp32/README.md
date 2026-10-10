# ESP32 gamepad receiver

Firmware for an ESP32-WROOM-32 that connects a PS4 controller (DualShock 4; DualSense and
Xbox Series X|S work as well) with [Bluepad32](https://github.com/ricardoquesada/bluepad32)
and forwards its state over UART to the grblHAL controller board. Protocol, allow list and
wiring are described at the top of [`sketch.cpp`](sketch.cpp).

An ESP32-S3 or -C3 only has Bluetooth Low Energy and works with Xbox Series X|S controllers
only, PS4/PS5 controllers need the classic ESP32 (Bluetooth Classic).

## Build

Bluepad32 is built from its official ESP-IDF + Arduino template, only `sketch.cpp` comes
from this repository. ESP-IDF does not build in paths with spaces.

1. Clone the template with its components to a path without spaces:
   ```
   git clone --recursive https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template.git esp32_gamepad_build
   ```
2. With PlatformIO Core 6.2 the template's platform 54.03.21 fails with
   `No module named 'SCons.Tool.FortranCommon'`. Adapt it:
   - `platformio.ini`: `platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.312-1/platform-espressif32.zip`
   - Arduino component to the matching version:
     `git -C components/arduino fetch --depth 1 origin tag 3.3.12` and `git -C components/arduino checkout 3.3.12`
   - `components/cmd_system/cmd_system.c`: add `#include <driver/gpio.h>` (needed with ESP-IDF 5.5)
3. Copy `esp32/sketch.cpp` to `esp32_gamepad_build/main/sketch.cpp`.
4. Build and flash the environment `esp32dev` with PlatformIO from PowerShell or VS Code, not from
   Git Bash (ESP-IDF refuses to install its tools in an MSYS shell).

## Wiring

| grblHAL board UART (e.g. BTT Octopus Pro TFT header) | ESP32 DevKit |
|---|---|
| 5V | 5V / VIN |
| GND | GND |
| TX | GPIO16 (RX2, "D16") |
| RX | GPIO17 (TX2, "D17") |

Do not use RX0/TX0, they belong to the USB port of the ESP32. While the ESP32 is connected to the PC
over USB leave the 5V wire open. A reset pin on the board connector (e.g. RST on the BTT TFT header)
is not connected.

## Pairing

- Hold **BOOT** for 3 s: the on-board LED blinks for 60 s, put the controller in pairing mode
  (PS4: Share + PS until the light bar flashes). The controller is added to the allow list.
- A controller that is already allowed can pair again at any time (e.g. after it was used on a console).
- Hold **BOOT** for 10 s to clear the allow list.
- The on-board LED is on while a controller is connected.
