# ESP32 gamepad receiver

Firmware for an ESP32-WROOM-32 that connects a PS4 controller (DualShock 4; DualSense and
Xbox Series X|S work as well) with [Bluepad32](https://github.com/ricardoquesada/bluepad32)
and forwards its state over UART to the grblHAL controller board. Protocol, allow list and
wiring are described at the top of [`sketch.cpp`](sketch.cpp).

An ESP32-S3 or -C3 only has Bluetooth Low Energy and works with Xbox Series X|S controllers
only, PS4/PS5 controllers need the classic ESP32 (Bluetooth Classic).

Tested and working with a PS4 controller:

- **ESP32 DevKit** (ESP32-WROOM-32, 30/38 pins)
- **D1 Mini ESP32** (ESP32-WROOM-32, USB-C with CH9102F). It has no BOOT button: pair the controller
  from the display (gamepad plugin 1.2, display link 1.1), see [Pairing](#pairing).

Both use the same firmware (environment `esp32dev`). A new board is flashed completely once
(`firmware.factory.bin` at 0x0, or `pio run -e esp32dev -t upload`).

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
4. For updates over the display (see below): copy `esp32/partitions_ota.csv` to `esp32_gamepad_build/` and
   set in `sdkconfig.esp32dev` (or, before the first build, in `sdkconfig.defaults`):
   ```
   CONFIG_PARTITION_TABLE_CUSTOM=y
   CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions_ota.csv"
   # CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE is not set
   CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y
   ```
   `nvs` stays where it was, the allowed controllers are kept.
5. Build and flash the environment `esp32dev` with PlatformIO from PowerShell or VS Code, not from
   Git Bash (ESP-IDF refuses to install its tools in an MSYS shell). The first time with the new
   partition table the whole flash is written (bootloader, partition table, app).

## Update over the P4 display

From sketch 1.6 on (with `partitions_ota.csv`; 1.4 and 1.5 give up at the first block, flash them once
over USB) the receiver can be updated without opening the housing:
copy the app image `firmware.bin` from `esp32_gamepad_build/.pio/build/esp32dev/` as **`gamepad.bin`** to the
microSD card of the CrowPanel ESP32-P4 display and switch the display off and on. The display sends the file
to the receiver while it starts (progress on the screen, about 15 s) and renames it to `gamepad.done`, or
`gamepad.bad` when it failed (the display console shows the reason, `FD,0,<why>`). A block with lost bytes
is sent again. The receiver checks the image before it starts it; a firmware that does not get
up is replaced by the previous one at its next start.

## Wiring

| grblHAL board UART (e.g. BTT Octopus Pro TFT header) | ESP32 DevKit | D1 Mini ESP32 |
|---|---|---|
| 5V | 5V / VIN | VCC |
| GND | GND | GND |
| TX | GPIO16 (RX2, "D16") | IO16 |
| RX | GPIO17 (TX2, "D17") | IO17 |

On the D1 Mini ESP32 the four pins are next to each other in the inner pin row on the TXD/RXD side
(IO17, IO16, GND, VCC).

Baud rate: `UART_BAUD` in `sketch.cpp` (921600) and `GAMEPAD_BAUD` in grblHAL's `my_machine.h` must be
the same. **Directly at the BTT Octopus Pro at most 460800** (230400 recommended): at 921600 the Octopus
froze now and then during SD card access, see the main [README](../README.md#baud-rate). With the
CrowPanel ESP32-P4 display and the Octopus-Bridge the 921600 stays, the bridge passes it on at 230400.

Do not use RX0/TX0, they belong to the USB port of the ESP32. While the ESP32 is connected to the PC
over USB leave the 5V wire open. A reset pin on the board connector (e.g. RST on the BTT TFT header)
is not connected.

## Pairing

- Hold **BOOT** for 3 s or tap **Pair controller** on the gamepad screen of the display: the on-board
  LED blinks for 60 s, put the controller in pairing mode (PS4: Share + PS until the light bar flashes).
  The controller is added to the allow list. It holds 8 controllers; a ninth one replaces the oldest
  (sketch 1.5 and newer).
- A controller that is already allowed can pair again at any time (e.g. after it was used on a console).
- Hold **BOOT** for 10 s or hold **Forget all** on the display to clear the allow list.
- The on-board LED is on while a controller is connected.
- Boards without a BOOT button (e.g. D1 Mini ESP32) pair from the display only. This needs the gamepad
  plugin 1.2 and the display link 1.1 in the controller firmware.
