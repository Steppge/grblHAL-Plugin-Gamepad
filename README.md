# grblHAL-Plugin-Gamepad

Jog a [grblHAL](https://github.com/grblHAL) CNC or laser machine with a **Bluetooth gamepad**
(PS4 DualShock 4; DualSense and Xbox Series X|S work as well), or alternatively with an **analog joystick**.

The gamepad is connected to an **ESP32** running [Bluepad32](https://github.com/ricardoquesada/bluepad32).
The ESP32 sends the controller state over a UART to the grblHAL board, all machine logic runs in grblHAL.
The light bar shows the machine state, the controller rumbles on alarms and when a job is finished.

Developed for a BTT Octopus Pro v1.0 (STM32F446), see
[Steppge/grblHAL-OctopusPro-CNC-Laser](https://github.com/Steppge/grblHAL-OctopusPro-CNC-Laser).
The plugins only use grblHAL core APIs and are not tied to a driver or MCU.

> **Prototype.** Keep a hand on the power or emergency stop switch when trying it on your machine.

## Contents

| File | |
|---|---|
| [`gamepad_plugin.c`](gamepad_plugin.c) | Bluetooth gamepad plugin (`GAMEPAD_ENABLE`) |
| [`joystick_plugin.c`](joystick_plugin.c) | Analog joystick plugin (`JOYSTICK_ANALOG_ENABLE`) |
| [`my_plugin.c.example`](my_plugin.c.example) | Starts the plugins from `my_plugin_init()` |
| [`esp32/`](esp32/) | ESP32 firmware (Bluepad32) with build instructions |

## Installation (PlatformIO)

1. Add this repository to your grblHAL driver, next to the other plugins (e.g. `keypad`, `trinamic`):
   ```
   git submodule add https://github.com/Steppge/grblHAL-Plugin-Gamepad gamepad
   ```
   or download it and unpack it as folder `gamepad`.
2. `platformio.ini`, in your environment:
   - add `gamepad` to `lib_deps`
   - add `-D ADD_MY_PLUGIN=1` to `build_flags`
3. Copy `my_plugin.c.example` to `Src/my_plugin.c` (or add its calls to your existing `my_plugin_init()`).
4. `Inc/my_machine.h`:
   ```c
   #define GAMEPAD_ENABLE          1 // Bluetooth gamepad via ESP32
   #define JOYSTICK_ANALOG_ENABLE  0 // or the analog joystick, enable only one
   //#define GAMEPAD_STREAM        0 // serial stream instance of the UART the ESP32 is connected to
   //#define GAMEPAD_BAUD     115200
   ```
5. Build and flash the ESP32 firmware, see [`esp32/README.md`](esp32/README.md), and connect it.

For CMake based builds use the included `CMakeLists.txt` (library `gamepad`), like the official plugins.

### Which serial port?

`GAMEPAD_STREAM` is the instance number of a serial stream of your driver. With the main connection on
USB, instance 0 is usually the first UART (`SERIAL_PORT` in the board map). On the BTT Octopus Pro that
is USART1 on the TFT header. The port must not be used by anything else (e.g. MPG or keypad plugin).

## Wiring

| Board UART | ESP32 DevKit |
|---|---|
| 5V | 5V / VIN |
| GND | GND |
| TX | GPIO16 (RX2) |
| RX | GPIO17 (TX2) |

While the ESP32 is connected to the PC over USB leave the 5V wire open.

## Controls (gamepad)

| Control | Function |
|---|---|
| Left stick up/down | Y (10 speed steps) |
| Right stick left/right | X (10 speed steps) |
| L2 / R2 (analog) | Z up / Z down (both pressed: Z stops) |
| Options | Jog enable on/off (light bar red / blue) |
| Circle | Tap: feed hold. After the hold: hold = STOP (abort the job) |
| Cross | Tap: resume (cycle start) |
| Triangle | Hold: home |
| Square | Hold: X0 Y0 |
| R1 | Hold: Z0 |
| PS | Tap: reset after a hard limit alarm, else unlock |
| Share | Tap: next jog step 0.01 / 0.05 / 0.1 / 1 / 10 mm |
| D-pad, idle | One jog step X-/X+/Y-/Y+ per press (needs jog enable) |
| D-pad, job | Up/down: feed override ±10 %, right/left: power/spindle override ±10 % |

"Hold" = 0.6 s. Every axis has its own control, so a slightly diagonal stick never moves a second axis.
Step 1 has a fixed slow speed (5 mm/min X/Y, 2 mm/min Z) for touching off, steps 2-10 rise up to 35 % of
the axis max rate (`$110`-`$112`). Axis directions can be reversed with `GP_INVERT_X/Y/Z` in the plugin.

If a display plugin provides `tft_move_step_mm()` and `tft_move_step_next()`, the jog step is shared with it,
otherwise the plugin uses its own.

## Light bar and rumble

| State | Light bar |
|---|---|
| Idle, jog locked | blue |
| Idle or jog, jog enabled | red |
| Job running | green |
| Hold | orange |
| Homing | violet |
| Alarm | red blinking |
| Controller battery below 20 % | short yellow flash every 10 s |

Rumble: short click on acknowledges, long on an alarm, two pulses when a job running at least 10 s has finished.

## Safety

- Jogging needs the jog enable (Options). It switches off by itself after 60 s without motion, when the
  controller disconnects, on an alarm, when a job starts and at power-up. It can only be switched on while
  the machine is idle and all sticks and triggers are at rest.
- Releasing a stick or trigger stops the axis. No valid packet for 200 ms (cable, ESP32) stops everything.
- Jogging only happens while idle or jogging, never during a job or an alarm.
- The ESP32 only accepts controllers on its allow list. Packets carry a checksum.
- Recommended grblHAL settings: soft limits (`$20=1`), jog soft limit clipping (`$40=1`),
  hard limits strict mode (`$21=3`).

## Status

- `$I` shows `[PLUGIN:Gamepad v1.0 (connected, jog locked)]`, `(no controller)` or `(no ESP32)`.
- The status report contains `|Pad:connected,enabled,rx,ly,l2,r2` (disable with `GP_DEBUG 0`).

## Protocol (UART, 115200 baud, ASCII lines)

```
ESP32 -> grblHAL, every 20 ms:
  $GP,<seq>,<conn>,<lx>,<ly>,<rx>,<ry>,<l2>,<r2>,<buttons>,<dpad>,<misc>,<battery>*<hh>
grblHAL -> ESP32, on changes and once a second:
  $ST,<state>,<event>,<enabled>*<hh>
```
`<hh>` is the XOR of all characters between `$` and `*`. Values are the Bluepad32 ones (sticks -512..511,
triggers 0..1023, button bitmasks in hex). States: 0 idle, 1 run, 2 hold, 3 alarm, 4 jog, 5 homing, 6 other.
Events: 1 alarm, 2 job finished, 3 acknowledge.

## Analog joystick (alternative)

[`joystick_plugin.c`](joystick_plugin.c) jogs with three analog sticks (e.g. from an old RC transmitter) and an
enable switch on a digital aux input:

- Analog aux inputs `JOYSTICK_PORT_X/Y/Z` (default 0, 1, 2), enable switch `JOYSTICK_ENABLE_PORT` (default 3).
- Same 10 speed steps with hysteresis, smoothed readings, start lock, implausible readings (broken wire)
  stop the motion.
- The stick calibration (`joy_cfg[]`) is the one of the author's sticks, measure yours with the
  `|Joy:x,y,z,en` values in the status report.
- The driver must support enough analog aux inputs. The STM32F4xx driver supports two by default.

## License

GPL v3 or later, see [LICENSE](LICENSE). Copyright (c) 2026 Steppge.
grblHAL by Terje Io and contributors, Bluepad32 by Ricardo Quesada (Apache 2.0).
