/*
  esp32/sketch.cpp - Bluetooth gamepad receiver for the grblHAL gamepad plugin

  Part of grblHAL-Plugin-Gamepad, https://github.com/Steppge/grblHAL-Plugin-Gamepad

  Copyright (c) 2026 Steppge

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  ESP32-WROOM-32 with Bluepad32. Connects a PS4 controller (DualShock 4; DualSense and
  Xbox Series X|S work the same way) and forwards its state over UART to the grblHAL controller board.
  All machine logic (jogging, dead man switch, buttons) is done in grblHAL, this sketch only
  transports data and drives the light bar and rumble.

  Wiring (board UART, e.g. BTT Octopus Pro TFT header):  5V -> 5V/VIN,  GND -> GND,  board TX -> GPIO16 (RX2),  board RX <- GPIO17 (TX2)

  Allowed controllers
    Only controllers stored in the allow list may connect, others are disconnected at once.
    A stored controller may always pair again (e.g. after it was used on a console).
    To add a new controller: hold BOOT (GPIO0) for 3 s or start it on the display ($PA,1), the
    on-board LED blinks for 60 s, put the controller in pairing mode (PS4: Share + PS until the
    light bar flashes). Hold BOOT for 10 s or $PA,2 from the display to clear the allow list
    (a connected controller is disconnected). Boards without BOOT button pair from the display.
    Bluepad32 only searches for controllers while the pairing window is open: a search running
    while an allowed controller reconnects by itself makes the reconnection fail (and could crash
    Bluepad32 4.2.2).

  Protocol (ASCII lines, 460800 baud (921600 before 1.7), checksum = XOR of all characters between '$' and '*')
    ESP32 -> board, every 20 ms:
      $GP,<seq>,<conn>,<lx>,<ly>,<rx>,<ry>,<l2>,<r2>,<buttons>,<dpad>,<misc>,<battery>,<pair>*<hh>
        conn     1 = controller connected, 0 = no controller (heartbeat, all values 0)
        lx..ry   sticks -512..511 (Bluepad32, up = negative)
        l2, r2   analog triggers 0..1023
        buttons  Bluepad32 button bitmask (hex), dpad (hex), misc buttons (hex)
        battery  0..255, 0 = unknown
        pair     seconds left in the pairing window, 0 = closed
    board -> ESP32, on changes and once a second:
      $ST,<state>,<event>,<enabled>*<hh>
        state    0 idle, 1 run, 2 hold, 3 alarm, 4 jog, 5 homing, 6 other
        event    0 none, 1 alarm/limit, 2 job finished, 3 short click (acknowledge)
        enabled  jog enable 1/0
    board -> ESP32, from the display:
      $PA,<cmd>*<hh>
        cmd      1 = open the pairing window, 0 = close it, 2 = clear the allow list
      $PW,<on>*<hh>       (1.9, the P4 display sends it every 2 s)
        on       0 = radio off: the controller is disconnected (it switches itself off), Bluetooth is
                 powered down and the CPU runs at 80 MHz, only the UART stays; 1 = on again (default)
    ESP32 -> board, Bluetooth log (1.11, for the diagnosis of slow connections; the P4 display shows them
    with "gplog on", the gamepad plugin ignores them):
      $LG,<ms>,<event>[,<values>]*<hh>
        req,<addr>          a controller asks for a connection     conn,<status>,<addr>  connection complete
        keyreq / pin / ssp  link key, PIN or confirmation asked    auth,<status>  enc,<status>,<on>
        disc,<status>,<reason>                                     state,<n>  BTstack state
        pad,<model> / rejected,<addr> / gone                       Bluepad32: controller up, refused, away
      status / reason: HCI error codes (0 = success, 0x04 page timeout, 0x05 auth failure, 0x08 supervision
      timeout, 0x13 remote user, 0x16 local host)

  Light bar
    idle locked blue, idle or jog with jog enable violet, job green, hold orange, homing red,
    alarm violet blinking (violet / red with jog enable: home works). 1.10: red and violet swapped.
    Controller battery below 20 %:
    short yellow flash every 10 s.
  Rumble
    short click on acknowledges, long strong on an alarm, two medium pulses when a job is finished.

  Stalled link: a DualShock 4 / DualSense sends its state all the time. When nothing new comes for
  STALL_MS while Bluetooth still reports the controller as connected (battery empty, out of reach), the
  packets report "no controller" (conn 0), so grblHAL stops jogging at once instead of after the
  Bluetooth timeout of some seconds. Xbox controllers only send on changes, there the rule is off.

  Firmware update over the UART (from the P4 display, gamepad.bin on its SD card), needs the two OTA
  partitions of partitions_ota.csv:
    board -> ESP32   $FW,<size>,<crc32 hex>*<hh>      then the image in blocks of FW_BLOCK bytes
    ESP32 -> board   $FA,<1 ready / 0 refused>*<hh>   $FK,<bytes written>*<hh> after each block
                     $FD,1,<resends>*<hh>             done: the ESP32 restarts with the new firmware
                     $FD,0,<why>*<hh>                 failed
  Lost bytes: when a block stays incomplete for FW_RESEND_MS, the ESP32 drops it and sends the last
  $FK,<bytes written> again, the board then sends that block once more (up to FW_RESENDS times in a row).
  The image is checked (CRC32 and the image check of the ESP32) before it becomes the boot partition. A new
  firmware that does not get up is replaced by the old one at the next start (app rollback).
*/

#include "sdkconfig.h"

#include <Arduino.h>
#include <Bluepad32.h>
#include <Preferences.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <esp_rom_crc.h>
#include <btstack.h>            // page scan interval (gap_set_page_scan_activity), HCI events for $LG
#include <stdarg.h>

#define SKETCH_VERSION  "1.12"
#define STALL_MS        200     // DualShock 4 / DualSense: no new data this long = link stalled
#define FW_BLOCK        1024    // firmware update: bytes per block, acknowledged one by one
#define FW_RESEND_MS    300     // block incomplete this long: bytes lost, ask for the block again
#define FW_RESENDS      5       // at most this many times in a row
#define FW_TIMEOUT_MS   3000    // no data for this long: the update is given up

#define UART_RX_PIN     16
#define UART_TX_PIN     17
#define UART_BAUD       460800  // 1.7: halved (display P4: GAMEPAD_BAUD), before 921600
#define SEND_INTERVAL   20      // ms
#define PAIR_BUTTON_PIN        0
#define LED_PIN         2
#define PAIR_HOLD_MS    3000
#define CLEAR_HOLD_MS   10000
#define PAIR_WINDOW_MS  60000
#define MAX_ALLOWED     8
#define LIGHT_INTERVAL  100     // ms, light bar update
#define BLINK_MS        400     // alarm blink half period
#define BATTERY_LOW     51      // Bluepad32 battery 0..255, 51 = 20 %, 0 = unknown
#define BATTERY_FLASH_EVERY 10000
#define BATTERY_FLASH_MS    300

static ControllerPtr pad = nullptr;
static Preferences prefs;

static uint8_t allowed[MAX_ALLOWED][6];
static uint8_t n_allowed = 0;
static bool pairing = false;
static uint32_t pairing_until = 0;

static int machine_state = 0;      // 0 idle, 1 run, 2 hold, 3 alarm, 4 jog, 5 homing, 6 other
static int jog_enabled = 0;
static uint32_t light_shown = 0xFFFFFFFF;   // last color sent, forces an update when changed
static uint32_t second_pulse_at = 0;        // job finished: time of the second rumble pulse, 0 = none

static uint32_t last_data = 0;      // millis() of the last new data from the controller
static bool streaming = false;      // the controller sends all the time (DualShock 4, DualSense)
static bool stalled = false;

static struct {
    bool on;
    size_t size, got, n;
    uint32_t crc, want, last;
    uint8_t resends;            // in a row
    uint16_t resends_all;
    uint8_t buf[FW_BLOCK];
} fw;

// ------------------------------------------------------------------------
// Allow list (stored in NVS)
// ------------------------------------------------------------------------

static void allow_load (void)
{
    n_allowed = prefs.getBytes("allowed", allowed, sizeof(allowed)) / 6;
}

static void allow_save (void)
{
    prefs.putBytes("allowed", allowed, n_allowed * 6);
}

static bool is_allowed (const uint8_t *addr)
{
    for(uint8_t i = 0; i < n_allowed; i++) {
        if(!memcmp(allowed[i], addr, 6))
            return true;
    }
    return false;
}

// a full list drops its oldest controller, a new one can always be paired
static void allow_add (const uint8_t *addr)
{
    if(!is_allowed(addr)) {
        if(n_allowed == MAX_ALLOWED) {
            memmove(allowed[0], allowed[1], (MAX_ALLOWED - 1) * 6);
            n_allowed--;
            Console.println("Allow list full: the oldest controller was removed");
        }
        memcpy(allowed[n_allowed++], addr, 6);
        allow_save();
    }
}

static void allow_clear (void)
{
    n_allowed = 0;
    prefs.remove("allowed");
    BP32.forgetBluetoothKeys();
    if(pad)
        pad->disconnect();
    Console.println("Allow list cleared");
}

static void pairing_start (uint32_t now)
{
    pairing = true;
    pairing_until = now + PAIR_WINDOW_MS;
    BP32.enableNewBluetoothConnections(true);
    Console.println("Pairing window open for 60 s");
}

static void pairing_stop (void)
{
    if(pairing) {
        pairing = false;
        BP32.enableNewBluetoothConnections(false);
        Console.println("Pairing window closed");
    }
}

// ------------------------------------------------------------------------
// Bluetooth log: $LG lines, from the BTstack thread over a queue to loop() (the only writer of the UART)
// ------------------------------------------------------------------------

#define LG_LEN  64

static QueueHandle_t lg_queue;
static btstack_packet_callback_registration_t lg_reg;

static void lg_push (const char *fmt, ...)
{
    char b[LG_LEN];
    int n = snprintf(b, sizeof(b), "LG,%lu,", (unsigned long)millis());
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b + n, sizeof(b) - n, fmt, ap);
    va_end(ap);
    if(lg_queue)
        xQueueSend(lg_queue, b, 0);
}

static void lg_handler (uint8_t type, uint16_t channel, uint8_t *p, uint16_t size)
{
    bd_addr_t a;

    if(type != HCI_EVENT_PACKET)
        return;

    switch(hci_event_packet_get_type(p)) {
        case BTSTACK_EVENT_STATE:
            lg_push("state,%u", btstack_event_state_get_state(p));
            break;
        case HCI_EVENT_CONNECTION_REQUEST:
            hci_event_connection_request_get_bd_addr(p, a);
            lg_push("req,%s", bd_addr_to_str(a));
            break;
        case HCI_EVENT_CONNECTION_COMPLETE:
            hci_event_connection_complete_get_bd_addr(p, a);
            lg_push("conn,%02X,%s", hci_event_connection_complete_get_status(p), bd_addr_to_str(a));
            break;
        case HCI_EVENT_LINK_KEY_REQUEST:
            lg_push("keyreq");
            break;
        case HCI_EVENT_PIN_CODE_REQUEST:
            lg_push("pin");
            break;
        case HCI_EVENT_USER_CONFIRMATION_REQUEST:
            lg_push("ssp");
            break;
        case HCI_EVENT_AUTHENTICATION_COMPLETE:
            lg_push("auth,%02X", hci_event_authentication_complete_get_status(p));
            break;
        case HCI_EVENT_ENCRYPTION_CHANGE:
            lg_push("enc,%02X,%u", hci_event_encryption_change_get_status(p), hci_event_encryption_change_get_encryption_enabled(p));
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            lg_push("disc,%02X,%02X", hci_event_disconnection_complete_get_status(p), hci_event_disconnection_complete_get_reason(p));
            break;
        default:
            break;
    }
}

static void lg_register (void *ctx)
{
    lg_reg.callback = &lg_handler;
    hci_add_event_handler(&lg_reg);
}

static void lg_begin (void)
{
    static btstack_context_callback_registration_t reg = { .callback = lg_register };

    lg_queue = xQueueCreate(16, LG_LEN);
    btstack_run_loop_execute_on_main_thread(&reg);
}

// ------------------------------------------------------------------------
// Bluepad32 callbacks
// ------------------------------------------------------------------------

static void on_connected (ControllerPtr ctl)
{
    const uint8_t *addr = ctl->getProperties().btaddr;

    if(pairing && !is_allowed(addr)) {
        allow_add(addr);
        Console.printf("New controller allowed: %s\n", ctl->getModelName().c_str());
    }

    if(!is_allowed(addr) || (pad != nullptr && pad != ctl)) {
        Console.printf("Controller rejected: %02X:%02X:%02X:%02X:%02X:%02X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
        lg_push("rejected,%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
        ctl->disconnect();
        return;
    }

    pad = ctl;
    pairing_stop();
    String model = ctl->getModelName();
    streaming = model.indexOf("DualShock") >= 0 || model.indexOf("DualSense") >= 0 || model.indexOf("PS4") >= 0 ||
                model.indexOf("PS5") >= 0;
    last_data = millis();
    stalled = false;
    Console.printf("Controller connected: %s%s\n", model.c_str(), streaming ? "" : " (no stall check)");
    lg_push("pad,%s", model.c_str());
    light_shown = 0xFFFFFFFF;   // set the light bar on the next update
    ctl->playDualRumble(0, 150, 0x60, 0x60);
}

static void on_disconnected (ControllerPtr ctl)
{
    if(ctl == pad) {
        pad = nullptr;
        Console.println("Controller disconnected");
        lg_push("gone");
    }
}

// ------------------------------------------------------------------------
// UART protocol
// ------------------------------------------------------------------------

static uint8_t checksum (const char *s)
{
    uint8_t c = 0;
    while(*s)
        c ^= (uint8_t)*s++;
    return c;
}

static void send_state (void)
{
    static uint8_t seq = 0;
    char body[96];

    uint32_t now = millis();
    unsigned pair = pairing ? (unsigned)((int32_t)(pairing_until - now) + 999) / 1000 : 0;

    // a stalled link counts as no controller: grblHAL stops at once
    bool live = pad && pad->isConnected() && !(streaming && now - last_data >= STALL_MS);
    if(pad && live == stalled) {
        stalled = !live;
        Console.println(stalled ? "Controller link stalled" : "Controller link back");
    }

    if(live)
        snprintf(body, sizeof(body), "GP,%u,1,%d,%d,%d,%d,%d,%d,%X,%X,%X,%u,%u", seq++,
                  pad->axisX(), pad->axisY(), pad->axisRX(), pad->axisRY(),
                   pad->brake(), pad->throttle(), pad->buttons(), pad->dpad(), pad->miscButtons(), pad->battery(), pair);
    else
        snprintf(body, sizeof(body), "GP,%u,0,0,0,0,0,0,0,0,0,0,0,%u", seq++, pair);

    Serial2.printf("$%s*%02X\n", body, checksum(body));
}

#define RGB(r, g, b) (((uint32_t)(r) << 16) | ((uint32_t)(g) << 8) | (uint32_t)(b))

// Light bar color for the machine state (DualShock 4 / DualSense)
static uint32_t state_color (uint32_t now)
{
    switch(machine_state) {
        case 0:  return jog_enabled ? RGB(50, 0, 60) : RGB(0, 0, 60);  // idle: violet enabled, blue locked
        case 1:  return RGB(0, 70, 0);                                 // job: green
        case 2:  return RGB(100, 40, 0);                               // hold: orange
        case 3:  return (now / BLINK_MS) & 1 ? RGB(50, 0, 60) : (jog_enabled ? RGB(100, 0, 0) : 0);  // alarm: violet blinking, violet/red with enable
        case 4:  return RGB(50, 0, 60);                                // jog: violet (enabled)
        case 5:  return RGB(100, 0, 0);                                // homing: red
        default: return RGB(30, 30, 30);                               // other: white
    }
}

static void update_light (uint32_t now)
{
    static uint32_t last = 0;

    if(!pad || now - last < LIGHT_INTERVAL)
        return;

    last = now;

    uint32_t color = state_color(now);
    uint8_t bat = pad->battery();

    if(bat && bat < BATTERY_LOW && now % BATTERY_FLASH_EVERY < BATTERY_FLASH_MS)
        color = RGB(100, 80, 0);    // low battery: yellow flash

    if(color != light_shown) {
        light_shown = color;
        pad->setColorLED(color >> 16, (color >> 8) & 0xFF, color & 0xFF);
        if(machine_state != 3)      // not while it blinks
            Console.printf("light: %u,%u,%u\n", (unsigned)(color >> 16), (unsigned)((color >> 8) & 0xFF), (unsigned)(color & 0xFF));
    }
}

static void show_event (int event, uint32_t now)
{
    if(!pad)
        return;

    switch(event) {
        case 1:     // alarm / limit: long and strong
            pad->playDualRumble(0, 700, 0xFF, 0xFF);
            break;
        case 2:     // job finished: two medium pulses
            pad->playDualRumble(0, 250, 0x90, 0x60);
            second_pulse_at = now + 450;
            break;
        case 3:     // acknowledge: short click
            pad->playDualRumble(0, 60, 0x50, 0x00);
            break;
    }
}

static void update_rumble (uint32_t now)
{
    if(second_pulse_at && (int32_t)(now - second_pulse_at) >= 0) {
        second_pulse_at = 0;
        if(pad)
            pad->playDualRumble(0, 250, 0x90, 0x60);
    }
}

static void send_line (const char *body)
{
    Serial2.printf("$%s*%02X\n", body, checksum(body));
}

// ------------------------------------------------------------------------
// Firmware update over the UART
// ------------------------------------------------------------------------

static void fw_fail (const char *why)
{
    Update.abort();
    fw.on = false;
    Console.printf("Firmware update failed: %s\n", why);
    char body[48];
    snprintf(body, sizeof(body), "FD,0,%s", why);
    for(char *p = body + 5; *p; p++)        // keep the line parsable
        if(*p == '*' || *p == '$' || *p < ' ')
            *p = ' ';
    send_line(body);
}

static void fw_ack (void)
{
    char body[24];
    snprintf(body, sizeof(body), "FK,%u", (unsigned)fw.got);
    send_line(body);
}

static void fw_start (unsigned long size, unsigned long crc)
{
    if(fw.on)
        Update.abort();
    memset(&fw, 0, sizeof(fw));

    if(size == 0 || !Update.begin(size)) {
        Console.printf("Firmware update refused: %s\n", size ? Update.errorString() : "size 0");
        send_line("FA,0");
        return;
    }
    fw.on = true;
    fw.size = size;
    fw.want = crc;
    fw.last = millis();
    Console.printf("Firmware update: %lu bytes\n", size);
    send_line("FA,1");
}

// the image in blocks: each block is written and acknowledged, then the board sends the next one
static void fw_poll (uint32_t now)
{
    while(fw.on && Serial2.available()) {
        size_t block = min((size_t)FW_BLOCK, fw.size - fw.got);
        size_t n = Serial2.read(fw.buf + fw.n, block - fw.n);
        if(n == 0)
            break;
        fw.n += n;
        fw.last = now;
        if(fw.n < block)
            continue;

        if(Update.write(fw.buf, fw.n) != fw.n) {
            fw_fail(Update.errorString());
            return;
        }
        fw.crc = esp_rom_crc32_le(fw.crc, fw.buf, fw.n);
        fw.got += fw.n;
        fw.n = 0;
        fw.resends = 0;
        fw_ack();

        if(fw.got == fw.size) {
            if(fw.crc != fw.want) {
                fw_fail("CRC");
                return;
            }
            if(!Update.end(true)) {         // checks the image, makes it the boot partition
                fw_fail(Update.errorString());
                return;
            }
            fw.on = false;
            char body[24];
            snprintf(body, sizeof(body), "FD,1,%u", (unsigned)fw.resends_all);
            send_line(body);
            Serial2.flush();
            Console.printf("Firmware update done (%u blocks sent again), restarting\n", (unsigned)fw.resends_all);
            delay(200);
            ESP.restart();
        }
    }

    // signed: fw.last may be a little newer than now (fw_start() and the reads use their own millis())
    int32_t quiet = (int32_t)(now - fw.last);
    if(fw.on && fw.n && quiet > FW_RESEND_MS) {     // block incomplete: bytes lost
        if(++fw.resends > FW_RESENDS) {
            fw_fail("bytes lost");
            return;
        }
        fw.resends_all++;
        Console.printf("Firmware update: block at %u incomplete (%u bytes), sent again\n",
                       (unsigned)fw.got, (unsigned)fw.n);
        while(Serial2.available())
            Serial2.read();
        fw.n = 0;
        fw.last = now;
        fw_ack();                                   // the last block written: the board repeats the next one
    }
    if(fw.on && quiet > FW_TIMEOUT_MS)
        fw_fail("timeout");
}

static bool radio_on = true;        // $PW, see radio_set()
static void radio_set (bool on);

// Parse "$ST,<state>,<event>,<enabled>*hh", "$PA,<cmd>*hh", "$PW,<on>*hh" and "$FW,<size>,<crc>*hh" lines from the board
static void receive (uint32_t now)
{
    static char line[48];
    static uint8_t len = 0;

    while(Serial2.available() && !fw.on) {      // during an update the bytes are the image (fw_poll)
        char c = Serial2.read();
        if(c == '$')
            len = 0;
        else if(c == '\n' || c == '\r') {
            line[len] = '\0';
            char *star = strchr(line, '*');
            int state, event, enabled, cmd;
            unsigned long size, crc;
            if(star && len > 3) {
                *star = '\0';
                if(strtoul(star + 1, NULL, 16) != checksum(line))
                    ;
                else if(sscanf(line, "FW,%lu,%lx", &size, &crc) == 2)
                    fw_start(size, crc);
                else if(sscanf(line, "ST,%d,%d,%d", &state, &event, &enabled) == 3) {
                    if(state != machine_state || enabled != jog_enabled)
                        Console.printf("ST: state %d, enabled %d\n", state, enabled);
                    machine_state = state;
                    jog_enabled = enabled;
                    show_event(event, now);
                } else if(sscanf(line, "PW,%d", &cmd) == 1)
                    radio_set(cmd != 0);
                else if(sscanf(line, "PA,%d", &cmd) == 1) {
                    if(cmd == 1 && radio_on)
                        pairing_start(now);
                    else {
                        pairing_stop();
                        if(cmd == 2)
                            allow_clear();
                    }
                }
            }
            len = 0;
        } else if(len < sizeof(line) - 1)
            line[len++] = c;
    }
}

// ------------------------------------------------------------------------
// BOOT button: pairing window / clear allow list
// ------------------------------------------------------------------------

static void poll_boot_button (uint32_t now)
{
    static uint32_t down_since = 0;
    static bool handled = false;

    if(digitalRead(PAIR_BUTTON_PIN) == LOW) {
        if(!down_since)
            down_since = now;
        if(!handled && now - down_since >= CLEAR_HOLD_MS) {
            allow_clear();
            handled = true;
        }
    } else {
        if(down_since && !handled && now - down_since >= PAIR_HOLD_MS && radio_on)
            pairing_start(now);
        down_since = 0;
        handled = false;
    }

    if(pairing && (int32_t)(now - pairing_until) >= 0)
        pairing_stop();

    // LED: blinking while pairing, on while a controller is connected
    digitalWrite(LED_PIN, pairing ? ((now / 250) & 1) : (pad != nullptr));
}

// ------------------------------------------------------------------------

void setup (void)
{
    pinMode(PAIR_BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);

    Serial2.setRxBufferSize(4096);      // a firmware block arrives while the previous one is written
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    // The receive interrupt comes by default only at 120 of the 128 FIFO bytes: 8 bytes = 87 us at 921600 (174 us at 460800)
    // baud, too little against Bluetooth, a firmware update lost bytes. 32 leaves about 1 ms.
    Serial2.setRxFIFOFull(32);

    prefs.begin("gamepad", false);
    allow_load();

    BP32.setup(&on_connected, &on_disconnected, false);   // no search for new controllers, see pairing_start()
    BP32.enableVirtualDevice(false);    // no touchpad mouse
    BP32.enableBLEService(false);
    lg_begin();

    // this firmware got up: keep it (without this a new firmware is rolled back at the next start)
    esp_ota_mark_app_valid_cancel_rollback();

    Console.printf("esp32_gamepad %s ready, %u allowed controller(s)\n", SKETCH_VERSION, n_allowed);
}

// Page scan (the receiver listens for a controller that reconnects): every 1.28 s (the default), interlaced
// (1.12): in each window it listens on both frequency trains a paging controller goes through one after
// the other, so a controller that pages on the other train is heard at once instead of some seconds later
// (1.11 log: slow connections lost their time before the request reached the receiver, the connection
// itself took 0.7 s). 1.8 to 1.11 scanned every 2.56 s, standard mode.
// Set in the BTstack thread once its HCI is up, asked again every second until then.
#define PAGE_SCAN_INTERVAL  0x0800  // slots of 0.625 ms: 1.28 s
#define PAGE_SCAN_WINDOW    0x12    // 11.25 ms, the default

static volatile bool page_scan_set = false;

// $PW: Bluetooth radio on/off (P4 display in the PC-Deck mode), power control in the BTstack thread
static volatile bool radio_want = true;

static void radio_apply (void *ctx)
{
    hci_power_control(radio_want ? HCI_POWER_ON : HCI_POWER_OFF);
}

static void radio_set (bool on)
{
    static btstack_context_callback_registration_t reg = { .callback = radio_apply };

    if(on == radio_on)
        return;
    radio_on = on;
    if(!on) {
        pairing_stop();
        if(pad)
            pad->disconnect();
    }
    radio_want = on;
    btstack_run_loop_execute_on_main_thread(&reg);
    if(on)
        page_scan_set = false;      // set the page scan again once the HCI is up
    setCpuFrequencyMhz(on ? 240 : 80);
    Console.printf("Bluetooth %s\n", on ? "on" : "off (PW,0)");
}

static void page_scan_apply (void *ctx)
{
    if(hci_get_state() == HCI_STATE_WORKING) {
        gap_set_page_scan_activity(PAGE_SCAN_INTERVAL, PAGE_SCAN_WINDOW);
        gap_set_page_scan_type(PAGE_SCAN_MODE_INTERLACED);
        page_scan_set = true;
    }
}

static void page_scan_poll (uint32_t now)
{
    static btstack_context_callback_registration_t reg = { .callback = page_scan_apply };
    static uint32_t last = 0;

    if(page_scan_set || !radio_on || now - last < 1000)
        return;
    last = now;
    btstack_run_loop_execute_on_main_thread(&reg);
}

void loop (void)
{
    static uint32_t last_send = 0;
    uint32_t now = millis();

    BP32.update();
    page_scan_poll(now);
    if(pad && pad->hasData())
        last_data = now;

    receive(now);
    if(fw.on) {             // firmware update: nothing else is sent meanwhile
        fw_poll(now);
        vTaskDelay(1);
        return;
    }
    char lg[LG_LEN];
    while(xQueueReceive(lg_queue, lg, 0) == pdTRUE)
        send_line(lg);

    poll_boot_button(now);
    update_light(now);
    update_rumble(now);

    if(now - last_send >= SEND_INTERVAL) {
        last_send = now;
        send_state();
    }

    vTaskDelay(1);
}
