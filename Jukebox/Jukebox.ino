/*
  ==========================================================================
   Narod's Jukebox - ESP32 Bluetooth A2DP speaker with ILI9341 touch UI
  ==========================================================================
   Board   : ESP32 Dev Module (ESP32-WROOM-32, no PSRAM)
   Core    : arduino-esp32 3.x  (uses the ESP_I2S / I2SClass driver)
   Scheme  : Huge APP (3MB No OTA/1MB SPIFFS)
   Libs    : TFT_eSPI (Bodmer)  - configured by the User_Setup.h in this repo
             ESP32-A2DP (pschatzmann) 1.8.x

   How the program is organised (more detail in README.md):

     Bluetooth tasks (library)          loop() on core 1 (Arduino task)
     -------------------------          ----------------------------------
     A2DP / AVRCP callbacks  --copy-->  g_shared  --snapshot--> g_snap
     (only copy data + bump version)    (spinlock)             |
                                                               v
                                       button, touch, UI state machine,
                                       all TFT drawing (dirty regions only)
                                                               |
     cmdTask  <--------- queue --------------------------------+
     (sends AVRCP play/pause/next/volume, which block ~100 ms, so they
      never run inside loop())

   Audio path: phone -> SBC decode -> 32 KB ring buffer (BluetoothA2DPSinkQueued)
   -> high-priority I2S task -> I2S (BCLK 26, WS 25, DIN 22) -> 2x MAX98357A.
   The ring buffer is what keeps audio smooth while the screen is redrawing.
  ==========================================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include "ESP_I2S.h"
#include "BluetoothA2DPSinkQueued.h"
#include "esp_sleep.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"

// ==========================================================================
//  USER SETTINGS - the only section you should need to touch
// ==========================================================================

#define DEVICE_NAME "Narod's Jukebox"

// 1 or 3 are the two landscape orientations. If the picture is upside
// down in the cabinet, switch to the other one: the touch calibration is
// saved together with the rotation, so a change triggers a new one.
#define SCREEN_ROTATION 3

// Pixels hidden behind the cabinet frame, per edge, as the viewer sees
// the screen (after rotation). The panel is about 5.6 px per mm, so the
// 2 mm strip is about 11 px; 12 leaves a little slack. Use the edge-ruler
// screen in calibration mode (see README) to measure the real values.
// Nothing important is ever drawn, and no touch target is placed, in
// these margins.
#define SAFE_LEFT   12
#define SAFE_RIGHT  0
#define SAFE_TOP    0
#define SAFE_BOTTOM 0

// Paste the 5 numbers printed by TFT_eSPI's Touch_calibrate example (or
// by this sketch's own calibration mode) here. All zeros = "not set".
// A calibration done on the device itself (saved in flash) takes
// priority over these numbers.
static const uint16_t TOUCH_CAL_DATA[5] = { 0, 0, 0, 0, 0 };

// 1 = touch screen used (on-screen buttons, calibration).
// 0 = no touch: no calibration at start-up, no on-screen buttons; the
//     player shows the track and volume in a bigger layout, and the
//     physical button + the phone do everything.
#define USE_TOUCH 0

// What to do with the panel before deep sleep.
//   1 = DISPOFF (0x28) + SLPIN (0x10), as specified. Lowest panel current,
//       but with the backlight hard-wired on, most ILI9341 panels then
//       show a WHITE screen while asleep.
//   0 = just paint the screen black and leave the controller running
//       (a few mA more, but the screen looks off).
#define PANEL_SLEEP_CMDS 1

// ==========================================================================
//  Pins (fixed by the PCB). TFT/touch SPI pins live in User_Setup.h.
// ==========================================================================
static const int PIN_I2S_BCLK  = 26;
static const int PIN_I2S_LRC   = 25;
static const int PIN_I2S_DOUT  = 22;
static const int PIN_BUTTON    = 33;  // active LOW, external 10k pull-up + 100nF
static const int PIN_TOUCH_IRQ = 27;  // XPT2046 PENIRQ, active LOW

// ==========================================================================
//  Tuning constants
// ==========================================================================
static const uint32_t BTN_DEBOUNCE_MS     = 25;    // RC does most of the work
static const uint32_t BTN_LONG_MS         = 2000;  // hold time for sleep
static const uint32_t BTN_MULTI_GAP_MS    = 400;   // max pause inside a double/triple press
static const uint32_t TOUCH_POLL_MS       = 15;
static const uint16_t TOUCH_Z_THRESHOLD   = 400;   // XPT2046 pressure threshold
static const uint32_t VOL_REPEAT_DELAY_MS = 500;   // hold +/- to repeat
static const uint32_t VOL_REPEAT_MS       = 180;
static const int      VOL_STEP            = 8;     // of 127 (~6 %)
static const int      VOL_DEFAULT         = 100;   // used if phone never reported one
static const uint32_t SCROLL_STEP_MS      = 40;    // title marquee speed: 2 px / 40 ms
static const uint32_t SCROLL_PAUSE_MS     = 2000;  // pause at the start of each pass
static const int      SCROLL_GAP_PX       = 48;    // gap between repeats of the title
static const uint32_t GOODBYE_MIN_MS      = 1500;
static const uint32_t PAUSE_BEFORE_DISC_MS = 400;
static const uint32_t DISCONNECT_TIMEOUT_MS = 3000;

// Colours (RGB565)
static const uint16_t COL_BG        = TFT_BLACK;
static const uint16_t COL_TEXT      = TFT_WHITE;
static const uint16_t COL_DIM       = 0x9CD3;  // light grey
static const uint16_t COL_FAINT     = 0x52AA;  // dark grey
static const uint16_t COL_ACCENT    = 0xFD20;  // jukebox orange
static const uint16_t COL_BTN       = 0x2945;  // button face
static const uint16_t COL_PLAYING   = 0x07E0;  // green
static const uint16_t COL_CONNECTED = 0x5D1F;  // light blue

// ==========================================================================
//  Types (all declared before any function, see note on prototypes below)
// ==========================================================================

enum PlayState : uint8_t { PS_UNKNOWN, PS_PLAYING, PS_PAUSED, PS_STOPPED };

// Everything the Bluetooth callbacks report. Written by BT tasks, read by
// loop(), always under g_mux. `ver` is bumped on every change so loop()
// can tell cheaply that something is new.
struct SharedState {
  uint32_t  ver;
  uint8_t   connState;   // esp_a2d_connection_state_t
  bool      connected;
  PlayState play;
  int16_t   volume;      // 0..127, -1 = not reported yet
  uint16_t  sampleRate;
  char      title[128];
  char      artist[96];
  char      album[96];
  char      peer[64];    // phone's Bluetooth name
};

struct Rect {
  int16_t x, y, w, h;
  bool contains(int px, int py, int slack = 0) const {
    return px >= x - slack && px < x + w + slack &&
           py >= y - slack && py < y + h + slack;
  }
};

enum ButtonId : int8_t { BTN_NONE = -1, BTN_PREV, BTN_PLAY, BTN_NEXT, BTN_VOLDN, BTN_VOLUP, BTN_COUNT };

enum Screen : uint8_t { SCR_NONE, SCR_WAITING, SCR_PLAYER, SCR_GOODBYE };

enum SleepPhase : uint8_t { SLEEP_NONE, SLEEP_PAUSING, SLEEP_DISCONNECTING, SLEEP_FINAL };

enum CmdType : uint8_t { CMD_PLAY, CMD_PAUSE, CMD_NEXT, CMD_PREV, CMD_SET_VOLUME, CMD_GO_OFFLINE };
struct Cmd { CmdType type; uint8_t value; };

struct Layout {
  Rect safe;
  Rect status, title, artist, album;
  Rect volBar;
  Rect hint;   // help line (no-touch layout only)
  Rect btn[BTN_COUNT];
};

// Dirty flags: which screen regions must be redrawn on the next pass
enum : uint16_t {
  D_STATUS = 1 << 0, D_TITLE = 1 << 1, D_ARTIST = 1 << 2, D_ALBUM = 1 << 3,
  D_VOLUME = 1 << 4, D_PLAYBTN = 1 << 5, D_ALL = 0xFFFF
};

// Explicit prototypes. The Arduino IDE generates these automatically, but
// it sometimes places them above the types they use; declaring them
// ourselves avoids that.
void onConnectionState(esp_a2d_connection_state_t state, void *obj);
void onAudioState(esp_a2d_audio_state_t state, void *obj);
void onPlayStatus(esp_avrc_playback_stat_t status);
void onMetadata(uint8_t id, const uint8_t *text);
void onRemoteVolume(int volume);
void onPeerName(char *name);
void onSampleRate(uint16_t rate);
void cmdTask(void *arg);
bool postCmd(CmdType type, uint8_t value);
bool loadTouchCal();
bool runTouchCalibration(bool canTimeOut);
void computeLayout();
void pollButton(uint32_t now);
void pollTouch(uint32_t now);
void updateUi(uint32_t now);
void runSleepSequence(uint32_t now);
void enterDeepSleep();

// ==========================================================================
//  Globals
// ==========================================================================

TFT_eSPI    tft;
TFT_eSprite titleSpr(&tft);   // one small sprite for the scrolling title (~16 KB)
bool        g_titleSprOk = false;

I2SClass i2s;
BluetoothA2DPSinkQueued a2dp_sink(i2s);

static SharedState  g_shared;                                  // BT tasks write
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static SharedState  g_snap;                                    // loop() reads
static uint32_t     g_snapVer = 0xFFFFFFFF;

static QueueHandle_t g_cmdQueue = nullptr;
static uint16_t      g_cal[5];
static bool          g_touchOk = false;  // false = no calibration: touch ignored, button still works
static Layout        L;

// ---- UI state (loop() only, no locking needed) ----
static Screen   g_screen = SCR_NONE;
static uint16_t g_dirty = D_ALL;
static char     g_shownTitle[128], g_shownArtist[96], g_shownAlbum[96], g_shownPeer[64];
static PlayState g_shownPlay = PS_UNKNOWN;
static int16_t  g_shownVol = -2;

static char     g_titleAscii[128];
static int16_t  g_titleW = 0;
static bool     g_titleScrolling = false;
static int16_t  g_scrollX = 0;
static uint32_t g_nextScrollAt = 0;

static uint8_t  g_waitDots = 0;
static uint32_t g_nextDotsAt = 0;

// ---- Button ----
static struct {
  bool     stable = HIGH, lastRaw = HIGH;
  uint32_t changedAt = 0, pressedAt = 0;
  bool     longFired = false, ignoreUntilRelease = false;
  uint8_t  clicks = 0;          // short presses in the current series
  uint32_t releasedAt = 0;
} g_btn;

// ---- Touch ----
static struct {
  uint32_t nextPoll = 0;
  uint8_t  downCount = 0, upCount = 0;
  bool     down = false;
  int16_t  x = 0, y = 0;
  int8_t   active = BTN_NONE;
  uint32_t nextRepeat = 0;
} g_touch;

// ---- Sleep ----
static SleepPhase g_sleep = SLEEP_NONE;
static uint32_t   g_sleepT0 = 0, g_phaseT0 = 0;

// ==========================================================================
//  Small helpers
// ==========================================================================

// Copy `src` into `dst` if it differs. Returns true if it changed.
static bool copyIfChanged(char *dst, size_t dstSize, const char *src) {
  if (strncmp(dst, src, dstSize - 1) == 0) return false;
  strlcpy(dst, src, dstSize);
  return true;
}

// The built-in TFT_eSPI fonts are 7-bit ASCII only. Song metadata is
// UTF-8, so map accented Latin letters and common punctuation to ASCII
// and replace anything else with '?'. (Non-Latin scripts - Arabic,
// Cyrillic, emoji - need a smooth font loaded from flash; see README.)
static void utf8ToAscii(const char *in, char *out, size_t outSize) {
  // Latin-1 U+00C0..U+00FF -> closest ASCII letter
  static const char latin1[] = "AAAAAAACEEEEIIIIDNOOOOOxOUUUUYTsaaaaaaaceeeeiiiidnooooo/ouuuuyty";
  size_t o = 0;
  const uint8_t *p = (const uint8_t *)in;
  while (*p && o + 1 < outSize) {
    uint32_t cp;
    uint8_t c = *p;
    if (c < 0x80)                     { cp = c; p += 1; }
    else if ((c & 0xE0) == 0xC0 && p[1]) { cp = ((c & 0x1F) << 6) | (p[1] & 0x3F); p += 2; }
    else if ((c & 0xF0) == 0xE0 && p[1] && p[2]) {
      cp = ((c & 0x0F) << 12) | ((p[1] & 0x3F) << 6) | (p[2] & 0x3F); p += 3;
    } else if ((c & 0xF8) == 0xF0 && p[1] && p[2] && p[3]) { p += 4; continue; }  // emoji etc: drop
    else { p += 1; continue; }                                                      // broken byte: drop

    const char *rep = nullptr;
    char one[2] = { 0, 0 };
    if (cp < 0x20)                         one[0] = ' ';
    else if (cp < 0x7F)                    one[0] = (char)cp;
    else if (cp == 0xA0)                   one[0] = ' ';
    else if (cp >= 0xC0 && cp <= 0xFF)     one[0] = latin1[cp - 0xC0];
    else if (cp == 0x2018 || cp == 0x2019 || cp == 0x201B || cp == 0x2032 || cp == 0xB4) one[0] = '\'';
    else if (cp == 0x201C || cp == 0x201D) one[0] = '"';
    else if (cp >= 0x2010 && cp <= 0x2015) one[0] = '-';
    else if (cp == 0x2026)                 rep = "...";
    else                                   one[0] = '?';
    if (!rep) rep = one;
    while (*rep && o + 1 < outSize) out[o++] = *rep++;
  }
  out[o] = '\0';
}

// Copy `in` to `out`, cutting it with "..." so it fits in maxW pixels.
static void fitText(const char *in, char *out, size_t outSize, int maxW, uint8_t font) {
  strlcpy(out, in, outSize);
  if (tft.textWidth(out, font) <= maxW) return;
  const int dotsW = tft.textWidth("...", font);
  size_t n = strlen(out);
  if (n > outSize - 4) n = outSize - 4;  // leave room for "..."
  out[n] = '\0';
  while (n > 0 && tft.textWidth(out, font) + dotsW > maxW) out[--n] = '\0';
  while (n > 0 && out[n - 1] == ' ') out[--n] = '\0';
  strcat(out, "...");
}

static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ==========================================================================
//  Bluetooth callbacks - run in Bluetooth tasks, NOT in loop().
//  Rule: copy the data into g_shared under the lock, bump `ver`, return.
//  No drawing, no Serial, no delays.
// ==========================================================================

void onConnectionState(esp_a2d_connection_state_t state, void *) {
  portENTER_CRITICAL(&g_mux);
  g_shared.connState = (uint8_t)state;
  bool c = (state == ESP_A2D_CONNECTION_STATE_CONNECTED);
  if (c != g_shared.connected) {
    g_shared.connected = c;
    // Fresh connection or disconnection: forget the old track
    g_shared.play = PS_UNKNOWN;
    g_shared.title[0] = g_shared.artist[0] = g_shared.album[0] = '\0';
    if (!c) g_shared.peer[0] = '\0';
  }
  g_shared.ver++;
  portEXIT_CRITICAL(&g_mux);
}

// The A2DP audio stream itself starting/stopping. Reliable on every phone,
// so it is the fallback for phones that don't send AVRCP play status.
void onAudioState(esp_a2d_audio_state_t state, void *) {
  PlayState ps = (state == ESP_A2D_AUDIO_STATE_STARTED) ? PS_PLAYING : PS_PAUSED;
  portENTER_CRITICAL(&g_mux);
  if (g_shared.connected && g_shared.play != ps) { g_shared.play = ps; g_shared.ver++; }
  portEXIT_CRITICAL(&g_mux);
}

// AVRCP play-status notification: reacts faster than the audio stream.
void onPlayStatus(esp_avrc_playback_stat_t status) {
  PlayState ps;
  switch (status) {
    case ESP_AVRC_PLAYBACK_PLAYING: ps = PS_PLAYING; break;
    case ESP_AVRC_PLAYBACK_PAUSED:  ps = PS_PAUSED;  break;
    case ESP_AVRC_PLAYBACK_STOPPED: ps = PS_STOPPED; break;
    default: return;  // seeking / error: keep the current state
  }
  portENTER_CRITICAL(&g_mux);
  if (g_shared.play != ps) { g_shared.play = ps; g_shared.ver++; }
  portEXIT_CRITICAL(&g_mux);
}

void onMetadata(uint8_t id, const uint8_t *text) {
  const char *t = text ? (const char *)text : "";
  portENTER_CRITICAL(&g_mux);
  bool changed = false;
  if (id == ESP_AVRC_MD_ATTR_TITLE)       changed = copyIfChanged(g_shared.title,  sizeof g_shared.title,  t);
  else if (id == ESP_AVRC_MD_ATTR_ARTIST) changed = copyIfChanged(g_shared.artist, sizeof g_shared.artist, t);
  else if (id == ESP_AVRC_MD_ATTR_ALBUM)  changed = copyIfChanged(g_shared.album,  sizeof g_shared.album,  t);
  if (changed) g_shared.ver++;
  portEXIT_CRITICAL(&g_mux);
}

// The phone moved its volume slider (AVRCP absolute volume, 0..127).
void onRemoteVolume(int volume) {
  portENTER_CRITICAL(&g_mux);
  g_shared.volume = (int16_t)clampi(volume, 0, 127);
  g_shared.ver++;
  portEXIT_CRITICAL(&g_mux);
}

void onPeerName(char *name) {
  portENTER_CRITICAL(&g_mux);
  if (name && copyIfChanged(g_shared.peer, sizeof g_shared.peer, name)) g_shared.ver++;
  portEXIT_CRITICAL(&g_mux);
}

// Called once per connection when the codec is negotiated (before audio
// flows). The one deliberate exception to "callbacks only set flags": the
// I2S clock must match the stream *before* the first samples arrive, and
// loop() can't guarantee that. Without this, a phone sending 48 kHz would
// play ~9 % slow and flat, because the library doesn't reconfigure an
// I2SClass output on its own.
void onSampleRate(uint16_t rate) {
  if (rate == 0 || rate == g_shared.sampleRate) return;
  i2s.configureTX(rate, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  portENTER_CRITICAL(&g_mux);
  g_shared.sampleRate = rate;
  g_shared.ver++;
  portEXIT_CRITICAL(&g_mux);
}

// ==========================================================================
//  Command task: the library's play()/pause()/next()/previous() send an
//  AVRCP "press", wait 100 ms, then send "release". That wait would freeze
//  loop(), so loop() just drops a command in a queue and this task does it.
// ==========================================================================

void cmdTask(void *) {
  Cmd c;
  for (;;) {
    if (xQueueReceive(g_cmdQueue, &c, portMAX_DELAY) != pdTRUE) continue;
    switch (c.type) {
      case CMD_PLAY:       a2dp_sink.play();     break;
      case CMD_PAUSE:      a2dp_sink.pause();    break;
      case CMD_NEXT:       a2dp_sink.next();     break;
      case CMD_PREV:       a2dp_sink.previous(); break;
      case CMD_SET_VOLUME: a2dp_sink.set_volume(c.value); break;  // applies locally + tells the phone
      case CMD_GO_OFFLINE:
        // Stop phones (re)connecting while we shut down, then drop the link.
        // NOTE: we deliberately do NOT call a2dp_sink.end(): it erases the
        // remembered phone from flash, which would kill auto-reconnect.
        a2dp_sink.set_connectable(false);
        if (a2dp_sink.is_connected()) a2dp_sink.disconnect();
        break;
    }
  }
}

bool postCmd(CmdType type, uint8_t value) {
  Cmd c{ type, value };
  return xQueueSend(g_cmdQueue, &c, 0) == pdTRUE;  // never block loop()
}

// ==========================================================================
//  Touch: calibration storage, raw -> screen mapping, calibration mode
// ==========================================================================
//  g_cal uses exactly TFT_eSPI's 5-value format, so numbers from the
//  Touch_calibrate example and from our own calibration are interchangeable:
//    [0] raw value at screen x = 0     [1] raw span across the full width
//    [2] raw value at screen y = 0     [3] raw span across the full height
//    [4] bit0 = axes swapped, bit1 = invert x, bit2 = invert y

static bool calLooksValid(const uint16_t *c) {
  return c[1] > 500 && c[3] > 500 && c[0] < 4096 && c[2] < 4096 && c[4] < 8;
}

bool loadTouchCal() {
  Preferences prefs;
  if (prefs.begin("jukebox", true)) {
    uint16_t c[5];
    // A calibration only holds for the rotation it was made in
    bool ok = prefs.getBytes("tcal", c, sizeof c) == sizeof c && calLooksValid(c) &&
              prefs.getUChar("trot", 0xFF) == SCREEN_ROTATION;
    prefs.end();
    if (ok) { memcpy(g_cal, c, sizeof c); Serial.println("[touch] using calibration saved on device"); return true; }
  }
  if (calLooksValid(TOUCH_CAL_DATA)) {
    memcpy(g_cal, TOUCH_CAL_DATA, sizeof g_cal);
    Serial.println("[touch] using TOUCH_CAL_DATA from the sketch");
    return true;
  }
  return false;
}

static void saveTouchCal() {
  Preferences prefs;
  if (prefs.begin("jukebox", false)) {
    prefs.putBytes("tcal", g_cal, sizeof g_cal);
    prefs.putUChar("trot", SCREEN_ROTATION);
    prefs.end();
  }
}

// Same maths as TFT_eSPI::convertRawXY(), but signed and clamped, so a
// touch slightly outside the calibrated range can't wrap around.
static void rawToScreen(uint16_t rx, uint16_t ry, int16_t &sx, int16_t &sy) {
  const int W = tft.width(), H = tft.height();
  int ax = (g_cal[4] & 1) ? ry : rx;
  int ay = (g_cal[4] & 1) ? rx : ry;
  int xx = (int)((long)(ax - g_cal[0]) * W / g_cal[1]);
  int yy = (int)((long)(ay - g_cal[2]) * H / g_cal[3]);
  if (g_cal[4] & 2) xx = W - xx;
  if (g_cal[4] & 4) yy = H - yy;
  sx = clampi(xx, 0, W - 1);
  sy = clampi(yy, 0, H - 1);
}

static bool touchPressedRaw() { return tft.getTouchRawZ() > TOUCH_Z_THRESHOLD; }

// The physical button can always get out of calibration, so a broken or
// disconnected touch panel never stops the jukebox from starting.
// True once per new press (a button already held when calibration starts
// doesn't count until it has been released).
static bool g_calBtnWasDown = false;
static bool calButtonClicked() {
  bool down = digitalRead(PIN_BUTTON) == LOW;
  bool clicked = down && !g_calBtnWasDown;
  g_calBtnWasDown = down;
  return clicked;
}

// Returns false if the button was pressed instead.
static bool waitTouchRelease() {
  uint8_t up = 0;
  while (up < 5) {
    if (calButtonClicked()) return false;
    up = touchPressedRaw() ? 0 : up + 1;
    delay(10);
  }
  return true;
}

// Reads one XPT2046 value without going through TFT_eSPI, at 1 MHz.
// cmd 0xD0 = X, 0x90 = Y, 0xB0 = pressure Z1. Result 0..4095.
static uint16_t directTouchRead(uint8_t cmd) {
  SPIClass &spi = tft.getSPIinstance();
  spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  // TFT_eSPI leaves the SPI unit in write-only mode after drawing:
  // switch MISO sampling back on (what TFT_eSPI does before a touch read)
  *(volatile uint32_t *)(SPI_USER_REG(SPI_PORT)) = SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN;
  digitalWrite(TOUCH_CS, LOW);
  spi.transfer(cmd);
  uint16_t v = spi.transfer16(0) >> 3;
  digitalWrite(TOUCH_CS, HIGH);
  spi.endTransaction();
  return v & 0x0FFF;
}

// Live readings from the touch controller, drawn on the screen: the only
// way to see them once the screen covers the USB port. Shown on the edge
// ruler, with a weak pull-down on MISO (see showEdgeRuler).
//   line 1 "lib": through TFT_eSPI. Not touched: Z=0, IRQ high. Touched:
//                 Z of several hundred, IRQ LOW, X/Y ~200..3900 that
//                 follow the finger.
//   line 2 "own": same chip read by this sketch, slower, 0..4095.
// If both stay at the maximum (8191 / 4095): MISO isn't being read at all.
// If both drop to 0 with the pull-down: nothing drives MISO, the touch
// chip doesn't answer (T_CS / T_CLK / T_DIN / T_DO side).
static void drawTouchReadout(int y) {
  uint16_t z = tft.getTouchRawZ();
  uint16_t rx = 0, ry = 0;
  tft.getTouchRaw(&rx, &ry);
  char s[56];
  snprintf(s, sizeof s, "lib Z=%u X=%u Y=%u IRQ=%s", z, rx, ry,
           digitalRead(PIN_TOUCH_IRQ) == LOW ? "LOW" : "high");
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(COL_ACCENT, COL_BG);
  tft.setTextPadding(260);  // stays clear of the ruler frames
  tft.drawString(s, tft.width() / 2, y, 2);
  snprintf(s, sizeof s, "own X=%u Y=%u Z1=%u", directTouchRead(0xD0), directTouchRead(0x90), directTouchRead(0xB0));
  tft.setTextColor(COL_CONNECTED, COL_BG);
  tft.drawString(s, tft.width() / 2, y + 18, 2);
  tft.setTextPadding(0);
}

static void drawTarget(int x, int y, uint16_t col) {
  tft.drawFastHLine(x - 12, y, 25, col);
  tft.drawFastVLine(x, y - 12, 25, col);
  tft.drawCircle(x, y, 7, col);
}

// Waits for a steady press on the target and returns the averaged raw value.
// Returns false if the button was pressed instead (calibration skipped).
static bool sampleTarget(int x, int y, uint16_t &outX, uint16_t &outY) {
  drawTarget(x, y, TFT_WHITE);
  if (!waitTouchRelease()) return false;
  uint32_t sx = 0, sy = 0;
  int n = 0;
  while (n < 16) {
    uint16_t rx, ry;
    if (calButtonClicked()) return false;
    if (touchPressedRaw()) {
      if (n == 0) delay(60);  // let the pressure settle before the first sample
      tft.getTouchRaw(&rx, &ry);
      if (!touchPressedRaw()) { n = 0; sx = sy = 0; continue; }  // lifted mid-sample
      sx += rx; sy += ry; n++;
      delay(8);
    } else {
      n = 0; sx = sy = 0;
      delay(10);
    }
  }
  drawTarget(x, y, TFT_GREEN);
  if (!waitTouchRelease()) return false;
  drawTarget(x, y, COL_BG);
  outX = sx / 16;
  outY = sy / 16;
  return true;
}

// Four targets at (xl,yt) (xl,yb) (xr,yt) (xr,yb) - well inside the
// visible area - then extrapolate linearly to the screen edges so the
// result is in TFT_eSPI's format. This is why the hidden strip doesn't
// matter: nobody has to touch the very edge.
static bool computeCal(const uint16_t *rx, const uint16_t *ry,
                       int xl, int xr, int yt, int yb, uint16_t *out) {
  const float W = tft.width(), H = tft.height();
  // Moving from target 0 to target 1 only changes screen Y. If raw X
  // changed more than raw Y, the touch axes are swapped vs the display.
  bool swapXY = abs((int)rx[0] - (int)rx[1]) > abs((int)ry[0] - (int)ry[1]);
  float a[4], b[4];  // a = raw axis that follows screen X, b = follows screen Y
  for (int i = 0; i < 4; i++) { a[i] = swapXY ? ry[i] : rx[i]; b[i] = swapXY ? rx[i] : ry[i]; }

  float aL = (a[0] + a[1]) / 2, aR = (a[2] + a[3]) / 2;  // left / right column
  float bT = (b[0] + b[2]) / 2, bB = (b[1] + b[3]) / 2;  // top / bottom row
  float kx = (aR - aL) / (float)(xr - xl);                // raw units per pixel
  float ky = (bB - bT) / (float)(yb - yt);
  float a0 = aL - kx * xl, aW = aR + kx * (W - xr);       // raw at x = 0 and x = W
  float b0 = bT - ky * yt, bH = bB + ky * (H - yb);       // raw at y = 0 and y = H

  bool invX = a0 > aW, invY = b0 > bH;
  if (invX) { float t = a0; a0 = aW; aW = t; }
  if (invY) { float t = b0; b0 = bH; bH = t; }

  out[0] = (uint16_t)clampi((int)(a0 + 0.5f), 1, 4095);
  out[1] = (uint16_t)clampi((int)(aW - a0 + 0.5f), 1, 4095);
  out[2] = (uint16_t)clampi((int)(b0 + 0.5f), 1, 4095);
  out[3] = (uint16_t)clampi((int)(bH - b0 + 0.5f), 1, 4095);
  out[4] = (swapXY ? 1 : 0) | (invX ? 2 : 0) | (invY ? 4 : 0);
  return out[1] > 1500 && out[3] > 1500;  // XPT2046 spans ~3300 raw counts
}

// Concentric frames 4 px apart: the outermost colour you can fully see on
// an edge tells you that edge's SAFE_ value. Also shows the live touch
// readings. Returns true on a tap (go on with calibration), false if the
// button was pressed or, when canTimeOut, after 20 s without a tap.
static bool showEdgeRuler(bool canTimeOut) {
  static const uint16_t cols[] = { TFT_RED, TFT_ORANGE, TFT_YELLOW, TFT_GREEN, TFT_CYAN, TFT_BLUE, TFT_MAGENTA };
  static const char *names[]   = { "red", "orange", "yellow", "green", "cyan", "blue", "magenta" };
  const int W = tft.width(), H = tft.height();
  tft.fillScreen(COL_BG);
  for (int i = 0; i < 7; i++) tft.drawRect(i * 4, i * 4, W - i * 8, H - i * 8, cols[i]);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.drawString("EDGE RULER", W / 2, 32, 4);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.drawString("Outermost line visible on each edge:", W / 2, 60, 2);
  char line[40];
  for (int i = 0; i < 7; i++) {  // two columns: 4 on the left, 3 on the right
    snprintf(line, sizeof line, "%s = %d px", names[i], i * 4);
    tft.setTextColor(cols[i], COL_BG);
    tft.drawString(line, W / 2 + (i < 4 ? -64 : 64), 80 + (i % 4) * 16, 2);
  }
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.drawString("Tap = continue   Button = skip", W / 2, 148, 2);

  // Weak pull-down on MISO while this screen is up: an undriven line then
  // reads 0 instead of floating (diagnostic only, removed on exit).
  gpio_pulldown_en((gpio_num_t)TFT_MISO);
  g_calBtnWasDown = digitalRead(PIN_BUTTON) == LOW;
  const uint32_t t0 = millis();
  uint32_t nextDraw = 0;
  bool seenRelease = false;  // a panel stuck at "pressed" must not count as a tap
  bool tapped = false;
  for (;;) {
    uint32_t now = millis();
    if ((int32_t)(now - nextDraw) >= 0) { nextDraw = now + 150; drawTouchReadout(168); }
    if (!touchPressedRaw()) seenRelease = true;
    else if (seenRelease) { tapped = waitTouchRelease(); break; }
    if (calButtonClicked()) break;
    if (canTimeOut && now - t0 > 20000) break;
    delay(10);
  }
  gpio_pulldown_dis((gpio_num_t)TFT_MISO);
  return tapped;
}

// Blocking on purpose: it only runs from setup(), before Bluetooth starts.
// canTimeOut: true when it started by itself (no calibration yet), so an
// unattended jukebox with a dead touch panel still starts after 20 s.
// Returns true if a new calibration was made and saved.
bool runTouchCalibration(bool canTimeOut) {
  Serial.println("[touch] calibration mode");
  if (!showEdgeRuler(canTimeOut)) {
    Serial.println("[touch] calibration skipped");
    return false;
  }

  const int W = tft.width(), H = tft.height();
  const int INSET = 30;
  const int xl = SAFE_LEFT + INSET, xr = W - 1 - SAFE_RIGHT - INSET;
  const int yt = SAFE_TOP + INSET,  yb = H - 1 - SAFE_BOTTOM - INSET;
  const int tx[4] = { xl, xl, xr, xr }, ty[4] = { yt, yb, yt, yb };

  for (;;) {
    tft.fillScreen(COL_BG);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(COL_TEXT, COL_BG);
    tft.drawString("Touch each + and hold", W / 2, H / 2 - 10, 2);
    tft.drawString("until it turns green", W / 2, H / 2 + 10, 2);
    tft.setTextColor(COL_FAINT, COL_BG);
    tft.drawString("Button = skip", W / 2, H / 2 + 34, 2);
    uint16_t rx[4], ry[4];
    for (int i = 0; i < 4; i++) {
      if (!sampleTarget(tx[i], ty[i], rx[i], ry[i])) {
        Serial.println("[touch] calibration skipped");
        return false;
      }
    }
    uint16_t c[5];
    if (computeCal(rx, ry, xl, xr, yt, yb, c)) { memcpy(g_cal, c, sizeof c); break; }
    tft.fillScreen(COL_BG);
    tft.setTextColor(TFT_RED, COL_BG);
    tft.drawString("That didn't look right - again", W / 2, H / 2, 2);
    delay(1500);
  }
  saveTouchCal();

  Serial.println();
  Serial.println("// Touch calibration (also saved on the device).");
  Serial.println("// Optional: paste into TOUCH_CAL_DATA in Jukebox.ino:");
  Serial.printf("static const uint16_t TOUCH_CAL_DATA[5] = { %u, %u, %u, %u, %u };\n\n",
                g_cal[0], g_cal[1], g_cal[2], g_cal[3], g_cal[4]);

  // Test screen: draw a dot wherever the firmware thinks you touched.
  tft.fillScreen(COL_BG);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.drawString("Draw to test. Press the button when done.", W / 2, SAFE_TOP + 8, 2);
  tft.drawRect(SAFE_LEFT, SAFE_TOP, W - SAFE_LEFT - SAFE_RIGHT, H - SAFE_TOP - SAFE_BOTTOM, COL_FAINT);
  while (digitalRead(PIN_BUTTON) == LOW) delay(10);  // release the boot hold first
  delay(50);
  while (digitalRead(PIN_BUTTON) == HIGH) {
    if (touchPressedRaw()) {
      uint16_t rx, ry; int16_t sx, sy;
      tft.getTouchRaw(&rx, &ry);
      if (touchPressedRaw()) { rawToScreen(rx, ry, sx, sy); tft.fillCircle(sx, sy, 2, COL_ACCENT); }
    }
    delay(5);
  }
  while (digitalRead(PIN_BUTTON) == LOW) delay(10);
  return true;
}

// ==========================================================================
//  Layout - everything is placed inside the safe area
// ==========================================================================

void computeLayout() {
  const int W = tft.width(), H = tft.height();
  L.safe = { SAFE_LEFT, SAFE_TOP, (int16_t)(W - SAFE_LEFT - SAFE_RIGHT), (int16_t)(H - SAFE_TOP - SAFE_BOTTOM) };
  const int PAD = 6;
  const int x = L.safe.x + PAD, w = L.safe.w - 2 * PAD, y = L.safe.y;

#if !USE_TOUCH
  // No buttons to draw: the same regions, spread over the whole screen
  L.status = { (int16_t)x, (int16_t)(y + 10),  (int16_t)w, 18 };
  L.title  = { (int16_t)x, (int16_t)(y + 46),  (int16_t)w, 28 };
  L.artist = { (int16_t)x, (int16_t)(y + 84),  (int16_t)w, 18 };
  L.album  = { (int16_t)x, (int16_t)(y + 104), (int16_t)w, 18 };
  L.volBar = { (int16_t)x, (int16_t)(y + 128), (int16_t)w, 36 };
  L.hint   = { (int16_t)x, (int16_t)(L.safe.y + L.safe.h - 44), (int16_t)w, 36 };
  return;
#endif

  L.status = { (int16_t)x, (int16_t)(y + 4),  (int16_t)w, 18 };
  L.title  = { (int16_t)x, (int16_t)(y + 28), (int16_t)w, 28 };
  L.artist = { (int16_t)x, (int16_t)(y + 60), (int16_t)w, 18 };
  L.album  = { (int16_t)x, (int16_t)(y + 80), (int16_t)w, 18 };

  // Volume row:  [ - ]  [====bar==== 63%]  [ + ]
  const int vy = y + 104, vh = 40, vbw = 60;
  L.btn[BTN_VOLDN] = { (int16_t)x, (int16_t)vy, (int16_t)vbw, (int16_t)vh };
  L.btn[BTN_VOLUP] = { (int16_t)(x + w - vbw), (int16_t)vy, (int16_t)vbw, (int16_t)vh };
  L.volBar = { (int16_t)(x + vbw + 10), (int16_t)vy, (int16_t)(w - 2 * vbw - 20), (int16_t)vh };

  // Transport row: three big buttons filling the rest
  const int ty = vy + vh + 8;
  const int th = max(40, (int)(L.safe.y + L.safe.h - PAD - ty));
  const int gap = 8, bw = (w - 2 * gap) / 3;
  L.btn[BTN_PREV] = { (int16_t)x,                    (int16_t)ty, (int16_t)bw, (int16_t)th };
  L.btn[BTN_PLAY] = { (int16_t)(x + bw + gap),       (int16_t)ty, (int16_t)bw, (int16_t)th };
  L.btn[BTN_NEXT] = { (int16_t)(x + 2 * (bw + gap)), (int16_t)ty, (int16_t)(w - 2 * (bw + gap)), (int16_t)th };
}

// ==========================================================================
//  Drawing
// ==========================================================================

static void drawIcon(ButtonId id, int cx, int cy, int s, uint16_t col) {
  const int h = s / 2;
  switch (id) {
    case BTN_PLAY:
      if (g_shownPlay == PS_PLAYING) {  // show "pause" while playing
        tft.fillRect(cx - h, cy - h, s / 3, s, col);
        tft.fillRect(cx + h - s / 3, cy - h, s / 3, s, col);
      } else {
        tft.fillTriangle(cx - h + 3, cy - h, cx - h + 3, cy + h, cx + h + 3, cy, col);
      }
      break;
    case BTN_NEXT:
      tft.fillTriangle(cx - h, cy - h, cx - h, cy + h, cx + h / 2, cy, col);
      tft.fillRect(cx + h / 2, cy - h, s / 5, s, col);
      break;
    case BTN_PREV:
      tft.fillTriangle(cx + h, cy - h, cx + h, cy + h, cx - h / 2, cy, col);
      tft.fillRect(cx - h / 2 - s / 5, cy - h, s / 5, s, col);
      break;
    case BTN_VOLDN:
      tft.fillRect(cx - h, cy - 2, s, 5, col);
      break;
    case BTN_VOLUP:
      tft.fillRect(cx - h, cy - 2, s, 5, col);
      tft.fillRect(cx - 2, cy - h, 5, s, col);
      break;
    default: break;
  }
}

static void drawButton(ButtonId id, bool pressed) {
  const Rect &r = L.btn[id];
  tft.fillRoundRect(r.x, r.y, r.w, r.h, 10, pressed ? COL_ACCENT : COL_BTN);
  int s = min(r.w, r.h) * 2 / 5;
  if (id == BTN_VOLDN || id == BTN_VOLUP) s = min(r.w, r.h) / 2;
  drawIcon(id, r.x + r.w / 2, r.y + r.h / 2, s, pressed ? COL_BG : COL_TEXT);
}

static void drawStatus() {
  const Rect &r = L.status;
  tft.fillRect(r.x, r.y, r.w, r.h, COL_BG);
  const char *label;
  uint16_t col;
  switch (g_shownPlay) {
    case PS_PLAYING: label = "Playing"; col = COL_PLAYING; break;
    case PS_PAUSED:  label = "Paused";  col = COL_ACCENT;  break;
    case PS_STOPPED: label = "Stopped"; col = COL_ACCENT;  break;
    default:         label = "Connected"; col = COL_CONNECTED; break;
  }
  const int cy = r.y + r.h / 2;
  tft.fillCircle(r.x + 5, cy, 5, col);
  tft.setTextDatum(ML_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(col, COL_BG);
  tft.drawString(label, r.x + 16, cy, 2);
  if (g_shownPeer[0]) {
    int lx = r.x + 16 + tft.textWidth(label, 2) + 8;
    char ascii[64], fit[64];
    utf8ToAscii(g_shownPeer, ascii, sizeof ascii);
    fitText(ascii, fit, sizeof fit, r.x + r.w - lx, 2);
    tft.setTextColor(COL_FAINT, COL_BG);
    tft.drawString(fit, lx, cy, 2);
  }
}

static void drawTextLine(const Rect &r, const char *utf8, const char *placeholder, uint16_t col) {
  char ascii[128], fit[128];
  utf8ToAscii(utf8, ascii, sizeof ascii);
  if (!ascii[0] && placeholder) { strlcpy(ascii, placeholder, sizeof ascii); col = COL_FAINT; }
  fitText(ascii, fit, sizeof fit, r.w, 2);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(col, COL_BG);
  tft.setTextPadding(r.w);  // paints the rest of the line, so no clear/flicker
  tft.drawString(fit, r.x, r.y + 1, 2);
  tft.setTextPadding(0);
}

// One frame of the title: either static, or a marquee frame in the sprite.
static void drawTitleFrame() {
  const Rect &r = L.title;
  if (g_titleScrolling) {
    titleSpr.fillSprite(COL_BG);
    titleSpr.setTextDatum(TL_DATUM);
    titleSpr.setTextColor(COL_TEXT, COL_BG);
    titleSpr.drawString(g_titleAscii, -g_scrollX, 1, 4);
    titleSpr.drawString(g_titleAscii, -g_scrollX + g_titleW + SCROLL_GAP_PX, 1, 4);
    titleSpr.pushSprite(r.x, r.y);
    return;
  }
  char fit[128];
  const char *txt = g_titleAscii;
  uint16_t col = COL_TEXT;
  if (!txt[0]) { txt = "No track info yet"; col = COL_FAINT; }
  fitText(txt, fit, sizeof fit, r.w, 4);  // only actually cuts when no sprite
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(col, COL_BG);
  tft.setTextPadding(r.w);
  tft.drawString(fit, r.x, r.y + 1, 4);
  tft.setTextPadding(0);
}

static void setTitle(uint32_t now) {
  utf8ToAscii(g_shownTitle, g_titleAscii, sizeof g_titleAscii);
  g_titleW = tft.textWidth(g_titleAscii, 4);
  g_titleScrolling = g_titleSprOk && g_titleW > L.title.w;
  g_scrollX = 0;
  g_nextScrollAt = now + SCROLL_PAUSE_MS;
  drawTitleFrame();
}

static void drawVolume() {
  const Rect &r = L.volBar;
  const int textW = 44;
  const int bx = r.x, bw = r.w - textW, bh = 14, by = r.y + (r.h - bh) / 2;
  tft.drawRoundRect(bx, by, bw, bh, 4, COL_FAINT);
  int v = g_shownVol;
  int fill = v < 0 ? 0 : (bw - 4) * v / 127;
  tft.fillRect(bx + 2, by + 2, fill, bh - 4, COL_ACCENT);
  tft.fillRect(bx + 2 + fill, by + 2, bw - 4 - fill, bh - 4, COL_BG);
  char txt[8];
  if (v < 0) strcpy(txt, "--");
  else snprintf(txt, sizeof txt, "%d%%", (v * 100 + 63) / 127);
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.setTextPadding(textW - 4);
  tft.drawString(txt, r.x + r.w, r.y + r.h / 2, 2);
  tft.setTextPadding(0);
}

static void drawWaitingStatic() {
  const Rect &s = L.safe;
  const int cx = s.x + s.w / 2;
  tft.fillScreen(COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COL_FAINT, COL_BG);
  tft.drawString("BLUETOOTH JUKEBOX", cx, s.y + 22, 2);
  tft.setTextColor(COL_TEXT, COL_BG);
  if (tft.textWidth("Waiting for connection", 4) <= s.w - 8) {
    tft.drawString("Waiting for connection", cx, s.y + 66, 4);
  } else {
    tft.drawString("Waiting for", cx, s.y + 54, 4);
    tft.drawString("connection", cx, s.y + 80, 4);
  }
  tft.setTextColor(COL_DIM, COL_BG);
  tft.drawString("On your phone, open Bluetooth and pick:", cx, s.y + 136, 2);
  tft.setTextColor(COL_ACCENT, COL_BG);
  tft.drawString(DEVICE_NAME, cx, s.y + 164, 4);
  tft.setTextColor(COL_FAINT, COL_BG);
  tft.drawString("Hold the button 2 s to sleep", cx, s.y + s.h - 14, 2);
}

static void drawWaitingDots() {
  const int cx = L.safe.x + L.safe.w / 2, cy = L.safe.y + 106;
  for (int i = 0; i < 3; i++) tft.fillCircle(cx - 20 + i * 20, cy, 5, i == g_waitDots ? COL_ACCENT : COL_FAINT);
}

static void drawGoodbye() {
  const Rect &s = L.safe;
  const int cx = s.x + s.w / 2, cy = s.y + s.h / 2;
  tft.fillScreen(COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COL_ACCENT, COL_BG);
  tft.drawString("Goodbye!", cx, cy - 14, 4);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.drawString("Press the button to wake me up", cx, cy + 18, 2);
}

static void enterScreen(Screen s) {
  g_screen = s;
  g_touch.active = BTN_NONE;
  if (s == SCR_WAITING) {
    drawWaitingStatic();
    g_waitDots = 0;
    g_nextDotsAt = 0;
  } else if (s == SCR_PLAYER) {
    tft.fillScreen(COL_BG);
#if USE_TOUCH
    for (int i = 0; i < BTN_COUNT; i++) drawButton((ButtonId)i, false);
#else
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(COL_FAINT, COL_BG);
    tft.drawString("Press: play / pause     2 presses: next", L.hint.x + L.hint.w / 2, L.hint.y, 2);
    tft.drawString("3 presses: previous     Hold 2 s: sleep", L.hint.x + L.hint.w / 2, L.hint.y + 18, 2);
#endif
    g_dirty = D_ALL;
  }
}

// Called every loop(): take a snapshot of what the BT side reported,
// work out which regions changed, and redraw only those.
void updateUi(uint32_t now) {
  if (g_shared.ver != g_snapVer) {  // 32-bit read is atomic; copy under lock
    portENTER_CRITICAL(&g_mux);
    memcpy(&g_snap, &g_shared, sizeof g_snap);
    portEXIT_CRITICAL(&g_mux);
    g_snapVer = g_snap.ver;
  }

  Screen want = g_snap.connected ? SCR_PLAYER : SCR_WAITING;
  if (want != g_screen) {
    Serial.printf("[bt] %s\n", g_snap.connected ? "connected" : "waiting for connection");
    enterScreen(want);
  }

  if (g_screen == SCR_WAITING) {
    if ((int32_t)(now - g_nextDotsAt) >= 0) {
      g_nextDotsAt = now + 450;
      drawWaitingDots();
      g_waitDots = (g_waitDots + 1) % 3;
    }
    return;
  }

  // --- Player screen: compare snapshot with what's on screen ---
  if (copyIfChanged(g_shownTitle,  sizeof g_shownTitle,  g_snap.title))  g_dirty |= D_TITLE;
  if (copyIfChanged(g_shownArtist, sizeof g_shownArtist, g_snap.artist)) g_dirty |= D_ARTIST;
  if (copyIfChanged(g_shownAlbum,  sizeof g_shownAlbum,  g_snap.album))  g_dirty |= D_ALBUM;
  if (copyIfChanged(g_shownPeer,   sizeof g_shownPeer,   g_snap.peer))   g_dirty |= D_STATUS;
  if (g_snap.play != g_shownPlay)  { g_shownPlay = g_snap.play; g_dirty |= D_STATUS | D_PLAYBTN; }
  if (g_snap.volume != g_shownVol) { g_shownVol = g_snap.volume; g_dirty |= D_VOLUME; }

  // At most one region per pass keeps each loop() iteration short.
  if (g_dirty & D_STATUS)       { g_dirty &= ~D_STATUS;  drawStatus(); }
  else if (g_dirty & D_TITLE)   { g_dirty &= ~D_TITLE;   setTitle(now); }
  else if (g_dirty & D_ARTIST)  { g_dirty &= ~D_ARTIST;  drawTextLine(L.artist, g_shownArtist, "", COL_DIM); }
  else if (g_dirty & D_ALBUM)   { g_dirty &= ~D_ALBUM;   drawTextLine(L.album,  g_shownAlbum,  "", COL_FAINT); }
  else if (g_dirty & D_VOLUME)  { g_dirty &= ~D_VOLUME;  drawVolume(); }
  else if (g_dirty & D_PLAYBTN) { g_dirty &= ~D_PLAYBTN; if (USE_TOUCH) drawButton(BTN_PLAY, g_touch.active == BTN_PLAY); }
  else if (g_titleScrolling && (int32_t)(now - g_nextScrollAt) >= 0) {
    g_scrollX += 2;
    g_nextScrollAt = now + SCROLL_STEP_MS;
    if (g_scrollX >= g_titleW + SCROLL_GAP_PX) {  // one full pass: rest at the start
      g_scrollX = 0;
      g_nextScrollAt = now + SCROLL_PAUSE_MS;
    }
    drawTitleFrame();
  }
}

// ==========================================================================
//  Input: touch
// ==========================================================================

static void changeVolume(int delta) {
  int v = g_snap.volume >= 0 ? g_snap.volume : VOL_DEFAULT;
  v = clampi(v + delta, 0, 127);
  postCmd(CMD_SET_VOLUME, (uint8_t)v);
  // Show it immediately; the library doesn't echo local changes back.
  portENTER_CRITICAL(&g_mux);
  g_shared.volume = v;
  g_shared.ver++;
  portEXIT_CRITICAL(&g_mux);
  g_snap.volume = v;
}

static void onButtonAction(ButtonId id) {
  switch (id) {
    case BTN_PREV:  postCmd(CMD_PREV, 0); break;
    case BTN_NEXT:  postCmd(CMD_NEXT, 0); break;
    case BTN_PLAY:  postCmd(g_snap.play == PS_PLAYING ? CMD_PAUSE : CMD_PLAY, 0); break;
    case BTN_VOLDN: changeVolume(-VOL_STEP); break;
    case BTN_VOLUP: changeVolume(+VOL_STEP); break;
    default: break;
  }
}

// Non-blocking: one quick SPI read every 15 ms, and only when the XPT2046
// pulls T_IRQ low. Two matching samples = press, two empty = release.
void pollTouch(uint32_t now) {
  if (!g_touchOk) return;
  if ((int32_t)(now - g_touch.nextPoll) < 0) return;
  g_touch.nextPoll = now + TOUCH_POLL_MS;

  bool pressed = false;
  int16_t x = 0, y = 0;
  if (digitalRead(PIN_TOUCH_IRQ) == LOW && tft.getTouchRawZ() > TOUCH_Z_THRESHOLD) {
    uint16_t rx, ry;
    tft.getTouchRaw(&rx, &ry);
    rawToScreen(rx, ry, x, y);
    pressed = true;
  }

  if (pressed) {
    g_touch.upCount = 0;
    if (!g_touch.down) {
      bool near = abs(x - g_touch.x) < 25 && abs(y - g_touch.y) < 25;
      g_touch.downCount = (g_touch.downCount > 0 && near) ? g_touch.downCount + 1 : 1;
      g_touch.x = x; g_touch.y = y;
      if (g_touch.downCount >= 2) {  // confirmed press
        g_touch.down = true;
        if (g_screen == SCR_PLAYER) {
          for (int i = 0; i < BTN_COUNT; i++) {
            if (L.btn[i].contains(x, y, 4)) {
              g_touch.active = i;
              drawButton((ButtonId)i, true);
              onButtonAction((ButtonId)i);  // act on press: feels instant
              g_touch.nextRepeat = now + VOL_REPEAT_DELAY_MS;
              break;
            }
          }
        }
      }
    } else if ((g_touch.active == BTN_VOLUP || g_touch.active == BTN_VOLDN) &&
               (int32_t)(now - g_touch.nextRepeat) >= 0) {
      g_touch.nextRepeat = now + VOL_REPEAT_MS;  // auto-repeat while held
      onButtonAction((ButtonId)g_touch.active);
    }
  } else {
    g_touch.downCount = 0;
    if (g_touch.down && ++g_touch.upCount >= 2) {  // confirmed release
      g_touch.down = false;
      if (g_touch.active != BTN_NONE && g_screen == SCR_PLAYER) drawButton((ButtonId)g_touch.active, false);
      g_touch.active = BTN_NONE;
    }
  }
}

// ==========================================================================
//  Input: physical button (GPIO 33)
// ==========================================================================

// 1 press = play/pause, 2 = next track, 3 or more = previous track
static void onClicks(uint8_t n) {
  if (!g_snap.connected) return;
  if (n == 1)      postCmd(g_snap.play == PS_PLAYING ? CMD_PAUSE : CMD_PLAY, 0);
  else if (n == 2) postCmd(CMD_NEXT, 0);
  else             postCmd(CMD_PREV, 0);
}

static void onLongPress(uint32_t now) {
  Serial.println("[power] going to sleep");
  g_screen = SCR_GOODBYE;
  drawGoodbye();
  g_sleepT0 = g_phaseT0 = now;
  if (g_snap.connected && g_snap.play == PS_PLAYING) {
    postCmd(CMD_PAUSE, 0);  // let the phone pause instead of playing on its own speaker
    g_sleep = SLEEP_PAUSING;
  } else {
    postCmd(CMD_GO_OFFLINE, 0);
    g_sleep = SLEEP_DISCONNECTING;
  }
}

void pollButton(uint32_t now) {
  bool raw = digitalRead(PIN_BUTTON);
  if (raw != g_btn.lastRaw) { g_btn.lastRaw = raw; g_btn.changedAt = now; }

  if (raw != g_btn.stable && now - g_btn.changedAt >= BTN_DEBOUNCE_MS) {
    g_btn.stable = raw;
    if (raw == LOW) {
      g_btn.pressedAt = now;
      g_btn.longFired = false;
    } else if (g_btn.ignoreUntilRelease) {
      g_btn.ignoreUntilRelease = false;  // that was the press that woke us
    } else if (!g_btn.longFired && g_sleep == SLEEP_NONE) {
      g_btn.clicks++;  // counted now, acted on once the series is over
      g_btn.releasedAt = now;
    }
  }

  // The series is over once the button has stayed up for BTN_MULTI_GAP_MS
  // (this is why a single press acts ~0.4 s after release)
  if (g_btn.clicks && g_btn.stable == HIGH && now - g_btn.releasedAt >= BTN_MULTI_GAP_MS) {
    onClicks(g_btn.clicks);
    g_btn.clicks = 0;
  }

  if (g_btn.stable == LOW && !g_btn.longFired && !g_btn.ignoreUntilRelease &&
      g_sleep == SLEEP_NONE && now - g_btn.pressedAt >= BTN_LONG_MS) {
    g_btn.longFired = true;
    g_btn.clicks = 0;  // press, press, hold = sleep, not next track
    onLongPress(now);
  }
}

// ==========================================================================
//  Sleep / wake
// ==========================================================================

void runSleepSequence(uint32_t now) {
  switch (g_sleep) {
    case SLEEP_PAUSING:
      if (now - g_phaseT0 >= PAUSE_BEFORE_DISC_MS) {
        postCmd(CMD_GO_OFFLINE, 0);
        g_sleep = SLEEP_DISCONNECTING;
        g_phaseT0 = now;
      }
      break;
    case SLEEP_DISCONNECTING: {
      portENTER_CRITICAL(&g_mux);
      uint8_t cs = g_shared.connState;
      portEXIT_CRITICAL(&g_mux);
      if (cs == ESP_A2D_CONNECTION_STATE_DISCONNECTED || now - g_phaseT0 >= DISCONNECT_TIMEOUT_MS) {
        g_sleep = SLEEP_FINAL;
      }
      break;
    }
    case SLEEP_FINAL:
      // ext0 wakes on a LOW level: sleeping while the button is still held
      // would wake us instantly, so wait for the release.
      if (g_btn.stable == HIGH && now - g_sleepT0 >= GOODBYE_MIN_MS) enterDeepSleep();
      break;
    default: break;
  }
}

void enterDeepSleep() {
  Serial.println("[power] deep sleep");
  Serial.flush();

#if PANEL_SLEEP_CMDS
  tft.fillScreen(COL_BG);
  tft.writecommand(0x28);  // DISPOFF
  delay(20);
  tft.writecommand(0x10);  // SLPIN: stops the panel's oscillator and charge pump
  delay(120);              // datasheet: wait before anything else
#else
  tft.fillScreen(COL_BG);
#endif

  // Radio off before sleeping, as ESP-IDF recommends.
  esp_bluedroid_disable();
  esp_bt_controller_disable();

  // Keep both chip-selects high and the panel out of reset while the
  // ESP32's pins would otherwise float.
  gpio_hold_en((gpio_num_t)TFT_CS);
  gpio_hold_en((gpio_num_t)TOUCH_CS);
  gpio_hold_en((gpio_num_t)TFT_RST);
  gpio_deep_sleep_hold_en();

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0);  // wake when the button pulls LOW
  esp_deep_sleep_start();                         // waking = full reboot into setup()
}

// ==========================================================================
//  setup() and loop()
// ==========================================================================

void setup() {
  Serial.begin(115200);
  // GPIO 2 (TFT_DC) and 5 (TFT_CS) are strapping pins, but the chip reads
  // them at reset, before setup() runs. From here on they are ordinary
  // outputs, so nothing below can affect the boot mode.

  const bool wokeByButton = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0;

  // Undo what enterDeepSleep() did to the pins.
  rtc_gpio_deinit(GPIO_NUM_33);
  gpio_hold_dis((gpio_num_t)TFT_CS);
  gpio_hold_dis((gpio_num_t)TOUCH_CS);
  gpio_hold_dis((gpio_num_t)TFT_RST);
  gpio_deep_sleep_hold_dis();

  pinMode(PIN_BUTTON, INPUT);           // external 10k pull-up
  pinMode(PIN_TOUCH_IRQ, INPUT_PULLUP);
  g_btn.stable = g_btn.lastRaw = digitalRead(PIN_BUTTON);
  // If the button is down now, it's the wake press (or the calibration
  // hold); don't treat its release as a play/pause press.
  g_btn.ignoreUntilRelease = (g_btn.stable == LOW);

  memset(&g_shared, 0, sizeof g_shared);
  g_shared.volume = -1;

  tft.init();
  tft.setRotation(SCREEN_ROTATION);
  tft.fillScreen(COL_BG);
  computeLayout();

  // Touch calibration: on a cold boot with the button held, or if there
  // is no calibration at all yet.
  // Skipping (button) keeps any older calibration; with none at all the
  // jukebox still starts, touch is just ignored.
  bool haveCal = USE_TOUCH && loadTouchCal();
  const bool forced = !wokeByButton && g_btn.stable == LOW;
  if (USE_TOUCH && (!haveCal || forced)) {
    if (runTouchCalibration(!forced)) haveCal = true;
    g_btn.stable = g_btn.lastRaw = digitalRead(PIN_BUTTON);
    g_btn.ignoreUntilRelease = (g_btn.stable == LOW);  // the skip press must not count
  }
#if !USE_TOUCH
  // Touch off: holding the button at power-on still shows the edge ruler
  // and the touch readings (for testing); the button leaves it.
  if (forced) {
    showEdgeRuler(false);
    g_btn.stable = g_btn.lastRaw = digitalRead(PIN_BUTTON);
    g_btn.ignoreUntilRelease = (g_btn.stable == LOW);
  }
#endif
  g_touchOk = haveCal;
  if (g_touchOk) Serial.printf("[touch] cal = { %u, %u, %u, %u, %u }\n", g_cal[0], g_cal[1], g_cal[2], g_cal[3], g_cal[4]);
  else Serial.println("[touch] no calibration: touch disabled");

  // Splash while Bluetooth starts
  tft.fillScreen(COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COL_ACCENT, COL_BG);
  tft.drawString(DEVICE_NAME, L.safe.x + L.safe.w / 2, L.safe.y + L.safe.h / 2, 4);

  // AVRCP commands go through their own task (see cmdTask)
  g_cmdQueue = xQueueCreate(8, sizeof(Cmd));
  xTaskCreatePinnedToCore(cmdTask, "avrc_cmd", 4096, nullptr, 2, nullptr, 1);

  // I2S: plain stereo; the SD-pin resistors pick L/R on each amp
  i2s.setPins(PIN_I2S_BCLK, PIN_I2S_LRC, PIN_I2S_DOUT);
  if (!i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    Serial.println("[audio] I2S init FAILED");
  }

  // Register every callback before start()
  a2dp_sink.set_on_connection_state_changed(onConnectionState);
  a2dp_sink.set_on_audio_state_changed(onAudioState);
  a2dp_sink.set_avrc_rn_playstatus_callback(onPlayStatus);
  a2dp_sink.set_avrc_metadata_attribute_mask(ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST | ESP_AVRC_MD_ATTR_ALBUM);
  a2dp_sink.set_avrc_metadata_callback(onMetadata);
  a2dp_sink.set_avrc_rn_volumechange(onRemoteVolume);
  a2dp_sink.set_peer_name_callback(onPeerName);
  a2dp_sink.set_sample_rate_callback(onSampleRate);
  g_shared.sampleRate = 44100;
  // true = auto-reconnect to the last phone (the library's default is OFF)
  a2dp_sink.start(DEVICE_NAME, true);

  // The one sprite: title strip, 16-bit, ~16 KB. Without it, long titles
  // are cut with "..." instead of scrolling.
  titleSpr.setColorDepth(16);
  g_titleSprOk = titleSpr.createSprite(L.title.w, L.title.h) != nullptr;
  Serial.printf("[mem] free heap %u bytes, title sprite %s\n", (unsigned)ESP.getFreeHeap(), g_titleSprOk ? "ok" : "FAILED (truncating)");
}

void loop() {
  const uint32_t now = millis();
  pollButton(now);

  if (g_sleep != SLEEP_NONE) {  // Goodbye screen: just run the shutdown steps
    runSleepSequence(now);
    return;
  }

  pollTouch(now);
  updateUi(now);

  static uint16_t loggedRate = 0;
  if (g_snap.sampleRate != loggedRate) {
    loggedRate = g_snap.sampleRate;
    Serial.printf("[audio] sample rate %u Hz\n", loggedRate);
  }
}
