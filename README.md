# Narod's Jukebox: ESP32 firmware

A Bluetooth speaker (A2DP sink) with a 2.8" ILI9341 touch screen, two MAX98357A amps and one power/play button.

```
Jukebox/Jukebox.ino          the firmware
TFT_eSPI_Setup/User_Setup.h  display/touch config for the TFT_eSPI library
```

## 1. Build setup

| Setting | Value |
|---|---|
| Board package | **esp32 by Espressif, 3.x** (Boards Manager) |
| Board | ESP32 Dev Module |
| Partition scheme | Huge APP (3MB No OTA/1MB SPIFFS) |
| Flash mode / freq | DIO / 40 MHz |
| Upload speed | 115200 |

Libraries:

* **TFT_eSPI** by Bodmer (Library Manager). Use 2.5.43 or newer. Older versions fail to compile on core 3.x.
* **ESP32-A2DP** by pschatzmann. Download the ZIP from GitHub and use *Sketch → Include Library → Add .ZIP Library*. I checked the API against v1.8.11.

### Where User_Setup.h goes

TFT_eSPI takes its settings from a file inside the library itself, not from your sketch. Copy `TFT_eSPI_Setup/User_Setup.h` over the one that ships with the library:

| OS | Path |
|---|---|
| Windows | `Documents\Arduino\libraries\TFT_eSPI\User_Setup.h` |
| macOS | `~/Documents/Arduino/libraries/TFT_eSPI/User_Setup.h` |
| Linux | `~/Arduino/libraries/TFT_eSPI/User_Setup.h` |

Leave `User_Setup_Select.h` as it is (it includes `User_Setup.h` by default). **Updating TFT_eSPI overwrites this file**, so copy it back after every library update.

Open `Jukebox/Jukebox.ino` (the folder name has to match the sketch name) and upload.

## 2. Touch calibration and the hidden 2 mm strip

The cabinet frame covers about 2 mm of the screen, which is about 11 px (the panel has about 5.6 px per mm). This matters for calibration in two ways.

**Problem with the stock example:** TFT_eSPI's `Touch_calibrate` puts its arrows in the four very corners, and it assumes you touched exactly pixel 0 / pixel 319. If two of those corners are under the frame, you either can't reach them or you press about 11 px too far in. Every touch on that axis then comes out shifted.

Calibration describes the touch glass, not the cabinet, so you have two ways around it:

### Option A: built-in calibration (recommended now that the screen is mounted)

The firmware has its own calibration mode. Its targets sit 30 px inside the visible area, and it extrapolates out to the edges, so nobody has to touch the frame.

1. Power off. **Hold the button and power on** (keep holding until the edge ruler appears).
   The mode also starts on its own when no calibration exists yet (first start, or after changing `SCREEN_ROTATION`). Started that way, it gives up after 20 s without a tap, so the jukebox always starts.
   It doesn't start when you wake the jukebox from sleep with the button.
   **Pressing the button skips calibration** at any step. The jukebox then runs with the previous calibration, or with touch disabled if there is none (music, play/pause and sleep still work with the button and the phone).
2. **Edge ruler:** concentric coloured frames, 4 px apart. On each edge, note the outermost colour you can fully see. For example, if on the left you can see green (12 px) but not yellow, use `SAFE_LEFT 12`. Tap to continue.
   The orange line under it shows the touch controller's live readings: not touched = `Z=0`, `IRQ=high`; pressed = `Z` of several hundred, `IRQ=LOW`, and `X`/`Y` that change as the finger moves.
3. Touch each **+** and hold it until it turns green (4 targets).
4. **Test screen:** draw with your finger. The dots should land under your finger, including near the frame. Press the button to finish.

The result is saved in flash and survives reflashing. It is also printed on Serial Monitor (115200), so you can paste it as a backup:

```
static const uint16_t TOUCH_CAL_DATA[5] = { 312, 3401, 289, 3488, 7 };
```

If the edge ruler gave different margins than the defaults, edit `SAFE_LEFT/RIGHT/TOP/BOTTOM` at the top of `Jukebox.ino` and upload again. The layout moves itself inside those margins. Then recalibrate (hold the button at power-on) so the targets also sit inside the new margins.

### Option B: TFT_eSPI's Touch_calibrate example

Use this if you can reach all four corners, for example with the screen out of the cabinet.

1. *File → Examples → TFT_eSPI → Generic → Touch_calibrate*.
2. In that example, make sure `tft.setRotation(1);` matches `SCREEN_ROTATION` in `Jukebox.ino` (default 1). Calibration only holds for one rotation.
3. The example saves its result to SPIFFS and skips calibration the next time. Set `#define REPEAT_CAL true` to force it to run again.
4. Upload, open Serial Monitor at 115200, and touch the arrow in each corner.
5. It prints `uint16_t calData[5] = { a, b, c, d, e };`. Paste the five numbers into **`TOUCH_CAL_DATA`** near the top of `Jukebox.ino`:
   ```cpp
   static const uint16_t TOUCH_CAL_DATA[5] = { a, b, c, d, e };
   ```

Priority: a calibration saved on the device (option A) wins over `TOUCH_CAL_DATA`. To throw away a saved calibration, upload once with *Tools → Erase All Flash Before Sketch Upload → Enabled*. That also makes the jukebox forget the paired phone.

## 3. Test plan (in this order)

Keep Serial Monitor open at 115200 for every step.

1. **Screen.** Power on. You should see the name splash, then "Waiting for connection" with three animated dots.
   * Check that nothing important is cut off at the edges. If it is, run the edge ruler (section 2) and adjust `SAFE_*`.
   * If the image is upside down, switch `SCREEN_ROTATION` between 1 and 3. The next start asks for a new calibration by itself.
2. **Touch.** Calibrate (section 2). On the test screen, draw along all four edges and the centre. The dots must follow your finger.
3. **Bluetooth pairing.** On the phone, pair with "Narod's Jukebox".
   * The screen switches to the player and shows "Connected" plus the phone's name.
   * Start music: the status shows **Playing** (green), and title, artist and album appear.
   * Pick a song with a long title: it scrolls and doesn't spill over.
   * Test ⏮ ⏯ ⏭ and the − / + buttons on the screen. The phone's volume slider should move with − / +, and the bar should move when you use the phone's slider.
   * Turn the jukebox off and on again: it reconnects to the phone by itself.
   * Serial shows `[audio] sample rate 44100 Hz` (or 48000).
4. **Audio left/right check.** Play a left/right channel test track (search "left right stereo test" on YouTube or Spotify).
   * "Left" must come only from the left speaker and "right" only from the right.
   * If they're swapped, the SD-pin resistors are on the opposite amps. Fix it in firmware by adding `a2dp_sink.set_swap_lr_channels(true);` before `a2dp_sink.start(...)`.
   * Turn the volume up to maximum with a bass-heavy track to check that the boost converter holds (see "crackling" below).
5. **Button, sleep and wake.**
   * A short press toggles play/pause.
   * Hold for 2 s: "Goodbye!" appears, the phone pauses, the link drops, and once you let go the panel goes off.
   * Press again: it boots, reconnects to the phone, and that wake press does **not** toggle play/pause.
   * With a multimeter in series with the battery, compare current while asleep and while awake (see the note on sleep current below).

## 4. Common failures

| Symptom | Likely meaning |
|---|---|
| **White screen**, nothing drawn | The panel is powered but gets no valid SPI. Either the wrong `User_Setup.h` is in use (check that the Serial output and `USER_SETUP_INFO` match, and that the file is in the *library* folder), or there's a bad CS/DC/RST/SCK/MOSI joint. Also try `SPI_FREQUENCY 27000000`. |
| White screen **only while asleep** | Expected with `PANEL_SLEEP_CMDS 1`. The backlight is hard-wired on, and an ILI9341 in display-off/sleep shows white. Set `PANEL_SLEEP_CMDS 0` to keep it black instead (a few mA more). |
| Red and blue swapped / colours inverted | Uncomment `TFT_RGB_ORDER TFT_BGR` / `TFT_INVERSION_ON` in `User_Setup.h`. |
| **Touch mirrored** (left↔right or up↔down) | Calibration was done in a different rotation than `SCREEN_ROTATION`, or the stock example ran with a different `setRotation`. Recalibrate. |
| **Touch offset** a fixed amount on one side | Corners were pressed inside the frame with the stock example. Use the built-in calibration. |
| Touch does nothing / jumps randomly | Check the live readings on the edge ruler (hold the button at power-on). `Z` stuck at 0 while pressing = the touch chip doesn't answer: check T_CLK→18, T_CS→21, T_DIN→23, T_DO→19, and that nothing else drives MISO. `IRQ` never goes LOW = T_IRQ→27. Raw readings are garbage: Usually the display's MISO (SDO) doesn't release the shared bus. The firmware never reads from the display, so lifting the display module's SDO pin from the MISO line fixes it. Also check TOUCH_CS (21) and T_IRQ (27). |
| **One speaker silent** | That amp gets no audio or no power. Check its 5 V and GND, and BCLK/LRC/DIN at that amp. Check its SD resistor: SD pulled fully to GND = shutdown. A wrong value can also put both amps on the same channel, so the L/R test plays from both. |
| Both speakers play the same thing | Both SD resistors select the same channel (or both select "(L+R)/2"). Check the 220k and 470k values. |
| **Crackling** / distortion that grows with volume | The 5 V boost sags: two amps at 9 dB gain can pull >1 A peaks. Check with lower volume. If it's clean at low volume, it's power (thicker wires, bulk capacitor near the amps, weak/old cells). |
| Crackle / dropouts at **any** volume | Bluetooth radio issues (distance, metal cabinet, antenna end of the module against metal), or I2S wiring (loose BCLK/LRC). Speed or pitch wrong = sample-rate mismatch (Serial shows the rate). |
| **Doesn't wake** from sleep | GPIO 33 isn't pulled LOW by the button, or it's stuck LOW (it would wake instantly, so the symptom is "reboots right away"). Measure: 3.3 V released, 0 V pressed. If it wakes then reboots in a loop, check that the board gets power in sleep (the boost converter must stay on). |
| Brown-out reset when music starts | `Brownout detector was triggered` on Serial means the supply dips when the radio and amps start. Same fix as crackling: supply side. |
| Upload fails "Wrong boot mode" | Hold BOOT while uploading. GPIO 2 (TFT_DC) is a strapping pin, and on some boards the display pulls it during reset. |

**Sleep current, honestly:** the ESP32 itself drops to about 10 µA, but the backlight is wired straight to 3V3, the amps and boost converter stay powered, and the DevKit's regulator and USB chip draw a few mA. "Sleep" here mostly means "quiet and off-looking". Expect a few tens of mA, so a few days on a charge, not months. The only way to get real "off" is a power switch in the battery line.

## 5. How the firmware works

### Two worlds, one rule

Bluetooth runs in **its own FreeRTOS tasks**. When something happens (connection, track change, volume), the library calls our callbacks *from those tasks*. Drawing on the TFT from there would be a race: loop() could be halfway through drawing over the same SPI bus.

So the rule is:

* **Callbacks** (`onMetadata`, `onPlayStatus`, `onRemoteVolume` …) only copy the data into one struct, `g_shared`, increment `g_shared.ver`, and return. The copy runs inside `portENTER_CRITICAL(&g_mux)`, a spinlock that works across both ESP32 cores. Copying a 128-byte string takes microseconds, so the lock is never held long enough to hurt audio.
* **loop()** notices that `ver` changed, takes a **snapshot** (`g_snap`) under the same lock, and then works only with its private copy. It compares the snapshot with what is currently on screen (`g_shownTitle`, `g_shownPlay` …) and sets **dirty flags** only for what differs. "Redraw only when changed" falls out of this naturally.

The one exception is `onSampleRate`. It reconfigures the I2S clock directly, because that must happen before the first audio sample, and loop() can't guarantee that timing.

### No blocking in loop()

* **Button:** read every pass. A level counts only when it has been stable for 25 ms (the RC network does most of the debouncing). Released before 2 s = play/pause, reached 2 s = sleep.
* **Touch:** at most every 15 ms, and only when the XPT2046 pulls **T_IRQ** low, so idle loops never touch the SPI bus. Two matching samples = press, two empty samples = release. Buttons act on press, so they feel instant, and − / + auto-repeat while held.
* **AVRCP commands:** the library's `play()`/`next()` contain a 100 ms wait (press… release). loop() only puts a command in a FreeRTOS queue. A small `cmdTask` sends it.
* **Sleep:** a small state machine (`SLEEP_PAUSING → DISCONNECTING → FINAL`) checks timers instead of using `delay()`.

### Drawing without a framebuffer

A full 320×240 framebuffer would take 150 KB, which this chip can't spare next to Bluetooth. Instead:

* Each region (status line, title, artist, album, volume, play button) has its own draw function. Only dirty regions get redrawn, **one region per loop pass**, so no single pass hogs the CPU.
* Text is drawn with a background colour plus `setTextPadding`, which overwrites the old text in one go, without a flickering "clear then draw".
* The only sprite is a 296×28 strip (~16 KB) for the **scrolling title**. If the sprite can't be allocated, titles are cut with "…" instead.
* All coordinates come from `computeLayout()`, which builds the layout inside the `SAFE_*` margins. Change a margin and the whole UI moves.

### Why audio doesn't stutter

`BluetoothA2DPSinkQueued` puts decoded audio into a **32 KB ring buffer** (~185 ms). A separate high-priority I2S task drains it. loop() runs at the lowest priority, so the I2S task simply pre-empts it whenever it needs to, even in the middle of a redraw. Every draw is short anyway (the biggest, a full-screen clear, takes ~30 ms and happens only when the screen changes).

### Volume: AVRCP absolute volume

The jukebox is the **AVRCP volume target**: there is one volume number (0–127), applied digitally on the ESP32, and both sides keep it in sync.

* Phone slider → the phone sends absolute volume → `onRemoteVolume` → the bar updates.
* Screen − / + → `set_volume()` scales the audio *and* notifies the phone, so its slider moves too.

The alternatives are worse. "Local only" volume multiplies with the phone's volume (two volumes fighting, and you can end up with a quiet speaker at full phone volume). Sending VOL_UP/DOWN key presses is ignored by many phones. With absolute volume, iPhones and modern Androids send full-scale audio and let the speaker scale it.

### Sleep and wake

On a 2 s hold: show Goodbye → pause the phone → stop accepting connections and disconnect → wait for the disconnect (max 3 s) **and** for the button to be released → panel `0x28` + `0x10` → radio off → hold CS/RST pins → deep sleep with ext0 wake on GPIO 33 LOW.

Two details:

* We never call `a2dp_sink.end()`: it erases the remembered phone from flash, and auto-reconnect would stop working.
* We wait for the button release because ext0 wakes on a *level*. Sleeping while the button is still held would wake the chip immediately.

Waking is a full reboot. `setup()` sees the wake cause, ignores the wake press, and the library reconnects to the last phone.

### Strapping pins

GPIO 2 (TFT_DC) and 5 (TFT_CS) are read by the chip **at reset, before `setup()` runs**. After that they are ordinary pins, so no code in `setup()` can change the boot mode. The only risk is electrical: something pulling GPIO 2 high, or GPIO 5 low, while the chip is resetting.

### Known limitation

The built-in fonts are ASCII. Accented Latin letters are mapped (é → e, ñ → n), but Arabic, Cyrillic, CJK and emoji show as `?`. If Narod's music is mostly in a non-Latin script, this can be fixed by loading a smooth font from SPIFFS. Ask and it can be added.
