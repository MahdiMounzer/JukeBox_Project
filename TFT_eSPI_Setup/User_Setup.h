// =====================================================================
//  TFT_eSPI User_Setup.h for Narod's Jukebox
//  ESP32-WROOM-32 DevKit + 2.8" ILI9341 240x320 + XPT2046 touch
//
//  Copy this file over:
//    <Arduino sketchbook>/libraries/TFT_eSPI/User_Setup.h
//  (Windows: Documents\Arduino\libraries\TFT_eSPI\User_Setup.h
//   macOS:   ~/Documents/Arduino/libraries/TFT_eSPI/User_Setup.h
//   Linux:   ~/Arduino/libraries/TFT_eSPI/User_Setup.h)
//  Updating TFT_eSPI overwrites this file - copy it back afterwards.
// =====================================================================

#define USER_SETUP_INFO "Narod_Jukebox_ILI9341"

// ---- Driver ---------------------------------------------------------
#define ILI9341_DRIVER

// Only if red and blue come out swapped:
// #define TFT_RGB_ORDER TFT_BGR
// Only if every colour is inverted (black shows white):
// #define TFT_INVERSION_ON

// ---- Pins (fixed by the PCB) ----------------------------------------
// VSPI hardware pins, so the fast IO_MUX path is used.
#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS    5   // strapping pin - fine, only read at reset
#define TFT_DC    2   // strapping pin - fine, only read at reset
#define TFT_RST   4

#define TOUCH_CS 21   // XPT2046 on the same SPI bus
// T_IRQ (GPIO 27) is not a TFT_eSPI setting; the sketch reads it itself.

// No TFT_BL: the backlight is hard-wired to 3V3.

// ---- Fonts (only what the sketch uses, saves flash) -----------------
#define LOAD_GLCD   // Font 1, 8 px
#define LOAD_FONT2  // Font 2, 16 px  - artist, album, status
#define LOAD_FONT4  // Font 4, 26 px  - title, headings
// #define LOAD_FONT6
// #define LOAD_FONT7
// #define LOAD_FONT8
// #define LOAD_GFXFF
// #define SMOOTH_FONT

// ---- SPI clocks -----------------------------------------------------
#define SPI_FREQUENCY        40000000  // drop to 27000000 if you see glitches
#define SPI_READ_FREQUENCY   20000000
#define SPI_TOUCH_FREQUENCY   2500000  // XPT2046 max is ~2.5 MHz

// Needed because the touch controller shares the bus with the display
#define SUPPORT_TRANSACTIONS
