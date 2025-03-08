#ifndef _USER_SETUP_H_
#define _USER_SETUP_H_

//=============================================================================
// 1. Display Driver Selection
//-----------------------------------------------------------------------------

// Select the driver for the ILI9341 display:
#define ILI9341_DRIVER

//=============================================================================
// 2. SPI Bus Pin Definitions
//-----------------------------------------------------------------------------

// These pins are for the hardware SPI (VSPI) on the ESP32:
#define TFT_MOSI  23   // VSPI MOSI
#define TFT_MISO  19   // VSPI MISO
#define TFT_SCLK  18   // VSPI Clock

//=============================================================================
// 3. Display (ILI9341) Control Pin Definitions
//-----------------------------------------------------------------------------

// Assign the pins for the display module as follows:
#define TFT_CS    5    // Chip Select (screen_CS) -> GPIO 5
#define TFT_DC    4    // Data/Command control (screen_D/C) -> GPIO 4

// For many modules the reset pin is controlled internally or tied to 3.3V.
// If your module does not require an external reset control, set it to -1:
#define TFT_RST   -1   // Reset pin (-1 if not used)

//=============================================================================
// 4. SPI Frequency Setting
//-----------------------------------------------------------------------------

// Define the SPI clock frequency for the display.
// 40 MHz is typical for ILI9341 displays.
#define SPI_FREQUENCY  40000000

//=============================================================================
// 5. Touch Support & Touch Pins
//-----------------------------------------------------------------------------

#define TOUCH_CS           15  // Touchscreen chip select
#define TOUCH_IRQ          25  // IRQ pin
#define SPI_TOUCH_FREQUENCY 2500000


// ##################################################################################
//
// Section 3. Define the fonts that are to be used here
//
// ##################################################################################

// Comment out the #defines below with // to stop that font being loaded
// The ESP8366 and ESP32 have plenty of memory so commenting out fonts is not
// normally necessary. If all fonts are loaded the extra FLASH space required is
// about 17Kbytes. To save FLASH space only enable the fonts you need!

#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
//#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
//#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
//#define LOAD_FONT6  // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
//#define LOAD_FONT7  // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:-.
//#define LOAD_FONT8  // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.
//#define LOAD_FONT8N // Font 8. Alternative to Font 8 above, slightly narrower, so 3 digits fit a 160 pixel TFT
//#define LOAD_GFXFF  // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts

// Comment out the #define below to stop the SPIFFS filing system and smooth font code being loaded
// this will save ~20kbytes of FLASH
//#define SMOOTH_FONT

#endif  // _USER_SETUP_H_
