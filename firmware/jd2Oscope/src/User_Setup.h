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

#endif  // _USER_SETUP_H_
