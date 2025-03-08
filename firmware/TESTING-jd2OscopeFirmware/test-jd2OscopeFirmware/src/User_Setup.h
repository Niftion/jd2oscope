#ifndef _USER_SETUP_H_
#define _USER_SETUP_H_

#define ILI9341_DRIVER
#define TFT_SPI_FREQUENCY 40000000
#define SPI_TOUCH_FREQUENCY 2000000

// Hardware SPI Pins
#define TFT_MOSI 23
#define TFT_MISO 19
#define TFT_SCLK 18

// Display Control Pins
#define TFT_CS   5
#define TFT_DC   4
#define TFT_RST  -1

// Touchscreen Pins
#define TOUCH_CS  15
#define TOUCH_IRQ 25

// Font Configuration
#define LOAD_GLCD
#define SMOOTH_FONT

#endif