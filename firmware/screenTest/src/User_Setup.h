// User_Setup.h for ILI9341 ESP32 configuration
#define USER_SETUP_LOADED

#define ILI9341_DRIVER

#define TFT_MISO  19
#define TFT_MOSI  23
#define TFT_SCLK  18

#define TFT_CS    5
#define TFT_DC    4
#define TFT_RST   -1  // Software reset

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4

#define SPI_FREQUENCY  27000000
#define SPI_READ_FREQUENCY 20000000
#define SPI_TOUCH_FREQUENCY 2500000