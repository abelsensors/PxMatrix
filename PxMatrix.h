/*********************************************************************
This is a library for Chinese LED matrix displays

Written by Dominic Buchstaller.
BSD license, check license.txt for more information
*********************************************************************/

#ifndef _PxMATRIX_H
#define _PxMATRIX_H

// Color depth per primary color - the more the slower the update
#ifndef PxMATRIX_MAX_COLOR_DEPTH
#define PxMATRIX_MAX_COLOR_DEPTH 4
#endif

#if PxMATRIX_MAX_COLOR_DEPTH > 8 || PxMATRIX_MAX_COLOR_DEPTH < 1
#error "PxMATRIX_MAX_COLOR_DEPTH must be 1 to 8 bits maximum"
#endif

#ifndef PxMATRIX_MIN_COLOR_DEPTH
#define PxMATRIX_MIN_COLOR_DEPTH 1
#endif

#if PxMATRIX_MIN_COLOR_DEPTH > 8 || PxMATRIX_MIN_COLOR_DEPTH < 1
#error "PxMATRIX_MIN_COLOR_DEPTH must be 1 to 8 bits maximum"
#endif

#if PxMATRIX_MIN_COLOR_DEPTH > PxMATRIX_MAX_COLOR_DEPTH
#error "PxMATRIX_MIN_COLOR_DEPTH must smaller than PxMATRIX_MAX_COLOR_DEPTH"
#endif

// Defines the buffer height / the maximum height of the matrix
#ifndef PxMATRIX_MAX_HEIGHT
#define PxMATRIX_MAX_HEIGHT 64
#endif

// Defines the buffer width / the maximum width of the matrix
#ifndef PxMATRIX_MAX_WIDTH
#define PxMATRIX_MAX_WIDTH 64
#endif

// Defines the maximum amount of pixels on the matrix
#ifndef PxMATRIX_MAX_PIXELS
#define PxMATRIX_MAX_PIXELS (PxMATRIX_MAX_HEIGHT * PxMATRIX_MAX_WIDTH)
#endif

// Defines how long we display things by default
#ifndef PxMATRIX_DEFAULT_SHOWTIME
#define PxMATRIX_DEFAULT_SHOWTIME 30
#endif

// Defines the speed of the SPI bus (reducing this may help if you experience noisy images)
#ifndef PxMATRIX_MAX_SPI_FREQUENCY
#define PxMATRIX_MAX_SPI_FREQUENCY 20000000
#endif

#ifndef PxMATRIX_MIN_SPI_FREQUENCY
#define PxMATRIX_MIN_SPI_FREQUENCY 3000000
#endif

#if PxMATRIX_MIN_SPI_FREQUENCY > PxMATRIX_MAX_SPI_FREQUENCY
#error "PxMATRIX_MIN_SPI_FREQUENCY must be smaller than PxMATRIX_MAX_SPI_FREQUENCY"
#endif

// Legacy suppport
#ifdef double_buffer
#define PxMATRIX_double_buffer true
#endif

#ifndef _BV
#define _BV(x) (1 << (x))
#endif

#if defined(ESP8266) || defined(ESP32)
#define SPI_TRANSFER(x, y) SPI.writeBytes(x, y)
#define SPI_BYTE(x) SPI.write(x)
#endif

#include <SPI.h>

#include "Adafruit_GFX.h"
#include "Arduino.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <stdlib.h>

// Sometimes some extra width needs to be passed to Adafruit GFX constructor
// to render text close to the end of the display correctly
#ifndef ADAFRUIT_GFX_EXTRA
#define ADAFRUIT_GFX_EXTRA 0
#endif

#ifdef ESP32

#include "esp32-hal-gpio.h"
#include "soc/spi_struct.h"

struct spi_struct_t {
  spi_dev_t* dev;
#if !CONFIG_DISABLE_HAL_LOCKS
  xSemaphoreHandle lock;
#endif
  uint8_t num;
};
#endif

// HW SPI PINS
#define SPI_BUS_CLK 14
#define SPI_BUS_MOSI 13
#define SPI_BUS_MISO 12
#define SPI_BUS_SS 4

struct PxMatrixBuffer {
  uint8_t* Data[PxMATRIX_MAX_COLOR_DEPTH] = {0};
};

class PxMATRIX : public Adafruit_GFX {
 public:
  PxMATRIX(uint16_t width, uint16_t height, uint8_t LATCH, uint8_t OE, uint8_t A, uint8_t B);
  PxMATRIX(uint16_t width, uint16_t height, uint8_t LATCH, uint8_t OE, uint8_t A, uint8_t B, uint8_t C);
  PxMATRIX(uint16_t width, uint16_t height, uint8_t LATCH, uint8_t OE, uint8_t A, uint8_t B, uint8_t C, uint8_t D);
  PxMATRIX(uint16_t width, uint16_t height, uint8_t LATCH, uint8_t OE, uint8_t A, uint8_t B, uint8_t C, uint8_t D,
           uint8_t E);

  void begin(uint8_t row_pattern, uint8_t CLK, uint8_t MOSI, uint8_t MISO, uint8_t SS);
  void begin(uint8_t row_pattern);
  void begin();

  void clearDisplay(void);
  void clearDisplay(bool selected_buffer);

  // Updates the display
  void display();

  // Draw pixels
  void drawPixelRGB565(int16_t x, int16_t y, uint16_t color);

  void drawPixel(int16_t x, int16_t y, uint16_t color);

  void drawPixelRGB888(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b);

  // Does nothing for now (always returns 0)
  uint8_t getPixel(int8_t x, int8_t y);

  // Converts RGB888 to RGB565
  uint16_t color565(uint8_t r, uint8_t g, uint8_t b);

  // FLush the buffer of the display
  void flushDisplay();

  // When using double buffering, this displays the draw buffer
  void showBuffer();

  // Set the number of panels that make up the display area width (default is 1)
  void setPanelsWidth(uint8_t panels);

  // Set the color depth
  void setColorDepth(uint8_t color_depth);

  // Set SPI frequency
  void setSpiFrequency(uint32_t spi_frequency);

  // Get the total amount of pixels controlled by this driver
  uint32_t getPixelCount();

 private:
  // GPIO pins
  int8_t _LATCH_PIN = -1;
  int8_t _OE_PIN = -1;
  int8_t _A_PIN = -1;
  int8_t _B_PIN = -1;
  int8_t _C_PIN = -1;
  int8_t _D_PIN = -1;
  int8_t _E_PIN = -1;

  // SPI pins
  int8_t _SPI_CLK = SPI_BUS_CLK;
  int8_t _SPI_MOSI = SPI_BUS_MOSI;
  int8_t _SPI_MISO = SPI_BUS_MISO;
  int8_t _SPI_SS = SPI_BUS_SS;

  uint16_t _width;
  uint16_t _height;
  uint8_t _panels_width;
  uint8_t _rows_per_buffer;
  uint8_t _row_sets_per_buffer;
  uint8_t _panel_width_bytes;

  // Color offset
  uint8_t _color_R_offset;
  uint8_t _color_G_offset;
  uint8_t _color_B_offset;

  // Color depth currently used
  uint8_t _color_depth;
  uint8_t _color_half_step;

  // Holds some pre-computed values for faster pixel drawing
  uint32_t _row_offset[PxMATRIX_MAX_HEIGHT];

  // Holds the display row pattern type
  uint8_t _row_pattern;

  // Number of bytes in one color
  uint8_t _pattern_color_bytes;

  // Total number of bytes that is pushed to the display at a time
  // 3 * _pattern_color_bytes
  uint16_t _send_buffer_size;

  // This is for double buffering
  bool _active_buffer;

  // Generic function that draw one pixel
  void fillMatrixBuffer(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b, bool selected_buffer);

  // Init code common to both constructors
  void init(uint16_t width, uint16_t height, uint8_t LATCH, uint8_t OE, uint8_t A, uint8_t B);

  // Latch the data in to the buffers
  void latch();

  // Set row multiplexer
  void set_mux(uint8_t value);

  void spi_init();
};
#endif  //_PxMATRIX_H
