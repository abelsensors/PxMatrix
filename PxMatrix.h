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
#define SPI_2BYTE(x) SPI.write16(x)
#endif

#ifdef __AVR__
#define SPI_TRANSFER(x, y) SPI.transfer(x, y)
#define SPI_BYTE(x) SPI.transfer(x)
#define SPI_2BYTE(x) SPI.transfer16(x)
#endif

#include <SPI.h>

#include "Adafruit_GFX.h"
#include "Arduino.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifdef __AVR__
#include <util/delay.h>
#endif

#include <stdlib.h>

// Sometimes some extra width needs to be passed to Adafruit GFX constructor
// to render text close to the end of the display correctly
#ifndef ADAFRUIT_GFX_EXTRA
#define ADAFRUIT_GFX_EXTRA 0
#endif

#ifdef ESP8266
#define GPIO_REG_SET(val) GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, val)
#define GPIO_REG_CLEAR(val) GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, val)
#endif
#ifdef ESP32
#define GPIO_REG_SET(val) GPIO.out_w1ts = val
#define GPIO_REG_CLEAR(val) GPIO.out_w1tc = val
#endif
#ifdef __AVR__
#define GPIO_REG_SET(val) (val < 8) ? PORTD |= _BV(val) : PORTB |= _BV(val - 8)
#define GPIO_REG_CLEAR(val) (val < 8) ? PORTD &= ~_BV(val) : PORTB &= ~_BV(val - 8)
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

// Either the panel handles the multiplexing and we feed BINARY to A-E pins
// or we handle the multiplexing and activate one of A-D pins (STRAIGHT)
enum mux_patterns { BINARY, STRAIGHT };

// This is how the scanning is implemented. LINE just scans it left to right,
// ZIGZAG jumps 4 rows after every byte, ZAGGII alse revereses every second byte
enum scan_patterns { LINE, ZIGZAG, ZZAGG, ZAGGIZ, WZAGZIG, VZAG, ZAGZIG };

// Specifies speciffic driver chip. Most panels implement a standard shifted
// register (SHIFT). Other chips/panels may need special treatment in oder to work
enum driver_chips { SHIFT, FM6124, FM6126A };

// Specify the color order
enum color_orders { RRGGBB, RRBBGG, GGRRBB, GGBBRR, BBRRGG, BBGGRR };

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
  void display(uint16_t show_time);
  void display();

  // Draw pixels
  void drawPixelRGB565(int16_t x, int16_t y, uint16_t color);

  void drawPixel(int16_t x, int16_t y, uint16_t color);

  void drawPixelRGB888(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b);

  // Does nothing for now (always returns 0)
  uint8_t getPixel(int8_t x, int8_t y);

  // Converts RGB888 to RGB565
  uint16_t color565(uint8_t r, uint8_t g, uint8_t b);

  // Helpful for debugging (place in display update loop)
  void displayTestPattern(uint16_t showtime);

  // Helpful for debugging (place in display update loop)
  void displayTestPixel(uint16_t show_time);

  // FLush the buffer of the display
  void flushDisplay();

  // Rotate display
  void setRotate(bool rotate);

  // Flip display
  void setFlip(bool flip);

  // Helps to reduce display update latency on larger displays
  void setFastUpdate(bool fast_update);

  // When using double buffering, this displays the draw buffer
  void showBuffer();

  // Control the minimum color values that result in an active pixel
  void setColorOffset(uint8_t r, uint8_t g, uint8_t b);

  // Set the multiplex implemention {BINARY, STRAIGHT} (default is BINARY)
  void setMuxPattern(mux_patterns mux_pattern);

  // Set the color order
  void setColorOrder(color_orders color_order);

  // Set the time in microseconds that we pause after selecting each mux channel
  // (May help if some rows are missing / the mux chip is too slow)
  void setMuxDelay(uint8_t mux_delay_A, uint8_t mux_delay_B, uint8_t mux_delay_C, uint8_t mux_delay_D,
                   uint8_t mux_delay_E);

  // Set the multiplex pattern {LINE, ZIGZAG, ZAGGIZ, WZAGZIG, VZAG} (default is LINE)
  void setScanPattern(scan_patterns scan_pattern);

  // Set the number of panels that make up the display area width (default is 1)
  void setPanelsWidth(uint8_t panels);

  // Set the brightness of the panels (default is 255)
  void setBrightness(uint8_t brightness);

  // Set driver chip type
  void setDriverChip(driver_chips driver_chip);

  // Set the color depth
  void setColorDepth(uint8_t color_depth);

  // Set SPI frequency
  void setSpiFrequency(uint32_t spi_frequency);

  // Get the total amount of pixels controlled by this driver
  uint32_t getPixelCount();

 private:
  // GPIO pins
  uint8_t _LATCH_PIN;
  uint8_t _OE_PIN;
  uint8_t _A_PIN;
  uint8_t _B_PIN;
  uint8_t _C_PIN;
  uint8_t _D_PIN;
  uint8_t _E_PIN;

  // SPI pins
  uint8_t _SPI_CLK = SPI_BUS_CLK;
  uint8_t _SPI_MOSI = SPI_BUS_MOSI;
  uint8_t _SPI_MISO = SPI_BUS_MISO;
  uint8_t _SPI_SS = SPI_BUS_SS;

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

  // Panel Brightness
  uint8_t _brightness;

  // Color pattern that is pushed to the display
  uint8_t _display_color;

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

  // Display and color engine
  bool _rotate;
  bool _flip;
  bool _fast_update;

  // Holds multiplex pattern
  mux_patterns _mux_pattern;

  // Holdes the color order
  color_orders _color_order;

  uint8_t _mux_delay_A;
  uint8_t _mux_delay_B;
  uint8_t _mux_delay_C;
  uint8_t _mux_delay_D;
  uint8_t _mux_delay_E;

  // Holds the scan pattern
  scan_patterns _scan_pattern;

  // Holds the used driver chip
  driver_chips _driver_chip;

  // Used for test pattern
  uint16_t _test_pixel_counter;
  uint16_t _test_line_counter;
  unsigned long _test_last_call;

  // Generic function that draw one pixel
  void fillMatrixBuffer(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b, bool selected_buffer);

  // Init code common to both constructors
  void init(uint16_t width, uint16_t height, uint8_t LATCH, uint8_t OE, uint8_t A, uint8_t B);

  // Light up LEDs and hold for show_time microseconds
  void latch(uint16_t show_time);

  // Set row multiplexer
  void set_mux(uint8_t value);

  void spi_init();

  // Write configuration register in some driver chips
  void writeRegister(uint16_t reg_value, uint8_t reg_position);
  void fm612xWriteRegister(uint16_t reg_value, uint8_t reg_position);
};
#endif  //_PxMATRIX_H
