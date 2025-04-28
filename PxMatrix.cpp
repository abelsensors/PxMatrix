/*********************************************************************
This is a library for Chinese LED matrix displays

Written by Dominic Buchstaller.
BSD license, check license.txt for more information
*********************************************************************/

#include "PxMatrix.h"

// the display buffers for the LED matrix
static PxMatrixBuffer PxMATRIX_buffer = {};
#ifdef PxMATRIX_double_buffer
static PxMatrixBuffer PxMATRIX_buffer2 = {};
#endif

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t PxMATRIX::color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// Init code common to both constructors
void PxMATRIX::init(uint16_t width, uint16_t height, uint8_t LATCH, uint8_t OE, uint8_t A, uint8_t B) {
  _LATCH_PIN = LATCH;
  _OE_PIN = OE;
  _color_depth = 0;

  _A_PIN = A;
  _B_PIN = B;

  if (width > PxMATRIX_MAX_WIDTH) {
#ifdef DEBUG_ESP_PORT
    DEBUG_ESP_PORT.print("[PxMatrix] Width larger than PxMATRIX_MAX_WIDTH.\n");
#endif
  }

  if (height > PxMATRIX_MAX_HEIGHT) {
#ifdef DEBUG_ESP_PORT
    DEBUG_ESP_PORT.print("[PxMatrix] Height larger than PxMATRIX_MAX_HEIGHT.\n");
#endif
  }

  if (width * height > PxMATRIX_MAX_PIXELS) {
#ifdef DEBUG_ESP_PORT
    DEBUG_ESP_PORT.print("[PxMatrix] Amount of needed pixels bigger than PxMATRIX_MAX_PIXELS.\n");
#endif
  }

  _width = width;
  _height = height;
  _panels_width = 1;

  _rows_per_buffer = _height / 2;
  _panel_width_bytes = (_width / _panels_width) / 8;

  _active_buffer = false;

  _color_R_offset = 0;
  _color_G_offset = 0;
  _color_B_offset = 0;

  _row_pattern = 0;
  clearDisplay(0);
#ifdef PxMATRIX_double_buffer
  clearDisplay(1);
#endif
}

void PxMATRIX::setPanelsWidth(uint8_t panels) {
  _panels_width = panels;
  _panel_width_bytes = (_width / _panels_width) / 8;
}

PxMATRIX::PxMATRIX(uint16_t width, uint16_t height, uint8_t LATCH, uint8_t OE, uint8_t A, uint8_t B)
    : Adafruit_GFX(width + ADAFRUIT_GFX_EXTRA, height) {
  init(width, height, LATCH, OE, A, B);
}

PxMATRIX::PxMATRIX(uint16_t width, uint16_t height, uint8_t LATCH, uint8_t OE, uint8_t A, uint8_t B, uint8_t C)
    : Adafruit_GFX(width + ADAFRUIT_GFX_EXTRA, height) {
  _C_PIN = C;
  init(width, height, LATCH, OE, A, B);
}

PxMATRIX::PxMATRIX(uint16_t width, uint16_t height, uint8_t LATCH, uint8_t OE, uint8_t A, uint8_t B, uint8_t C,
                   uint8_t D)
    : Adafruit_GFX(width + ADAFRUIT_GFX_EXTRA, height) {
  _C_PIN = C;
  _D_PIN = D;
  init(width, height, LATCH, OE, A, B);
}

PxMATRIX::PxMATRIX(uint16_t width, uint16_t height, uint8_t LATCH, uint8_t OE, uint8_t A, uint8_t B, uint8_t C,
                   uint8_t D, uint8_t E)
    : Adafruit_GFX(width + ADAFRUIT_GFX_EXTRA, height) {
  _C_PIN = C;
  _D_PIN = D;
  _E_PIN = E;
  init(width, height, LATCH, OE, A, B);
}

void PxMATRIX::drawPixel(int16_t x, int16_t y, uint16_t color) { drawPixelRGB565(x, y, color); }

void PxMATRIX::showBuffer() { _active_buffer = !_active_buffer; }

void PxMATRIX::fillMatrixBuffer(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b, bool selected_buffer) {
  if (_color_depth == 0) return;

  x = _width - 1 - x;

  if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height)) return;

  uint32_t base_offset;
  uint32_t total_offset_r = 0;
  uint32_t total_offset_g = 0;
  uint32_t total_offset_b = 0;

  // can only be non-zero when _height/(2 inputs per panel)/_row_pattern > 1
  // i.e.: 32x32 panel with 1/8 scan (A/B/C lines) -> 32/2/8 = 2
  uint8_t vert_index_in_buffer = (y % _rows_per_buffer) / _row_pattern;  // which set of rows per buffer
  // can only ever be 0/1 since there are only ever 2 separate input sets present for this variety of panels
  // (R1G1B1/R2G2B2)
  uint8_t which_buffer = y / _rows_per_buffer;
  uint8_t x_byte = x / 8;
  // assumes panels are only ever chained for more width
  uint16_t which_panel = x_byte / _panel_width_bytes;
  uint8_t in_row_byte_offset = x_byte % _panel_width_bytes;
  // this could be pretty easily extended to vertical stacking as well
  total_offset_r =
      _row_offset[y] - in_row_byte_offset -
      _panel_width_bytes * (_row_sets_per_buffer * (_panels_width * which_buffer + which_panel) + vert_index_in_buffer);

  uint8_t bit_select = x % 8;

  total_offset_g = total_offset_r - _pattern_color_bytes;
  total_offset_b = total_offset_g - _pattern_color_bytes;

  PxMatrixBuffer* PxMATRIX_bufferp = &PxMATRIX_buffer;

#ifdef PxMATRIX_double_buffer
  PxMATRIX_bufferp = selected_buffer ? &PxMATRIX_buffer2 : &PxMATRIX_buffer;
#endif

  // if a color closer to on than off, switch the color on
  r = (r == 0) ? 0 : (r > 255 - _color_half_step) ? 255 : r + _color_half_step;
  g = (g == 0) ? 0 : (g > 255 - _color_half_step) ? 255 : g + _color_half_step;
  b = (b == 0) ? 0 : (b > 255 - _color_half_step) ? 255 : b + _color_half_step;

  r = r >> (8 - _color_depth);
  g = g >> (8 - _color_depth);
  b = b >> (8 - _color_depth);

  // Color interlacing
  for (int this_color_bit = 0; this_color_bit < _color_depth; this_color_bit++) {
    if ((r >> this_color_bit) & 0x01)
      PxMATRIX_bufferp->Data[this_color_bit][total_offset_r] |= _BV(bit_select);
    else
      PxMATRIX_bufferp->Data[this_color_bit][total_offset_r] &= ~_BV(bit_select);

    if ((g >> this_color_bit) & 0x01)
      PxMATRIX_bufferp->Data[this_color_bit][total_offset_g] |= _BV(bit_select);
    else
      PxMATRIX_bufferp->Data[this_color_bit][total_offset_g] &= ~_BV(bit_select);

    if ((b >> this_color_bit) & 0x01)
      PxMATRIX_bufferp->Data[this_color_bit][total_offset_b] |= _BV(bit_select);
    else
      PxMATRIX_bufferp->Data[this_color_bit][total_offset_b] &= ~_BV(bit_select);
  }
}

void PxMATRIX::drawPixelRGB565(int16_t x, int16_t y, uint16_t color) {
  uint8_t r = ((((color >> 11) & 0x1F) * 527) + 23) >> 6;
  uint8_t g = ((((color >> 5) & 0x3F) * 259) + 33) >> 6;
  uint8_t b = (((color & 0x1F) * 527) + 23) >> 6;
#ifdef PxMATRIX_double_buffer
  fillMatrixBuffer(x, y, r, g, b, !_active_buffer);
#else
  fillMatrixBuffer(x, y, r, g, b, false);
#endif
}

void PxMATRIX::drawPixelRGB888(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b) {
#ifdef PxMATRIX_double_buffer
  fillMatrixBuffer(x, y, r, g, b, !_active_buffer);
#else
  fillMatrixBuffer(x, y, r, g, b, false);
#endif
}

// the most basic function, get a single pixel
uint8_t PxMATRIX::getPixel(int8_t x, int8_t y) {
  return (0);  // PxMATRIX_buffer[x+ (y/8)*LCDWIDTH] >> (y%8)) & 0x1;
}

void PxMATRIX::begin() { begin(8); }

void PxMATRIX::begin(uint8_t row_pattern, uint8_t CLK, uint8_t MOSI, uint8_t MISO, uint8_t SS) {
  _SPI_CLK = CLK;
  _SPI_MOSI = MOSI;
  _SPI_MISO = MISO;
  _SPI_SS = SS;
  begin(row_pattern);
}

void PxMATRIX::setColorDepth(uint8_t color_depth) {
  if (color_depth < PxMATRIX_MIN_COLOR_DEPTH) {
    color_depth = PxMATRIX_MIN_COLOR_DEPTH;
  }
  if (color_depth > PxMATRIX_MAX_COLOR_DEPTH) {
    color_depth = PxMATRIX_MAX_COLOR_DEPTH;
  }
  _color_depth = color_depth;
  _color_half_step = static_cast<int>((256 / _color_depth) / 2);

  // color depth has been set, so now we know the size of the buffers
  for (uint8_t i = 0; i < _color_depth; i++) {
    if (PxMATRIX_buffer.Data[i] == nullptr) {
      PxMATRIX_buffer.Data[i] = new uint8_t[(_width * _height * 3) / 8];
      memset(PxMATRIX_buffer.Data[i], 0, (_width * _height * 3) / 8);
    }
#ifdef PxMATRIX_double_buffer
    if (PxMATRIX_buffer2.Data[i] == nullptr) {
      PxMATRIX_buffer2.Data[i] = new uint8_t[(_width * _height * 3) / 8];
      memset(PxMATRIX_buffer2.Data[i], 0, (_width * _height * 3) / 8);
    }
#endif
  }
}

void PxMATRIX::setSpiFrequency(uint32_t spi_frequency) {
#if defined(ESP32) || defined(ESP8266)
  if (spi_frequency < PxMATRIX_MIN_SPI_FREQUENCY) {
    spi_frequency = PxMATRIX_MIN_SPI_FREQUENCY;
  }
  if (spi_frequency > PxMATRIX_MAX_SPI_FREQUENCY) {
    spi_frequency = PxMATRIX_MAX_SPI_FREQUENCY;
  }
  SPI.setFrequency(spi_frequency);
#endif
}

uint32_t PxMATRIX::getPixelCount() { return _width * _height; }

void PxMATRIX::spi_init() {
#ifdef ESP32
  SPI.begin(_SPI_CLK, _SPI_MISO, _SPI_MOSI, _SPI_SS);
#else
  SPI.begin();
#endif

#if defined(ESP32) || defined(ESP8266)
  SPI.setFrequency(PxMATRIX_MIN_SPI_FREQUENCY);
#endif

  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
}

void PxMATRIX::begin(uint8_t row_pattern) {
  _row_pattern = row_pattern;
  _pattern_color_bytes = (_height / _row_pattern) * (_width / 8);
  _row_sets_per_buffer = _rows_per_buffer / _row_pattern;
  _send_buffer_size = _pattern_color_bytes * 3;

  spi_init();

  pinMode(_OE_PIN, OUTPUT);
  pinMode(_LATCH_PIN, OUTPUT);
  pinMode(_A_PIN, OUTPUT);
  pinMode(_B_PIN, OUTPUT);
  digitalWrite(_A_PIN, LOW);
  digitalWrite(_B_PIN, LOW);
  digitalWrite(_OE_PIN, HIGH);

  if (_row_pattern >= 8) {
    pinMode(_C_PIN, OUTPUT);
    digitalWrite(_C_PIN, LOW);
  }
  if (_row_pattern >= 16) {
    pinMode(_D_PIN, OUTPUT);
    digitalWrite(_D_PIN, LOW);
  }
  if (_row_pattern >= 32) {
    pinMode(_E_PIN, OUTPUT);
    digitalWrite(_E_PIN, LOW);
  }

  // Precompute row offset values
  for (uint8_t yy = 0; yy < _height; yy++)
    _row_offset[yy] = ((yy) % _row_pattern) * _send_buffer_size + _send_buffer_size - 1;
}

void PxMATRIX::set_mux(uint8_t value) {
  gpio_set_level((gpio_num_t)_A_PIN, (value >> 0) & 1);
  gpio_set_level((gpio_num_t)_B_PIN, (value >> 1) & 1);
  gpio_set_level((gpio_num_t)_C_PIN, (value >> 2) & 1);
  gpio_set_level((gpio_num_t)_D_PIN, (value >> 3) & 1);
  gpio_set_level((gpio_num_t)_E_PIN, (value >> 4) & 1);
}

void PxMATRIX::latch() {
  digitalWrite(_LATCH_PIN, HIGH);
  digitalWrite(_LATCH_PIN, LOW);
}

void PxMATRIX::display() {
  if (_color_depth == 0) return;

  static uint32_t row_on_time = 0;
  PxMatrixBuffer* bufferp = &PxMATRIX_buffer;

#ifdef PxMATRIX_double_buffer
  if (_active_buffer)
    bufferp = &PxMATRIX_buffer2;
  else
    bufferp = &PxMATRIX_buffer;
#endif
  for (uint8_t i = 0; i < _color_depth; i++) {
    if (PxMATRIX_buffer.Data[i] == nullptr) return;
#ifdef PxMATRIX_double_buffer
    if (PxMATRIX_buffer2.Data[i] == nullptr) return;
#endif
  }

  for (uint8_t display_color = 0; display_color < _color_depth; display_color++) {
    for (uint8_t row = 0; row < _row_pattern; row++) {
      int64_t start_time = esp_timer_get_time();
      SPI_TRANSFER(&bufferp->Data[display_color][row * _send_buffer_size], _send_buffer_size);
      int64_t end_time = esp_timer_get_time();
      digitalWrite(_OE_PIN, 1);
      set_mux(row);
      latch();
      digitalWrite(_OE_PIN, 0);
      if (display_color == 0) {
        // use the first display color to determine the write speed, during the first display color the on time is the
        // write speed
        row_on_time = (uint32_t)(end_time - start_time);
      } else {
        delayMicroseconds((row_on_time << display_color) - row_on_time);
      }
    }
  }
  delayMicroseconds(
      row_on_time);  // Last row, data will not shift in again before disabling the LEDs so wait the row_on_time
  digitalWrite(_OE_PIN, 1);
}

void PxMATRIX::flushDisplay(void) {
  for (int ii = 0; ii < _send_buffer_size; ii++) SPI_BYTE(0x00);
}

void PxMATRIX::clearDisplay(void) {
#ifdef PxMATRIX_double_buffer
  clearDisplay(!_active_buffer);
#else
  clearDisplay(false);
#endif
}

// clear everything
void PxMATRIX::clearDisplay(bool selected_buffer) {
  for (uint8_t i = 0; i < _color_depth; i++) {
#ifdef PxMATRIX_double_buffer
    if (selected_buffer && PxMATRIX_buffer2.Data[i] != nullptr)
      memset(PxMATRIX_buffer2.Data[i], 0, (_width * _height * 3) / 8);
    else if (selected_buffer && PxMATRIX_buffer.Data[i] != nullptr)
      memset(PxMATRIX_buffer.Data[i], 0, (_width * _height * 3) / 8);
#else
    if (selected_buffer && PxMATRIX_buffer.Data[i] != nullptr)
      memset(PxMATRIX_buffer.Data[i], 0, (_width * _height * 3) / 8);
#endif
  }
}
