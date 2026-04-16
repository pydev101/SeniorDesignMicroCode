#pragma once
// =============================================================================
// st7789_display.hpp
// ST7789 Display Library for Raspberry Pi Pico (C++ / Pico SDK)
// 480x320 landscape orientation, direct SPI writes
// =============================================================================

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <cstdint>
#include <cstdlib>  // abs
#include <cstring>
#include <algorithm>

// =============================================================================
// COLOR CONSTANTS (RGB565)
// =============================================================================
static constexpr uint16_t BLACK   = 0x0000;
static constexpr uint16_t WHITE   = 0xFFFF;
static constexpr uint16_t RED     = 0xF800;
static constexpr uint16_t GREEN   = 0x07E0;
static constexpr uint16_t BLUE    = 0x001F;
static constexpr uint16_t CYAN    = 0x07FF;
static constexpr uint16_t MAGENTA = 0xF81F;
static constexpr uint16_t YELLOW  = 0xFFE0;
static constexpr uint16_t ORANGE  = 0xFD20;

// Sentinel to indicate "no background" (transparent)
static constexpr int32_t NO_BG = -1;

inline uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// =============================================================================
// BUILT-IN FONT (8x8 pixels, ASCII 32-126, MSB-first)
// =============================================================================
extern const uint8_t FONT_8X8[760];

// =============================================================================
// DISPLAY CLASS
// =============================================================================
class Display {
public:
    static constexpr int WIDTH  = 480;
    static constexpr int HEIGHT = 320;

    // pin defaults match the MicroPython version
    Display(spi_inst_t* spi     = spi1,
            uint        sck_pin  = 10,
            uint        mosi_pin = 11,
            uint        cs_pin   = 9,
            uint        dc_pin   = 8,
            uint        rst_pin  = 12,
            int         bl_pin   = -1,   // -1 = hardwired, no software control
            uint        baudrate = 62'500'000);

    // --- Screen fills --------------------------------------------------------
    void fill_screen(uint16_t color);
    void fill_rect(int x, int y, int w, int h, uint16_t color);
    void clear() { fill_screen(BLACK); }

    // --- Primitives ----------------------------------------------------------
    void pixel(int x, int y, uint16_t color);
    void hline(int x, int y, int w, uint16_t color);
    void vline(int x, int y, int h, uint16_t color);
    void line(int x1, int y1, int x2, int y2, uint16_t color);
    void rect(int x, int y, int w, int h, uint16_t color, int thickness = 1);

    // --- Text ----------------------------------------------------------------
    int char_draw(int x, int y, char c, uint16_t color,
                  int32_t bg = NO_BG, int scale = 1);
    int text(int x, int y, const char* str, uint16_t color,
             int32_t bg = NO_BG, int scale = 1);

    // --- Graph helpers -------------------------------------------------------
    void graph_bars(int x_start, int x_stop, int y_baseline,
                    const float* data, int data_len,
                    int bar_width, float y_scale,
                    uint16_t color, int32_t bg = NO_BG, int spacing = 0);

    void graph_bar_from_array(int x_start, int x_stop, int y_baseline,
                              const float* data_array, int index,
                              int bar_width, float y_scale,
                              uint16_t color, int32_t bg = NO_BG);

    void graph_line(int x_start, int y_center,
                    const float* data, int data_len,
                    float x_scale, float y_scale, uint16_t color);

    void graph_points(int x_start, int y_center,
                      const float* data, int data_len,
                      float x_scale, float y_scale, uint16_t color);

    // --- Logo ----------------------------------------------------------------
    void draw_logo(int x, int y, uint16_t color, int32_t bg = NO_BG);

    // --- Backlight -----------------------------------------------------------
    void backlight(bool on);

private:
    spi_inst_t* _spi;
    uint _cs, _dc, _rst;
    int  _bl;  // -1 if not used

    void _write_cmd(uint8_t cmd);
    void _write_data(const uint8_t* buf, size_t len);
    void _write_data_byte(uint8_t b);
    void _set_window(int x, int y, int w, int h);
    void _init_display();

    // Reusable row buffer for fill operations (2 bytes/pixel * 480 pixels)
    uint8_t _row_buf[WIDTH * 2];
};
