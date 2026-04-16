// =============================================================================
// st7789_display.cpp
// ST7789 Display Library for Raspberry Pi Pico (C++ / Pico SDK)
// =============================================================================

#include "st7789_display.hpp"
#include "pico/stdlib.h"
#include <cstring>
#include <algorithm>
#include <cstdlib>

using std::min;
using std::max;
using std::abs;

// =============================================================================
// FONT DATA (8x8, ASCII 32-126, MSB-first per row)
// =============================================================================
const uint8_t FONT_8X8[760] = {
    // Space (32)
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    // ! (33)
    0x18,0x18,0x18,0x18,0x18,0x00,0x18,0x00,
    // " (34)
    0x6C,0x6C,0x24,0x00,0x00,0x00,0x00,0x00,
    // # (35)
    0x6C,0x6C,0xFE,0x6C,0xFE,0x6C,0x6C,0x00,
    // $ (36)
    0x18,0x3E,0x60,0x3C,0x06,0x7C,0x18,0x00,
    // % (37)
    0x00,0x66,0xAC,0xD8,0x36,0x6A,0xCC,0x00,
    // & (38)
    0x38,0x6C,0x68,0x36,0xDC,0xCC,0x76,0x00,
    // ' (39)
    0x18,0x18,0x30,0x00,0x00,0x00,0x00,0x00,
    // ( (40)
    0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x00,
    // ) (41)
    0x30,0x18,0x0C,0x0C,0x0C,0x18,0x30,0x00,
    // * (42)
    0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00,
    // + (43)
    0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00,
    // , (44)
    0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30,
    // - (45)
    0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,
    // . (46)
    0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,
    // / (47)
    0x06,0x0C,0x18,0x30,0x60,0xC0,0x80,0x00,
    // 0 (48)
    0x7C,0xCE,0xDE,0xF6,0xE6,0xC6,0x7C,0x00,
    // 1 (49)
    0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00,
    // 2 (50)
    0x3C,0x66,0x06,0x1C,0x30,0x66,0x7E,0x00,
    // 3 (51)
    0x3C,0x66,0x06,0x1C,0x06,0x66,0x3C,0x00,
    // 4 (52)
    0x1C,0x3C,0x6C,0xCC,0xFE,0x0C,0x0C,0x00,
    // 5 (53)
    0x7E,0x60,0x7C,0x06,0x06,0x66,0x3C,0x00,
    // 6 (54)
    0x1C,0x30,0x60,0x7C,0x66,0x66,0x3C,0x00,
    // 7 (55)
    0x7E,0x66,0x06,0x0C,0x18,0x18,0x18,0x00,
    // 8 (56)
    0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00,
    // 9 (57)
    0x3C,0x66,0x66,0x3E,0x06,0x0C,0x38,0x00,
    // : (58)
    0x00,0x18,0x18,0x00,0x18,0x18,0x00,0x00,
    // ; (59)
    0x00,0x18,0x18,0x00,0x18,0x18,0x30,0x00,
    // < (60)
    0x0C,0x18,0x30,0x60,0x30,0x18,0x0C,0x00,
    // = (61)
    0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00,
    // > (62)
    0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x00,
    // ? (63)
    0x3C,0x66,0x0C,0x18,0x18,0x00,0x18,0x00,
    // @ (64)
    0x7C,0xC6,0xDE,0xDE,0xDE,0xC0,0x7C,0x00,
    // A (65)
    0x18,0x3C,0x66,0x66,0x7E,0x66,0x66,0x00,
    // B (66)
    0x7C,0x66,0x66,0x7C,0x66,0x66,0x7C,0x00,
    // C (67)
    0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00,
    // D (68)
    0x78,0x6C,0x66,0x66,0x66,0x6C,0x78,0x00,
    // E (69)
    0x7E,0x60,0x60,0x7C,0x60,0x60,0x7E,0x00,
    // F (70)
    0x7E,0x60,0x60,0x7C,0x60,0x60,0x60,0x00,
    // G (71)
    0x3C,0x66,0x60,0x6E,0x66,0x66,0x3E,0x00,
    // H (72)
    0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x00,
    // I (73)
    0x7E,0x18,0x18,0x18,0x18,0x18,0x7E,0x00,
    // J (74)
    0x3E,0x0C,0x0C,0x0C,0x0C,0x6C,0x38,0x00,
    // K (75)
    0x66,0x6C,0x78,0x70,0x78,0x6C,0x66,0x00,
    // L (76)
    0x60,0x60,0x60,0x60,0x60,0x60,0x7E,0x00,
    // M (77)
    0xC6,0xEE,0xFE,0xD6,0xC6,0xC6,0xC6,0x00,
    // N (78)
    0x66,0x76,0x7E,0x7E,0x6E,0x66,0x66,0x00,
    // O (79)
    0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,
    // P (80)
    0x7C,0x66,0x66,0x7C,0x60,0x60,0x60,0x00,
    // Q (81)
    0x3C,0x66,0x66,0x66,0x6E,0x3C,0x0E,0x00,
    // R (82)
    0x7C,0x66,0x66,0x7C,0x6C,0x66,0x66,0x00,
    // S (83)
    0x3C,0x66,0x60,0x3C,0x06,0x66,0x3C,0x00,
    // T (84)
    0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x00,
    // U (85)
    0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,
    // V (86)
    0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00,
    // W (87)
    0xC6,0xC6,0xC6,0xD6,0xFE,0xEE,0xC6,0x00,
    // X (88)
    0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x00,
    // Y (89)
    0x66,0x66,0x66,0x3C,0x18,0x18,0x18,0x00,
    // Z (90)
    0x7E,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00,
    // [ (91)
    0x3C,0x30,0x30,0x30,0x30,0x30,0x3C,0x00,
    // \ (92)
    0xC0,0x60,0x30,0x18,0x0C,0x06,0x02,0x00,
    // ] (93)
    0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00,
    // ^ (94)
    0x18,0x3C,0x66,0x00,0x00,0x00,0x00,0x00,
    // _ (95)
    0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00,
    // ` (96)
    0x30,0x18,0x0C,0x00,0x00,0x00,0x00,0x00,
    // a (97)
    0x00,0x00,0x3C,0x06,0x3E,0x66,0x3E,0x00,
    // b (98)
    0x60,0x60,0x7C,0x66,0x66,0x66,0x7C,0x00,
    // c (99)
    0x00,0x00,0x3C,0x66,0x60,0x66,0x3C,0x00,
    // d (100)
    0x06,0x06,0x3E,0x66,0x66,0x66,0x3E,0x00,
    // e (101)
    0x00,0x00,0x3C,0x66,0x7E,0x60,0x3C,0x00,
    // f (102)
    0x1C,0x30,0x30,0x7C,0x30,0x30,0x30,0x00,
    // g (103)
    0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x3C,
    // h (104)
    0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x00,
    // i (105)
    0x18,0x00,0x38,0x18,0x18,0x18,0x3C,0x00,
    // j (106)
    0x18,0x00,0x38,0x18,0x18,0x18,0x18,0x70,
    // k (107)
    0x60,0x60,0x66,0x6C,0x78,0x6C,0x66,0x00,
    // l (108)
    0x38,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,
    // m (109)
    0x00,0x00,0xEC,0xFE,0xD6,0xD6,0xC6,0x00,
    // n (110)
    0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x00,
    // o (111)
    0x00,0x00,0x3C,0x66,0x66,0x66,0x3C,0x00,
    // p (112)
    0x00,0x00,0x7C,0x66,0x66,0x7C,0x60,0x60,
    // q (113)
    0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x06,
    // r (114)
    0x00,0x00,0x7C,0x66,0x60,0x60,0x60,0x00,
    // s (115)
    0x00,0x00,0x3E,0x60,0x3C,0x06,0x7C,0x00,
    // t (116)
    0x30,0x30,0x7C,0x30,0x30,0x30,0x1C,0x00,
    // u (117)
    0x00,0x00,0x66,0x66,0x66,0x66,0x3E,0x00,
    // v (118)
    0x00,0x00,0x66,0x66,0x66,0x3C,0x18,0x00,
    // w (119)
    0x00,0x00,0xC6,0xD6,0xD6,0xFE,0x6C,0x00,
    // x (120)
    0x00,0x00,0x66,0x3C,0x18,0x3C,0x66,0x00,
    // y (121)
    0x00,0x00,0x66,0x66,0x66,0x3E,0x06,0x3C,
    // z (122)
    0x00,0x00,0x7E,0x0C,0x18,0x30,0x7E,0x00,
    // { (123)
    0x0C,0x18,0x18,0x70,0x18,0x18,0x0C,0x00,
    // | (124)
    0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00,
    // } (125)
    0x30,0x18,0x18,0x0E,0x18,0x18,0x30,0x00,
    // ~ (126)
    0x31,0x6B,0x46,0x00,0x00,0x00,0x00,0x00,
};

// =============================================================================
// LOGO DATA (24x24 bitmap, 1 bit per pixel)
// =============================================================================
static const uint32_t LOGO_DATA[24] = {
    0b000000000000000000000000,  // Row 0
    0b000000000000001111000000,  // Row 1
    0b000000000000010000100000,  // Row 2
    0b000000000000100010111000,  // Row 3
    0b000000111110100100100100,  // Row 4
    0b000011000001101001000010,  // Row 5
    0b000100011000011110010010,  // Row 6
    0b001000011000000100100100,  // Row 7
    0b010011000001110011001000,  // Row 8
    0b010011000011111001010000,  // Row 9
    0b010000000111111000100000,  // Row 10
    0b010000001111110000100000,  // Row 11
    0b001000011111100000010000,  // Row 12
    0b001100011111000001010000,  // Row 13
    0b010101111110000011001000,  // Row 14
    0b010100111000000111101000,  // Row 15
    0b000010000000001111101000,  // Row 16
    0b000001100000111111001000,  // Row 17
    0b000010110001111100011000,  // Row 18
    0b000100001000000011111000,  // Row 19
    0b000010010111111101111100,  // Row 20
    0b000000010000000000001100,  // Row 21
    0b000000001000000000000010,  // Row 22
    0b000000000000000000000000,  // Row 23
};

// =============================================================================
// CONSTRUCTOR
// =============================================================================
Display::Display(spi_inst_t* spi, uint sck_pin, uint mosi_pin,
                 uint cs_pin, uint dc_pin, uint rst_pin,
                 int bl_pin, uint baudrate)
    : _spi(spi), _cs(cs_pin), _dc(dc_pin), _rst(rst_pin), _bl(bl_pin)
{
    // Init SPI
    spi_init(_spi, baudrate);
    spi_set_format(_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(sck_pin,  GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);

    // Control pins
    gpio_init(_cs);  gpio_set_dir(_cs,  GPIO_OUT); gpio_put(_cs,  1);
    gpio_init(_dc);  gpio_set_dir(_dc,  GPIO_OUT); gpio_put(_dc,  0);
    gpio_init(_rst); gpio_set_dir(_rst, GPIO_OUT); gpio_put(_rst, 1);

    // Backlight
    if (_bl >= 0) {
        gpio_init((uint)_bl);
        gpio_set_dir((uint)_bl, GPIO_OUT);
        gpio_put((uint)_bl, 1);
    }

    _init_display();
}

// =============================================================================
// LOW-LEVEL SPI
// =============================================================================
void Display::_write_cmd(uint8_t cmd) {
    gpio_put(_cs, 0);
    gpio_put(_dc, 0);
    spi_write_blocking(_spi, &cmd, 1);
    gpio_put(_cs, 1);
}

void Display::_write_data_byte(uint8_t b) {
    gpio_put(_cs, 0);
    gpio_put(_dc, 1);
    spi_write_blocking(_spi, &b, 1);
    gpio_put(_cs, 1);
}

void Display::_write_data(const uint8_t* buf, size_t len) {
    gpio_put(_cs, 0);
    gpio_put(_dc, 1);
    spi_write_blocking(_spi, buf, len);
    gpio_put(_cs, 1);
}

// =============================================================================
// DISPLAY INITIALISATION
// =============================================================================
void Display::_init_display() {
    // Hardware reset
    gpio_put(_rst, 1); sleep_ms(50);
    gpio_put(_rst, 0); sleep_ms(50);
    gpio_put(_rst, 1); sleep_ms(150);

    // Software reset
    _write_cmd(0x01); sleep_ms(150);

    // Sleep out
    _write_cmd(0x11); sleep_ms(150);

    // Color mode: 16-bit RGB565
    _write_cmd(0x3A);
    _write_data_byte(0x55);

    // Memory access control — 180° landscape (pins on right)
    _write_cmd(0x36);
    _write_data_byte(0xE8);
    // 0x28 = 0°   (pins left)  | 0x48 = 90°  (pins bottom)
    // 0xE8 = 180° (pins right) | 0x88 = 270° (pins top)

    // Display inversion on (required for most ST7789 panels)
    _write_cmd(0x21);

    // Display on
    _write_cmd(0x29); sleep_ms(50);
}

// =============================================================================
// WINDOW / ADDRESS SET
// =============================================================================
void Display::_set_window(int x, int y, int w, int h) {
    int xe = x + w - 1;
    int ye = y + h - 1;

    uint8_t caset[4] = {
        (uint8_t)(x  >> 8), (uint8_t)(x  & 0xFF),
        (uint8_t)(xe >> 8), (uint8_t)(xe & 0xFF)
    };
    uint8_t raset[4] = {
        (uint8_t)(y  >> 8), (uint8_t)(y  & 0xFF),
        (uint8_t)(ye >> 8), (uint8_t)(ye & 0xFF)
    };

    _write_cmd(0x2A); _write_data(caset, 4);
    _write_cmd(0x2B); _write_data(raset, 4);
    _write_cmd(0x2C);
}

// =============================================================================
// SCREEN FILLS
// =============================================================================
void Display::fill_screen(uint16_t color) {
    _set_window(0, 0, WIDTH, HEIGHT);

    // Build one row of pixel data
    uint8_t hi = color >> 8, lo = color & 0xFF;
    for (int i = 0; i < WIDTH; i++) {
        _row_buf[i * 2]     = hi;
        _row_buf[i * 2 + 1] = lo;
    }

    gpio_put(_cs, 0);
    gpio_put(_dc, 1);
    for (int r = 0; r < HEIGHT; r++) {
        spi_write_blocking(_spi, _row_buf, WIDTH * 2);
    }
    gpio_put(_cs, 1);
}

void Display::fill_rect(int x, int y, int w, int h, uint16_t color) {
    // Clip
    if (x < 0)          { w += x; x = 0; }
    if (y < 0)          { h += y; y = 0; }
    if (x + w > WIDTH)  { w = WIDTH  - x; }
    if (y + h > HEIGHT) { h = HEIGHT - y; }
    if (w <= 0 || h <= 0) return;

    _set_window(x, y, w, h);

    uint8_t hi = color >> 8, lo = color & 0xFF;
    for (int i = 0; i < w; i++) {
        _row_buf[i * 2]     = hi;
        _row_buf[i * 2 + 1] = lo;
    }

    gpio_put(_cs, 0);
    gpio_put(_dc, 1);
    for (int r = 0; r < h; r++) {
        spi_write_blocking(_spi, _row_buf, w * 2);
    }
    gpio_put(_cs, 1);
}

// =============================================================================
// PRIMITIVES
// =============================================================================
void Display::pixel(int x, int y, uint16_t color) {
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
    _set_window(x, y, 1, 1);
    uint8_t buf[2] = { (uint8_t)(color >> 8), (uint8_t)(color & 0xFF) };
    _write_data(buf, 2);
}

void Display::hline(int x, int y, int w, uint16_t color) {
    if (y < 0 || y >= HEIGHT) return;
    if (x < 0)         { w += x; x = 0; }
    if (x + w > WIDTH) { w = WIDTH - x; }
    if (w <= 0) return;

    _set_window(x, y, w, 1);
    uint8_t hi = color >> 8, lo = color & 0xFF;
    for (int i = 0; i < w; i++) { _row_buf[i*2] = hi; _row_buf[i*2+1] = lo; }
    gpio_put(_cs, 0); gpio_put(_dc, 1);
    spi_write_blocking(_spi, _row_buf, w * 2);
    gpio_put(_cs, 1);
}

void Display::vline(int x, int y, int h, uint16_t color) {
    if (x < 0 || x >= WIDTH) return;
    if (y < 0)          { h += y; y = 0; }
    if (y + h > HEIGHT) { h = HEIGHT - y; }
    if (h <= 0) return;

    _set_window(x, y, 1, h);
    uint8_t buf[2] = { (uint8_t)(color >> 8), (uint8_t)(color & 0xFF) };
    gpio_put(_cs, 0); gpio_put(_dc, 1);
    for (int i = 0; i < h; i++) spi_write_blocking(_spi, buf, 2);
    gpio_put(_cs, 1);
}

void Display::line(int x1, int y1, int x2, int y2, uint16_t color) {
    // Fast paths
    if (y1 == y2) {
        if (x1 > x2) { int t = x1; x1 = x2; x2 = t; }
        hline(x1, y1, x2 - x1 + 1, color);
        return;
    }
    if (x1 == x2) {
        if (y1 > y2) { int t = y1; y1 = y2; y2 = t; }
        vline(x1, y1, y2 - y1 + 1, color);
        return;
    }

    // Bresenham
    int dx =  abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
    int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        pixel(x1, y1, color);
        if (x1 == x2 && y1 == y2) break;
        int e2 = 2 * err;
        if (e2 > dy) { err += dy; x1 += sx; }
        if (e2 < dx) { err += dx; y1 += sy; }
    }
}

void Display::rect(int x, int y, int w, int h, uint16_t color, int thickness) {
    for (int i = 0; i < thickness; i++) {
        hline(x,         y + i,         w, color);  // top
        hline(x,         y + h - 1 - i, w, color);  // bottom
        vline(x + i,         y, h, color);           // left
        vline(x + w - 1 - i, y, h, color);           // right
    }
}

// =============================================================================
// TEXT
// =============================================================================
int Display::char_draw(int x, int y, char c, uint16_t color, int32_t bg, int scale) {
    int code = (uint8_t)c;
    if (code < 32 || code > 126) code = 32;
    int offset = (code - 32) * 8;

    for (int row = 0; row < 8; row++) {
        uint8_t byte = FONT_8X8[offset + row];
        for (int col = 0; col < 8; col++) {
            bool set = (byte & (0x80 >> col)) != 0;
            if (set || bg != NO_BG) {
                uint16_t px_color = set ? color : (uint16_t)bg;
                if (scale == 1) {
                    pixel(x + col, y + row, px_color);
                } else {
                    fill_rect(x + col * scale, y + row * scale, scale, scale, px_color);
                }
            }
        }
    }
    return 8 * scale;
}

int Display::text(int x, int y, const char* str, uint16_t color, int32_t bg, int scale) {
    int cursor_x = x;
    int char_w   = 8 * scale;
    while (*str) {
        if (cursor_x + char_w > WIDTH) break;
        char_draw(cursor_x, y, *str, color, bg, scale);
        cursor_x += char_w;
        str++;
    }
    return cursor_x - x;
}

// =============================================================================
// GRAPH HELPERS
// =============================================================================
void Display::graph_bars(int x_start, int x_stop, int y_baseline,
                         const float* data, int data_len,
                         int bar_width, float y_scale,
                         uint16_t color, int32_t bg, int spacing)
{
    int x    = x_start;
    int step = bar_width + spacing;
    int x_end = min(x_stop, WIDTH);

    for (int i = 0; i < data_len; i++) {
        if (x + bar_width > x_end) break;

        int bar_height = (int)(data[i] * y_scale);
        int y_top, y_bottom;
        if (bar_height >= 0) {
            y_top    = y_baseline - bar_height;
            y_bottom = y_baseline;
        } else {
            y_top    = y_baseline;
            y_bottom = y_baseline - bar_height;
        }

        int h = abs(y_bottom - y_top);
        if (bg != NO_BG) fill_rect(x, y_top, bar_width, h, (uint16_t)bg);
        fill_rect(x, y_top, bar_width, h, color);
        x += step;
    }
}

void Display::graph_bar_from_array(int x_start, int x_stop, int y_baseline,
                                    const float* data_array, int index,
                                    int bar_width, float y_scale,
                                    uint16_t color, int32_t bg)
{
    if (x_start + bar_width > x_stop || x_start + bar_width > WIDTH) return;

    float value    = data_array[index];
    int bar_height = (int)(value * y_scale);

    int y_top, y_bottom;
    if (bar_height >= 0) {
        y_top    = y_baseline - bar_height;
        y_bottom = y_baseline;
    } else {
        y_top    = y_baseline;
        y_bottom = y_baseline - bar_height;
    }

    y_top    = max(0, min(HEIGHT - 1, y_top));
    y_bottom = max(0, min(HEIGHT - 1, y_bottom));

    if (bg != NO_BG) {
        int max_bar_h = (int)y_scale;
        int clear_top = max(0, y_baseline - max_bar_h);
        int clear_h   = y_baseline - clear_top;
        fill_rect(x_start, clear_top, bar_width, clear_h, (uint16_t)bg);
    }

    int actual_h = abs(y_bottom - y_top);
    if (actual_h > 0) fill_rect(x_start, y_top, bar_width, actual_h, color);
}

void Display::graph_line(int x_start, int y_center,
                          const float* data, int data_len,
                          float x_scale, float y_scale, uint16_t color)
{
    if (data_len < 2) return;
    int prev_x = x_start;
    int prev_y = y_center - (int)(data[0] * y_scale);

    for (int i = 1; i < data_len; i++) {
        int curr_x = x_start + (int)(i * x_scale);
        if (curr_x >= WIDTH) break;
        int curr_y = y_center - (int)(data[i] * y_scale);
        prev_y = max(0, min(HEIGHT - 1, prev_y));
        curr_y = max(0, min(HEIGHT - 1, curr_y));
        line(prev_x, prev_y, curr_x, curr_y, color);
        prev_x = curr_x;
        prev_y = curr_y;
    }
}

void Display::graph_points(int x_start, int y_center,
                            const float* data, int data_len,
                            float x_scale, float y_scale, uint16_t color)
{
    for (int i = 0; i < data_len; i++) {
        int x = x_start + (int)(i * x_scale);
        if (x >= WIDTH) break;
        int y = y_center - (int)(data[i] * y_scale);
        y = max(0, min(HEIGHT - 1, y));
        pixel(x, y, color);
    }
}

// =============================================================================
// LOGO
// =============================================================================
void Display::draw_logo(int x, int y, uint16_t color, int32_t bg) {
    for (int row = 0; row < 24; row++) {
        uint32_t row_bits = LOGO_DATA[row];
        for (int col = 0; col < 24; col++) {
            uint32_t mask = 1u << (23 - col);
            if (row_bits & mask) {
                pixel(x + col, y + row, color);
            } else if (bg != NO_BG) {
                pixel(x + col, y + row, (uint16_t)bg);
            }
        }
    }
}

// =============================================================================
// BACKLIGHT
// =============================================================================
void Display::backlight(bool on) {
    if (_bl >= 0) gpio_put((uint)_bl, on ? 1 : 0);
}
