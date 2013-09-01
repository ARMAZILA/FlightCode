/*
 * osdgen.h
 *
 *  Created on: 2.10.2011
 *      Author: Samba
 */

#ifndef OSDGEN_H_
#define OSDGEN_H_

// Size of an array (num items.)
#define SIZEOF_ARRAY(x) (sizeof(x) / sizeof((x)[0]))

enum {
	HUD_VSCALE_FLAG_CLEAR 		= 1,
	HUD_VSCALE_FLAG_NO_NEGATIVE = 2
};

// horizontal alignment: -1 = left, +1 = right.
enum {
	HUD_HALIGN_LEFT = -1,
	HUD_HALIGN_RIGHT = 1
};

// Macros for computing addresses and bit positions.
// NOTE: /16 in y is because we are addressing by word not byte.
#define CALC_BUFF_ADDR(x, y)	(((x) / 8) + ((y) * (GRAPHICS_WIDTH_REAL / 8)))
#define CALC_BIT_IN_WORD(x)		((x) & 7)
#define DEBUG_DELAY

// Macro for writing a word with a mode (NAND = clear, OR = set, XOR = toggle)
// at a given position
#define WRITE_WORD_MODE(buff, addr, mask, mode) \
        switch(mode) { \
                case 0: buff[addr] &= ~mask; break; \
                case 1: buff[addr] |= mask; break; \
                case 2: buff[addr] ^= mask; break; }

#define WRITE_WORD_NAND(buff, addr, mask) { buff[addr] &= ~mask; DEBUG_DELAY; }
#define WRITE_WORD_OR(buff, addr, mask)   { buff[addr] |= mask; DEBUG_DELAY; }
#define WRITE_WORD_XOR(buff, addr, mask)  { buff[addr] ^= mask; DEBUG_DELAY; }

// Horizontal line calculations.
// Edge cases.
#define COMPUTE_HLINE_EDGE_L_MASK(b) ((1 << (8 - (b))) - 1)
#define COMPUTE_HLINE_EDGE_R_MASK(b) (~((1 << (7 - (b))) - 1))
// This computes an island mask.
#define COMPUTE_HLINE_ISLAND_MASK(b0, b1) (COMPUTE_HLINE_EDGE_L_MASK(b0) ^ COMPUTE_HLINE_EDGE_L_MASK(b1));

// Macro for initializing stroke/fill modes. Add new modes here
// if necessary.
#define SETUP_STROKE_FILL(stroke, fill, mode) \
        stroke = 0; fill = 0; \
        if(mode == 0) { stroke = 0; fill = 1; } \
        if(mode == 1) { stroke = 1; fill = 0; } \

// Line endcaps (for horizontal and vertical lines.)
#define ENDCAP_NONE             0
#define ENDCAP_ROUND    1
#define ENDCAP_FLAT     2

#define DRAW_ENDCAP_HLINE(e, x, y, s, f, l) \
        if((e) == ENDCAP_ROUND) /* single pixel endcap */ \
        { osdDrawPixel_lm(x, y, f, l); } \
        else if((e) == ENDCAP_FLAT) /* flat endcap: FIXME, quicker to draw a vertical line(?) */ \
        { osdDrawPixel_lm(x, y - 1, s, l); osdDrawPixel_lm(x, y, s, l); osdDrawPixel_lm(x, y + 1, s, l); }

#define DRAW_ENDCAP_VLINE(e, x, y, s, f, l) \
        if((e) == ENDCAP_ROUND) /* single pixel endcap */ \
        { osdDrawPixel_lm(x, y, f, l); } \
        else if((e) == ENDCAP_FLAT) /* flat endcap: FIXME, quicker to draw a horizontal line(?) */ \
        { osdDrawPixel_lm(x - 1, y, s, l); osdDrawPixel_lm(x, y, s, l); osdDrawPixel_lm(x + 1, y, s, l); }

// Macros for writing pixels in a midpoint circle algorithm.
#define CIRCLE_PLOT_8(buff, cx, cy, x, y, mode) \
        CIRCLE_PLOT_4(buff, cx, cy, x, y, mode); \
        if((x) != (y)) CIRCLE_PLOT_4(buff, cx, cy, y, x, mode);

#define CIRCLE_PLOT_4(buff, cx, cy, x, y, mode) \
        osdDrawPixel(buff, (cx) + (x), (cy) + (y), mode); \
        osdDrawPixel(buff, (cx) - (x), (cy) + (y), mode); \
        osdDrawPixel(buff, (cx) + (x), (cy) - (y), mode); \
        osdDrawPixel(buff, (cx) - (x), (cy) - (y), mode);

// Font flags.
enum {
	FONT_BOLD	= 1,              // bold text (no outline)
	FONT_INVERT	= 2               // invert: border white, inside black
};

// Vertical text alignments.
enum {
	TEXT_VA_TOP		= 0,
	TEXT_VA_MIDDLE	= 1,
	TEXT_VA_BOTTOM	= 2
};

// Horizintal text alignments.
enum {
	TEXT_HA_LEFT	= 0,
	TEXT_HA_CENTER	= 1,
	TEXT_HA_RIGHT	= 2
};

// Fonts indexes.
enum {
	FONT_OUTLINED8X14	= 0,
	FONT_OUTLINED8X8	= 1,
	FONT_8X10			= 2,
	FONT_12X18			= 3
};

// Text dimension structures.
struct FontDimensions
{
        uint16_t width, height;
};

// Max/Min macros.
#define MAX(a, b)       		((a) > (b) ? (a) : (b))
#define MIN(a, b)       		((a) < (b) ? (a) : (b))
#define MAX3(a, b, c)   		MAX(a, MAX(b, c))
#define MIN3(a, b, c)   		MIN(a, MIN(b, c))

// Apply DeadBand
#define APPLY_DEADBAND(x, y)	{ x = (x)+GRAPHICS_HDEADBAND; y=(y)+GRAPHICS_VDEADBAND; }
#define APPLY_VDEADBAND(y)		((y)+GRAPHICS_VDEADBAND)
#define APPLY_HDEADBAND(x)		((x)+GRAPHICS_HDEADBAND)

// Check if coordinates are valid. If not, return.
#define CHECK_COORDS(x, y)		if (x < 0 || x >= GRAPHICS_WIDTH_REAL || y < 0 || y >= GRAPHICS_HEIGHT_REAL) return;
#define CHECK_COORD_X(x)		if (x < 0 || x >= GRAPHICS_WIDTH_REAL) return;
#define CHECK_COORD_Y(y)		if (y < 0 || y >= GRAPHICS_HEIGHT_REAL) return;

// Clip coordinates out of range.
#define CLIP_COORD_X(x) 		{ x = MAX(0, MIN(x, GRAPHICS_WIDTH_REAL)); }
#define CLIP_COORD_Y(y) 		{ y = MAX(0, MIN(y, GRAPHICS_HEIGHT_REAL)); }
#define CLIP_COORDS(x, y) 		{ CLIP_COORD_X(x); CLIP_COORD_Y(y); }

// Macro to swap two variables using XOR swap.
#define SWAP(a, b) 				{ a ^= b; b ^= a; a ^= b; }

// Line triggering
#define LAST_LINE 312 //625/2 //PAL
//#define LAST_LINE 525/2 //NTSC

// Global vars

#define DELAY_1_NOP() asm("nop\r\n")
#define DELAY_2_NOP() asm("nop\r\nnop\r\n")
#define DELAY_3_NOP() asm("nop\r\nnop\r\nnop\r\n")
#define DELAY_4_NOP() asm("nop\r\nnop\r\nnop\r\nnop\r\n")
#define DELAY_5_NOP() asm("nop\r\nnop\r\nnop\r\nnop\r\nnop\r\n")
#define DELAY_6_NOP() asm("nop\r\nnop\r\nnop\r\nnop\r\nnop\r\nnop\r\n")
#define DELAY_7_NOP() asm("nop\r\nnop\r\nnop\r\nnop\r\nnop\r\nnop\r\nnop\r\n")
#define DELAY_8_NOP() asm("nop\r\nnop\r\nnop\r\nnop\r\nnop\r\nnop\r\nnop\r\nnop\r\n")
#define DELAY_9_NOP() asm("nop\r\nnop\r\nnop\r\nnop\r\nnop\r\nnop\r\nnop\r\nnop\r\nnop\r\n")
#define DELAY_10_NOP() asm("nop\r\nnop\r\nnop\r\nnop\r\nnop\r\nnop\r\nnop\r\nnop\r\nnop\r\nnop\r\n")

uint8_t getCharData(uint16_t charPos);
void introText();
void clearGraphics();
uint8_t validPos(uint16_t x, uint16_t y);
void setPixel(uint16_t x, uint16_t y, uint8_t state);
void drawCircle(uint16_t x0, uint16_t y0, uint16_t radius);
void swap(uint16_t* a, uint16_t* b);
void drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void drawBox(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void drawArrow(uint16_t x, uint16_t y, uint16_t angle, uint16_t size);
void drawAttitude(uint16_t x, uint16_t y, int16_t pitch, int16_t roll, uint16_t size);
void introGraphics();
void updateGraphics();
void drawGraphicsLine();


void osdWriteChar(char ch, uint16_t x, uint16_t y, uint8_t flags, uint8_t font);
void osdWriteChar16(char ch, uint16_t x, uint16_t y, uint8_t font);
void osdWriteString(char *str, uint16_t x, uint16_t y, uint16_t xs, uint16_t ys, uint8_t va, uint8_t ha, uint8_t flags, uint8_t font);
void osdDrawPixel(uint8_t *buff, uint16_t x, uint16_t y, uint8_t mode);
void osdDrawPixel_lm(uint16_t x, uint16_t y, uint8_t lmode, uint8_t mmode);
void osdDrawHline(uint8_t *buff, uint16_t x0, uint16_t x1, uint16_t y, uint8_t mode);
void osdDrawHline_lm(uint16_t x0, uint16_t x1, uint16_t y, uint8_t lmode, uint8_t mmode);
void osdDrawHline_outlined(uint16_t x0, uint16_t x1, uint16_t y, uint8_t endcap0, uint8_t endcap1, uint8_t mode, uint8_t mmode);
void osdDrawVline(uint8_t *buff, uint16_t x, uint16_t y0, uint16_t y1, uint8_t mode);
void osdDrawVline_lm(uint16_t x, uint16_t y0, uint16_t y1, uint8_t lmode, uint8_t mmode);
void osdDrawVline_outlined(uint16_t x, uint16_t y0, uint16_t y1, uint8_t endcap0, uint8_t endcap1, uint8_t mode, uint8_t mmode);
void osdDrawRectangle_filled(uint8_t *buff, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t mode);
void osdDrawRectangle_filled_lm(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t lmode, uint8_t mmode);
void osdDrawRectangle_lm(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t lmode, uint8_t mmode);

void write_rectangle_outlined(unsigned int x, unsigned int y, int width, int height, int mode, int mmode);
void write_circle(uint8_t *buff, unsigned int cx, unsigned int cy, unsigned int r, unsigned int dashp, int mode);
void write_circle_outlined(unsigned int cx, unsigned int cy, unsigned int r, unsigned int dashp, int bmode, int mode, int mmode);
void write_circle_filled(uint8_t *buff, unsigned int cx, unsigned int cy, unsigned int r, int mode);

void osdDrawLine(uint8_t *buff, uint16_t x0, uint16_t y0, uint16_t  x1, uint16_t y1, uint8_t mode);
void osdDrawLine_lm(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t lmode, uint8_t mmode);
void osdDrawLine_outlined(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t endcap0, uint8_t endcap1, uint8_t mode, uint8_t mmode);

void write_word_misaligned(uint8_t *buff, uint16_t word, unsigned int addr, unsigned int xoff, int mode);
void write_word_misaligned_NAND(uint8_t *buff, uint16_t word, unsigned int addr, unsigned int xoff);
void write_word_misaligned_OR(uint8_t *buff, uint16_t word, unsigned int addr, unsigned int xoff);
void write_word_misaligned_lm(uint16_t wordl, uint16_t wordm, unsigned int addr, unsigned int xoff, int lmode, int mmode);
//int fetch_font_info(char ch, int font, struct FontEntry *font_info, char *lookup);
//void calc_text_dimensions(char *str, struct FontEntry font, int xs, int ys, struct FontDimensions *dim);

void write_string_formatted(char *str, unsigned int x, unsigned int y, unsigned int xs, unsigned int ys, int va, int ha, int flags);

void osdUpdateGraphics();

#endif /* OSDGEN_H_ */
