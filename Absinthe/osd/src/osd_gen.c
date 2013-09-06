
// ****************
#include "main.h"
#include "osd_gen.h"
#include "fonts.h"
#include "font12x18.h"
#include "font8x10.h"

extern uint8_t *draw_buffer_level;
extern uint8_t *draw_buffer_mask;
extern osdData_t osdData;
extern rtc_struct datetime;					// time registers
extern xSemaphoreHandle osdSemaphore;

uint16_t osd_x = 180;
uint16_t osd_y = 130;
uint16_t osd_h = 60;
int16_t osdCicleTime = 0;
int16_t _GPS_directionToHome = 45;
uint16_t _GPS_distanceToHome = 80;

// ****************
// Private functions

// ****************
// Private constants


// ****************
// Private variables

struct splashEntry
{
	unsigned int width, height;
	const uint16_t *level;
	const uint16_t *mask;
};

uint16_t mirror(uint16_t source)
{
	int result = ((source & 0x8000) >> 7) | ((source & 0x4000) >> 5)
			| ((source & 0x2000) >> 3) | ((source & 0x1000) >> 1)
			| ((source & 0x0800) << 1) | ((source & 0x0400) << 3)
			| ((source & 0x0200) << 5) | ((source & 0x0100) << 7)
			| ((source & 0x0080) >> 7) | ((source & 0x0040) >> 5)
			| ((source & 0x0020) >> 3) | ((source & 0x0010) >> 1)
			| ((source & 0x0008) << 1) | ((source & 0x0004) << 3)
			| ((source & 0x0002) << 5) | ((source & 0x0001) << 7);

	return result;
}

void clearGraphics()
{
	memset((uint8_t *) draw_buffer_mask, 0, GRAPHICS_WIDTH * GRAPHICS_HEIGHT);
	memset((uint8_t *) draw_buffer_level, 0, GRAPHICS_WIDTH * GRAPHICS_HEIGHT);
}

#if 0
void copyimage(uint16_t offsetx, uint16_t offsety, int image)
{
	//check top/left position
	if (!validPos(offsetx, offsety))
	{
		return;
	}
	struct splashEntry splash_info;
	splash_info = splash[image];
	offsetx=offsetx/8;
	for (uint16_t y = offsety; y < ((splash_info.height)+offsety); y++)
	{
		uint16_t x1=offsetx;
		for (uint16_t x = offsetx; x < (((splash_info.width)/16)+offsetx); x++)
		{
			draw_buffer_level[y*GRAPHICS_WIDTH+x1+1] = (uint8_t)(mirror(splash_info.level[(y-offsety)*((splash_info.width)/16)+(x-offsetx)])>>8);
			draw_buffer_level[y*GRAPHICS_WIDTH+x1] = (uint8_t)(mirror(splash_info.level[(y-offsety)*((splash_info.width)/16)+(x-offsetx)])&0xFF);
			draw_buffer_mask[y*GRAPHICS_WIDTH+x1+1] = (uint8_t)(mirror(splash_info.mask[(y-offsety)*((splash_info.width)/16)+(x-offsetx)])>>8);
			draw_buffer_mask[y*GRAPHICS_WIDTH+x1] = (uint8_t)(mirror(splash_info.mask[(y-offsety)*((splash_info.width)/16)+(x-offsetx)])&0xFF);
			x1+=2;
		}
	}
}
#endif

uint8_t validPos(uint16_t x, uint16_t y)
{
	if (x < GRAPHICS_HDEADBAND || x >= GRAPHICS_WIDTH_REAL || y >= GRAPHICS_HEIGHT_REAL)
	{
		return 0;
	}
	return 1;
}

// Credit for this one goes to wikipedia! :-)
void drawCircle(uint16_t x0, uint16_t y0, uint16_t radius)
{
	int f = 1 - radius;
	int ddF_x = 1;
	int ddF_y = -2 * radius;
	int x = 0;
	int y = radius;

	osdDrawPixel_lm(x0, y0 + radius, 1, 1);
	osdDrawPixel_lm(x0, y0 - radius, 1, 1);
	osdDrawPixel_lm(x0 + radius, y0, 1, 1);
	osdDrawPixel_lm(x0 - radius, y0, 1, 1);

	while (x < y)
	{
		// ddF_x == 2 * x + 1;
		// ddF_y == -2 * y;
		// f == x*x + y*y - radius*radius + 2*x - y + 1;
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;
		osdDrawPixel_lm(x0 + x, y0 + y, 1, 1);
		osdDrawPixel_lm(x0 - x, y0 + y, 1, 1);
		osdDrawPixel_lm(x0 + x, y0 - y, 1, 1);
		osdDrawPixel_lm(x0 - x, y0 - y, 1, 1);
		osdDrawPixel_lm(x0 + y, y0 + x, 1, 1);
		osdDrawPixel_lm(x0 - y, y0 + x, 1, 1);
		osdDrawPixel_lm(x0 + y, y0 - x, 1, 1);
		osdDrawPixel_lm(x0 - y, y0 - x, 1, 1);
	}
}

/*
void swap(uint16_t* a, uint16_t* b)
{
	uint16_t temp = *a;
	*a = *b;
	*b = temp;
}
*/

const static int8_t sinData[91] =
{ 0, 2, 3, 5, 7, 9, 10, 12, 14, 16, 17, 19, 21, 22, 24, 26, 28, 29, 31, 33, 34,
		36, 37, 39, 41, 42, 44, 45, 47, 48, 50, 52, 53, 54, 56, 57, 59, 60, 62,
		63, 64, 66, 67, 68, 69, 71, 72, 73, 74, 75, 77, 78, 79, 80, 81, 82, 83,
		84, 85, 86, 87, 87, 88, 89, 90, 91, 91, 92, 93, 93, 94, 95, 95, 96, 96,
		97, 97, 97, 98, 98, 98, 99, 99, 99, 99, 100, 100, 100, 100, 100, 100 };

static int8_t iSin(uint16_t angle)
{
	uint16_t pos = angle % 360;
	int8_t mult = 1;

	// 180-359 is same as 0-179 but negative.
	if (pos >= 180)
	{
		pos = pos - 180;
		mult = -1;
	}
	// 0-89 is equal to 90-179 except backwards.
	if (pos >= 90)
	{
		pos = 180 - pos;
	}
	return mult * (int8_t) (sinData[pos]);
}

static int8_t iCos(uint16_t angle)
{
	return iSin(angle + 90);
}

/// Draws four points relative to the given center point.
///
/// \li centerX + X, centerY + Y
/// \li centerX + X, centerY - Y
/// \li centerX - X, centerY + Y
/// \li centerX - X, centerY - Y
///
/// \param centerX the x coordinate of the center point
/// \param centerY the y coordinate of the center point
/// \param deltaX the difference between the centerX coordinate and each pixel drawn
/// \param deltaY the difference between the centerY coordinate and each pixel drawn
/// \param color the color to draw the pixels with.
void plotFourQuadrants(int32_t centerX, int32_t centerY, int32_t deltaX, int32_t deltaY)
{
	osdDrawPixel_lm(centerX + deltaX, centerY + deltaY, 1, 1); // Ist      Quadrant
	osdDrawPixel_lm(centerX - deltaX, centerY + deltaY, 1, 1); // IInd     Quadrant
	osdDrawPixel_lm(centerX - deltaX, centerY - deltaY, 1, 1); // IIIrd    Quadrant
	osdDrawPixel_lm(centerX + deltaX, centerY - deltaY, 1, 1); // IVth     Quadrant
}

/// Implements the midpoint ellipse drawing algorithm which is a bresenham
/// style DDF.
///
/// \param centerX the x coordinate of the center of the ellipse
/// \param centerY the y coordinate of the center of the ellipse
/// \param horizontalRadius the horizontal radius of the ellipse
/// \param verticalRadius the vertical radius of the ellipse
/// \param color the color of the ellipse border
void ellipse(int centerX, int centerY, int horizontalRadius, int verticalRadius)
{
	int64_t doubleHorizontalRadius = horizontalRadius * horizontalRadius;
	int64_t doubleVerticalRadius = verticalRadius * verticalRadius;

	int64_t error = doubleVerticalRadius
			- doubleHorizontalRadius * verticalRadius
			+ (doubleVerticalRadius >> 2);

	int x = 0;
	int y = verticalRadius;
	int deltaX = 0;
	int deltaY = (doubleHorizontalRadius << 1) * y;

	plotFourQuadrants(centerX, centerY, x, y);

	while (deltaY >= deltaX)
	{
		x++;
		deltaX += (doubleVerticalRadius << 1);

		error += deltaX + doubleVerticalRadius;

		if (error >= 0)
		{
			y--;
			deltaY -= (doubleHorizontalRadius << 1);

			error -= deltaY;
		}
		plotFourQuadrants(centerX, centerY, x, y);
	}

	error = (int64_t) (doubleVerticalRadius * (x + 1 / 2.0) * (x + 1 / 2.0)
			+ doubleHorizontalRadius * (y - 1) * (y - 1)
			- doubleHorizontalRadius * doubleVerticalRadius);

	while (y >= 0)
	{
		error += doubleHorizontalRadius;
		y--;
		deltaY -= (doubleHorizontalRadius << 1);
		error -= deltaY;

		if (error <= 0)
		{
			x++;
			deltaX += (doubleVerticalRadius << 1);
			error += deltaX;
		}

		plotFourQuadrants(centerX, centerY, x, y);
	}
}

void osdDrawArrow(uint16_t x, uint16_t y, uint16_t angle, uint16_t size)
{
	int16_t a = iCos(angle);
	int16_t b = iSin(angle);
	a = (a * (size / 2)) / 100;
	b = (b * (size / 2)) / 100;
	osdDrawLine_lm(x - 1 - b, y - 1 + a, x - 1 + b, y - 1 - a, 1, 1); //Direction line
	//write_line_lm((GRAPHICS_SIZE/2)-1 + a/2, (GRAPHICS_SIZE/2)-1 + b/2, (GRAPHICS_SIZE/2)-1 - a/2, (GRAPHICS_SIZE/2)-1 - b/2, 1, 1); //Arrow bottom line
	osdDrawLine_lm(x - 1 + b, y - 1 - a, x - 1 - a / 2, y - 1 - b / 2, 1, 1); // Arrow "wings"
	osdDrawLine_lm(x - 1 + b, y - 1 - a, x - 1 + a / 2, y - 1 + b / 2, 1, 1);
}

void drawBox(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	osdDrawLine_lm(x1, y1, x2, y1, 1, 1); //top
	osdDrawLine_lm(x1, y1, x1, y2, 1, 1); //left
	osdDrawLine_lm(x2, y1, x2, y2, 1, 1); //right
	osdDrawLine_lm(x1, y2, x2, y2, 1, 1); //bottom
}

// simple routines

// SUPEROSD routines, modified

/**
 * osdDrawPixel: Write a pixel at an x,y position to a given surface.
 *
 * @param       buff    pointer to buffer to write in
 * @param       x               x coordinate
 * @param       y               y coordinate
 * @param       mode    0 = clear bit, 1 = set bit, 2 = toggle bit
 */
void osdDrawPixel(uint8_t *buff, uint16_t x, uint16_t y, uint8_t mode)
{
	CHECK_COORDS(x, y);
	APPLY_DEADBAND(x, y);
	// Determine the bit in the word to be set and the word
	// index to set it in.
	uint8_t bitnum = CALC_BIT_IN_WORD(x);
	uint16_t wordnum = CALC_BUFF_ADDR(x, y);
	// Apply a mask.
	uint8_t mask = 1 << (7 - bitnum);
	WRITE_WORD_MODE(buff, wordnum, mask, mode);
}

/**
 * osdDrawPixel_lm: write the pixel on both surfaces (level and mask.)
 * Uses current draw buffer.
 *
 * @param       x               x coordinate
 * @param       y               y coordinate
 * @param       mmode   0 = clear, 1 = set, 2 = toggle
 * @param       lmode   0 = black, 1 = white, 2 = toggle
 */
void osdDrawPixel_lm(uint16_t x, uint16_t y, uint8_t lmode, uint8_t mmode)
{
	CHECK_COORDS(x, y);
	APPLY_DEADBAND(x, y);
	// Determine the bit in the word to be set and the word
	// index to set it in.
	uint8_t bitnum = CALC_BIT_IN_WORD(x);
	uint16_t wordnum = CALC_BUFF_ADDR(x, y);
	// Apply the masks.
	uint8_t mask = 1 << (7 - bitnum);
	WRITE_WORD_MODE(draw_buffer_mask, wordnum, mask, mmode);

	bitnum = CALC_BIT_IN_WORD(x);
	wordnum = CALC_BUFF_ADDR(x, y);
	// Apply the masks.
	mask = 1 << (7 - bitnum);
	WRITE_WORD_MODE(draw_buffer_level, wordnum, mask, lmode);
}

/**
 * write_hline: optimised horizontal line writing algorithm
 *
 * @param       buff    pointer to buffer to write in
 * @param       x0              x0 coordinate
 * @param       x1              x1 coordinate
 * @param       y               y coordinate
 * @param       mode    0 = clear, 1 = set, 2 = toggle
 */
void osdDrawHline(uint8_t *buff, uint16_t x0, uint16_t x1, uint16_t y, uint8_t mode)
{
	x0 = APPLY_HDEADBAND(x0);
	x1 = APPLY_HDEADBAND(x1);
	y = APPLY_VDEADBAND(y);

	CLIP_COORDS(x0, y);
	CLIP_COORDS(x1, y);

	if (x0 > x1) SWAP(x0, x1);

	if (x0 == x1) return;

	/* This is an optimised algorithm for writing horizontal lines.
	 * We begin by finding the addresses of the x0 and x1 points. */
	uint16_t addr0 = CALC_BUFF_ADDR(x0, y);
	uint16_t addr1 = CALC_BUFF_ADDR(x1, y);
	uint16_t addr0_bit = CALC_BIT_IN_WORD(x0);
	uint16_t addr1_bit = CALC_BIT_IN_WORD(x1);
	uint8_t  mask, mask_l, mask_r;
	uint16_t i;

	/* If the addresses are equal, we only need to write one word
	 * which is an island. */
	if (addr0 == addr1)
	{
		mask = COMPUTE_HLINE_ISLAND_MASK(addr0_bit, addr1_bit);
		WRITE_WORD_MODE(buff, addr0, mask, mode);
	}
	/* Otherwise we need to write the edges and then the middle. */
	else
	{
		mask_l = COMPUTE_HLINE_EDGE_L_MASK(addr0_bit);
		mask_r = COMPUTE_HLINE_EDGE_R_MASK(addr1_bit);
		WRITE_WORD_MODE(buff, addr0, mask_l, mode);
		WRITE_WORD_MODE(buff, addr1, mask_r, mode);
		// Now write 0xffff words from start+1 to end-1.
		for (i = addr0 + 1; i <= addr1 - 1; i++)
		{
			uint8_t m = 0xff;
			WRITE_WORD_MODE(buff, i, m, mode);
		}
	}
}

/**
 * write_hline_lm: write both level and mask buffers.
 *
 * @param       x0              x0 coordinate
 * @param       x1              x1 coordinate
 * @param       y               y coordinate
 * @param       lmode   0 = clear, 1 = set, 2 = toggle
 * @param       mmode   0 = clear, 1 = set, 2 = toggle
 */
void osdDrawHline_lm(uint16_t x0, uint16_t x1, uint16_t y, uint8_t lmode, uint8_t mmode)
{
	// TODO: an optimisation would compute the masks and apply to
	// both buffers simultaneously.
	osdDrawHline(draw_buffer_level, x0, x1, y, lmode);
	osdDrawHline(draw_buffer_mask, x0, x1, y, mmode);
}

/**
 * osdDrawHline_outlined: outlined horizontal line with varying endcaps
 * Always uses draw buffer.
 *
 * @param       x0                      x0 coordinate
 * @param       x1                      x1 coordinate
 * @param       y                       y coordinate
 * @param       endcap0         0 = none, 1 = single pixel, 2 = full cap
 * @param       endcap1         0 = none, 1 = single pixel, 2 = full cap
 * @param       mode            0 = black outline, white body, 1 = white outline, black body
 * @param       mmode           0 = clear, 1 = set, 2 = toggle
 */
void osdDrawHline_outlined(uint16_t x0, uint16_t x1, uint16_t y, uint8_t endcap0, uint8_t endcap1, uint8_t mode, uint8_t mmode)
{
	int stroke, fill;
	SETUP_STROKE_FILL(stroke, fill, mode)
	if (x0 > x1)
	{
		SWAP(x0, x1);
	}
	// Draw the main body of the line.
	osdDrawHline_lm(x0 + 1, x1 - 1, y - 1, stroke, mmode);
	osdDrawHline_lm(x0 + 1, x1 - 1, y + 1, stroke, mmode);
	osdDrawHline_lm(x0 + 1, x1 - 1, y, fill, mmode);
	// Draw the endcaps, if any.
	DRAW_ENDCAP_HLINE(endcap0, x0, y, stroke, fill, mmode);
	DRAW_ENDCAP_HLINE(endcap1, x1, y, stroke, fill, mmode);
}

/**
 * osdDrawVline: optimised vertical line writing algorithm
 *
 * @param       buff    pointer to buffer to write in
 * @param       x               x coordinate
 * @param       y0              y0 coordinate
 * @param       y1              y1 coordinate
 * @param       mode    0 = clear, 1 = set, 2 = toggle
 */
void osdDrawVline(uint8_t *buff, uint16_t x, uint16_t y0, uint16_t y1, uint8_t mode)
{
	uint16_t a;

	x = APPLY_HDEADBAND(x);
	y0 = APPLY_VDEADBAND(y0);
	y1 = APPLY_VDEADBAND(y1);
	CLIP_COORDS(x, y0);
	CLIP_COORDS(x, y1);

	if (y0 > y1) SWAP(y0, y1);

	if (y0 == y1) return;

	/* This is an optimised algorithm for writing vertical lines.
	 * We begin by finding the addresses of the x,y0 and x,y1 points. */
	int addr0 = CALC_BUFF_ADDR(x, y0);
	int addr1 = CALC_BUFF_ADDR(x, y1);

	/* Then we calculate the pixel data to be written. */
	int bitnum = CALC_BIT_IN_WORD(x);
	uint16_t mask = 1 << (7 - bitnum);

	/* Run from addr0 to addr1 placing pixels. Increment by the number
	 * of words n each graphics line. */
	for (a = addr0; a <= addr1; a += GRAPHICS_WIDTH_REAL / 8)
	{
		WRITE_WORD_MODE(buff, a, mask, mode);
	}
}

/**
 * osdDrawVline_lm: write both level and mask buffers.
 *
 * @param       x               x coordinate
 * @param       y0              y0 coordinate
 * @param       y1              y1 coordinate
 * @param       lmode   0 = clear, 1 = set, 2 = toggle
 * @param       mmode   0 = clear, 1 = set, 2 = toggle
 */
void osdDrawVline_lm(uint16_t x, uint16_t y0, uint16_t y1, uint8_t lmode, uint8_t mmode)
{
	// TODO: an optimisation would compute the masks and apply to
	// both buffers simultaneously.
	osdDrawVline(draw_buffer_level, x, y0, y1, lmode);
	osdDrawVline(draw_buffer_mask, x, y0, y1, mmode);
}

/**
 * write_vline_outlined: outlined vertical line with varying endcaps
 * Always uses draw buffer.
 *
 * @param       x                       x coordinate
 * @param       y0                      y0 coordinate
 * @param       y1                      y1 coordinate
 * @param       endcap0         0 = none, 1 = single pixel, 2 = full cap
 * @param       endcap1         0 = none, 1 = single pixel, 2 = full cap
 * @param       mode            0 = black outline, white body, 1 = white outline, black body
 * @param       mmode           0 = clear, 1 = set, 2 = toggle
 */
void osdDrawVline_outlined(uint16_t x, uint16_t y0, uint16_t y1, uint8_t endcap0, uint8_t endcap1, uint8_t mode, uint8_t mmode)
{
	int stroke, fill;
	if (y0 > y1)
	{
		SWAP(y0, y1);
	}
	SETUP_STROKE_FILL(stroke, fill, mode);
	// Draw the main body of the line.
	osdDrawVline_lm(x - 1, y0 + 1, y1 - 1, stroke, mmode);
	osdDrawVline_lm(x + 1, y0 + 1, y1 - 1, stroke, mmode);
	osdDrawVline_lm(x, y0 + 1, y1 - 1, fill, mmode);
	// Draw the endcaps, if any.
	DRAW_ENDCAP_VLINE(endcap0, x, y0, stroke, fill, mmode);
	DRAW_ENDCAP_VLINE(endcap1, x, y1, stroke, fill, mmode);
}

/**
 * osdDrawRectangle_filled: draw a filled rectangle.
 *
 * Uses an optimised algorithm which is similar to the horizontal
 * line writing algorithm, but optimised for writing the lines
 * multiple times without recalculating lots of stuff.
 *
 * @param       buff    pointer to buffer to write in
 * @param       x               x coordinate (left)
 * @param       y               y coordinate (top)
 * @param       width   rectangle width
 * @param       height  rectangle height
 * @param       mode    0 = clear, 1 = set, 2 = toggle
 */
void osdDrawRectangle_filled(uint8_t *buff, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t mode)
{
	uint16_t yy, addr0_old, addr1_old;

	APPLY_DEADBAND(x, y);
	CHECK_COORDS(x, y);
	CHECK_COORD_X(x + width);
	CHECK_COORD_Y(y + height);
	if (width <= 0 || height <= 0) return;

	// Calculate as if the rectangle was only a horizontal line. We then
	// step these addresses through each row until we iterate `height` times.
	uint16_t addr0 = CALC_BUFF_ADDR(x, y);
	uint16_t addr1 = CALC_BUFF_ADDR(x + width, y);
	uint8_t  addr0_bit = CALC_BIT_IN_WORD(x);
	uint8_t  addr1_bit = CALC_BIT_IN_WORD(x + width);
	uint8_t  mask, mask_l, mask_r;
	uint16_t i;

	// If the addresses are equal, we need to write one word vertically.
	if (addr0 == addr1)
	{
		mask = COMPUTE_HLINE_ISLAND_MASK(addr0_bit, addr1_bit);
		while (height--)
		{
			WRITE_WORD_MODE(buff, addr0, mask, mode);
			addr0 += GRAPHICS_WIDTH_REAL / 8;
		}
	}
	// Otherwise we need to write the edges and then the middle repeatedly.
	else
	{
		mask_l = COMPUTE_HLINE_EDGE_L_MASK(addr0_bit);
		mask_r = COMPUTE_HLINE_EDGE_R_MASK(addr1_bit);
		// Write edges first.
		yy = 0;
		addr0_old = addr0;
		addr1_old = addr1;
		while (yy < height)
		{
			WRITE_WORD_MODE(buff, addr0, mask_l, mode);
			WRITE_WORD_MODE(buff, addr1, mask_r, mode);
			addr0 += GRAPHICS_WIDTH_REAL / 8;
			addr1 += GRAPHICS_WIDTH_REAL / 8;
			yy++;
		}
		// Now write 0xffff words from start+1 to end-1 for each row.
		yy = 0;
		addr0 = addr0_old;
		addr1 = addr1_old;
		while (yy < height)
		{
			for (i = addr0 + 1; i <= addr1 - 1; i++)
			{
				uint8_t m = 0xff;
				WRITE_WORD_MODE(buff, i, m, mode);
			}
			addr0 += GRAPHICS_WIDTH_REAL / 8;
			addr1 += GRAPHICS_WIDTH_REAL / 8;
			yy++;
		}
	}
}

/**
 * osdDrawRectangle_filled_lm: draw a filled rectangle on both draw buffers.
 *
 * @param       x               x coordinate (left)
 * @param       y               y coordinate (top)
 * @param       width   rectangle width
 * @param       height  rectangle height
 * @param       lmode   0 = clear, 1 = set, 2 = toggle
 * @param       mmode   0 = clear, 1 = set, 2 = toggle
 */
void osdDrawRectangle_filled_lm(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t lmode, uint8_t mmode)
{
	osdDrawRectangle_filled(draw_buffer_level, x, y, width, height, lmode);
	osdDrawRectangle_filled(draw_buffer_mask, x, y, width, height, mmode);
}

/**
 * osdDrawRectangle_lm: draw an outline of a rectangle. Essentially
 * a convenience wrapper for osdDrawHline_lm and osdDrawVline_lm.
 *
 * @param       x               x coordinate (left)
 * @param       y               y coordinate (top)
 * @param       width   rectangle width
 * @param       height  rectangle height
 * @param       lmode   0 = black outline, white body, 1 = white outline, black body
 * @param       mmode   0 = clear, 1 = set, 2 = toggle
 */
void osdDrawRectangle_lm(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t lmode, uint8_t mmode)
{
	osdDrawHline_lm(x, x + width, y, lmode, mmode);
	osdDrawHline_lm(x, x + width, y + height, lmode, mmode);
	osdDrawVline_lm(x, y, y + height, lmode, mmode);
	osdDrawVline_lm(x + width, y, y + height, lmode, mmode);
}

/**
 * write_rectangle_outlined: draw an outline of a rectangle. Essentially
 * a convenience wrapper for draw_hline_outlined and draw_vline_outlined.
 *
 * @param       x               x coordinate (left)
 * @param       y               y coordinate (top)
 * @param       width   rectangle width
 * @param       height  rectangle height
 * @param       mode    0 = black outline, white body, 1 = white outline, black body
 * @param       mmode   0 = clear, 1 = set, 2 = toggle
 */
void write_rectangle_outlined(unsigned int x, unsigned int y, int width,
		int height, int mode, int mmode)
{
	//CHECK_COORDS(x, y);
	//CHECK_COORDS(x + width, y + height);
	//if((x + width) > DISP_WIDTH) width = DISP_WIDTH - x;
	//if((y + height) > DISP_HEIGHT) height = DISP_HEIGHT - y;
	osdDrawHline_outlined(x, x + width, y, ENDCAP_ROUND, ENDCAP_ROUND, mode, mmode);
	osdDrawHline_outlined(x, x + width, y + height, ENDCAP_ROUND, ENDCAP_ROUND, mode, mmode);
	osdDrawVline_outlined(x, y, y + height, ENDCAP_ROUND, ENDCAP_ROUND, mode, mmode);
	osdDrawVline_outlined(x + width, y, y + height, ENDCAP_ROUND, ENDCAP_ROUND, mode, mmode);
}

/**
 * write_circle: draw the outline of a circle on a given buffer,
 * with an optional dash pattern for the line instead of a normal line.
 *
 * @param       buff    pointer to buffer to write in
 * @param       cx              origin x coordinate
 * @param       cy              origin y coordinate
 * @param       r               radius
 * @param       dashp   dash period (pixels) - zero for no dash
 * @param       mode    0 = clear, 1 = set, 2 = toggle
 */
void write_circle(uint8_t *buff, unsigned int cx, unsigned int cy,
		unsigned int r, unsigned int dashp, int mode)
{
	CHECK_COORDS(cx, cy);
	int error = -r, x = r, y = 0;
	while (x >= y)
	{
		if (dashp == 0 || (y % dashp) < (dashp / 2))
		{
			CIRCLE_PLOT_8(buff, cx, cy, x, y, mode);
		}
		error += (y * 2) + 1;
		y++;
		if (error >= 0)
		{
			--x;
			error -= x * 2;
		}
	}
}

/**
 * write_circle_outlined: draw an outlined circle on the draw buffer.
 *
 * @param       cx              origin x coordinate
 * @param       cy              origin y coordinate
 * @param       r               radius
 * @param       dashp   dash period (pixels) - zero for no dash
 * @param       bmode   0 = 4-neighbour border, 1 = 8-neighbour border
 * @param       mode    0 = black outline, white body, 1 = white outline, black body
 * @param       mmode   0 = clear, 1 = set, 2 = toggle
 */
void write_circle_outlined(unsigned int cx, unsigned int cy, unsigned int r,
		unsigned int dashp, int bmode, int mode, int mmode)
{
	int stroke, fill;
	CHECK_COORDS(cx, cy);
	SETUP_STROKE_FILL(stroke, fill, mode);
	// This is a two step procedure. First, we draw the outline of the
	// circle, then we draw the inner part.
	int error = -r, x = r, y = 0;
	while (x >= y)
	{
		if (dashp == 0 || (y % dashp) < (dashp / 2))
		{
			CIRCLE_PLOT_8(draw_buffer_mask, cx, cy, x + 1, y, mmode);
			CIRCLE_PLOT_8(draw_buffer_level, cx, cy, x + 1, y, stroke);
			CIRCLE_PLOT_8(draw_buffer_mask, cx, cy, x, y + 1, mmode);
			CIRCLE_PLOT_8(draw_buffer_level, cx, cy, x, y + 1, stroke);
			CIRCLE_PLOT_8(draw_buffer_mask, cx, cy, x - 1, y, mmode);
			CIRCLE_PLOT_8(draw_buffer_level, cx, cy, x - 1, y, stroke);
			CIRCLE_PLOT_8(draw_buffer_mask, cx, cy, x, y - 1, mmode);
			CIRCLE_PLOT_8(draw_buffer_level, cx, cy, x, y - 1, stroke);
			if (bmode == 1)
			{
				CIRCLE_PLOT_8(draw_buffer_mask, cx, cy, x + 1, y + 1, mmode);
				CIRCLE_PLOT_8(draw_buffer_level, cx, cy, x + 1, y + 1, stroke);
				CIRCLE_PLOT_8(draw_buffer_mask, cx, cy, x - 1, y - 1, mmode);
				CIRCLE_PLOT_8(draw_buffer_level, cx, cy, x - 1, y - 1, stroke);
			}
		}
		error += (y * 2) + 1;
		y++;
		if (error >= 0)
		{
			--x;
			error -= x * 2;
		}
	}
	error = -r;
	x = r;
	y = 0;
	while (x >= y)
	{
		if (dashp == 0 || (y % dashp) < (dashp / 2))
		{
			CIRCLE_PLOT_8(draw_buffer_mask, cx, cy, x, y, mmode);
			CIRCLE_PLOT_8(draw_buffer_level, cx, cy, x, y, fill);
		}
		error += (y * 2) + 1;
		y++;
		if (error >= 0)
		{
			--x;
			error -= x * 2;
		}
	}
}

/**
 * write_circle_filled: fill a circle on a given buffer.
 *
 * @param       buff    pointer to buffer to write in
 * @param       cx              origin x coordinate
 * @param       cy              origin y coordinate
 * @param       r               radius
 * @param       mode    0 = clear, 1 = set, 2 = toggle
 */
void write_circle_filled(uint8_t *buff, unsigned int cx, unsigned int cy,
		unsigned int r, int mode)
{
	CHECK_COORDS(cx, cy);
	int error = -r, x = r, y = 0, xch = 0;
	// It turns out that filled circles can take advantage of the midpoint
	// circle algorithm. We simply draw very fast horizontal lines across each
	// pair of X,Y coordinates. In some cases, this can even be faster than
	// drawing an outlined circle!
	//
	// Due to multiple writes to each set of pixels, we have a special exception
	// for when using the toggling draw mode.
	while (x >= y)
	{
		if (y != 0)
		{
			osdDrawHline(buff, cx - x, cx + x, cy + y, mode);
			osdDrawHline(buff, cx - x, cx + x, cy - y, mode);
			if (mode != 2 || (mode == 2 && xch && (cx - x) != (cx - y)))
			{
				osdDrawHline(buff, cx - y, cx + y, cy + x, mode);
				osdDrawHline(buff, cx - y, cx + y, cy - x, mode);
				xch = 0;
			}
		}
		error += (y * 2) + 1;
		y++;
		if (error >= 0)
		{
			--x;
			xch = 1;
			error -= x * 2;
		}
	}
	// Handle toggle mode.
	if (mode == 2)
	{
		osdDrawHline(buff, cx - r, cx + r, cy, mode);
	}
}

/**
 * osdDrawLine: Draw a line of arbitrary angle.
 *
 * @param       buff    pointer to buffer to write in
 * @param       x0              first x coordinate
 * @param       y0              first y coordinate
 * @param       x1              second x coordinate
 * @param       y1              second y coordinate
 * @param       mode    0 = clear, 1 = set, 2 = toggle
 */
void osdDrawLine(uint8_t *buff, uint16_t x0, uint16_t y0, uint16_t  x1, uint16_t y1, uint8_t mode)
{
	// Based on http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep)
	{
		SWAP(x0, y0);
		SWAP(x1, y1);
	}
	if (x0 > x1)
	{
		SWAP(x0, x1);
		SWAP(y0, y1);
	}

	int16_t deltax = x1 - x0;
	int16_t deltay = abs(y1 - y0);
	int16_t error = deltax / 2;
	int16_t ystep;
	int16_t y = y0;
	int16_t x; 			//, lasty = y, stox = 0;

	if (y0 < y1)
		ystep = 1;
	else
		ystep = -1;

	for (x = x0; x < x1; x++)
	{
		if (steep)
		{
			osdDrawPixel(buff, y, x, mode);
		}
		else
		{
			osdDrawPixel(buff, x, y, mode);
		}
		error -= deltay;
		if (error < 0)
		{
			y += ystep;
			error += deltax;
		}
	}
}

/**
 * osdDrawLine_lm: Draw a line of arbitrary angle.
 *
 * @param       x0              first x coordinate
 * @param       y0              first y coordinate
 * @param       x1              second x coordinate
 * @param       y1              second y coordinate
 * @param       lmode   0 = clear, 1 = set, 2 = toggle
 * @param       mmode   0 = clear, 1 = set, 2 = toggle
 */
void osdDrawLine_lm(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t lmode, uint8_t mmode)
{
	osdDrawLine(draw_buffer_level, x0, y0, x1, y1, lmode);
	osdDrawLine(draw_buffer_mask, x0, y0, x1, y1, mmode);
}

/**
 * osdDrawLine_outlined: Draw a line of arbitrary angle, with an outline.
 *
 * @param       x0                      first x coordinate
 * @param       y0                      first y coordinate
 * @param       x1                      second x coordinate
 * @param       y1                      second y coordinate
 * @param       endcap0         0 = none, 1 = single pixel, 2 = full cap
 * @param       endcap1         0 = none, 1 = single pixel, 2 = full cap
 * @param       mode            0 = black outline, white body, 1 = white outline, black body
 * @param       mmode           0 = clear, 1 = set, 2 = toggle
 */
void osdDrawLine_outlined(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t endcap0, uint8_t endcap1, uint8_t mode, uint8_t mmode)
{
	// Based on http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	// This could be improved for speed.
	uint8_t omode, imode;
	if (mode == 0)
	{
		omode = 0;
		imode = 1;
	}
	else
	{
		omode = 1;
		imode = 0;
	}

	uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep)
	{
		SWAP(x0, y0);
		SWAP(x1, y1);
	}
	if (x0 > x1)
	{
		SWAP(x0, x1);
		SWAP(y0, y1);
	}

	int16_t deltax = x1 - x0;
	int16_t deltay = abs(y1 - y0);
	int16_t error = deltax / 2;
	int16_t ystep;
	int16_t y = y0;
	int16_t x;

	if (y0 < y1)
		ystep = 1;
	else
		ystep = -1;

	// Draw the outline.
	for (x = x0; x < x1; x++)
	{
		if (steep)
		{
			osdDrawPixel_lm(y - 1, x, mmode, omode);
			osdDrawPixel_lm(y + 1, x, mmode, omode);
			osdDrawPixel_lm(y, x - 1, mmode, omode);
			osdDrawPixel_lm(y, x + 1, mmode, omode);
		}
		else
		{
			osdDrawPixel_lm(x - 1, y, mmode, omode);
			osdDrawPixel_lm(x + 1, y, mmode, omode);
			osdDrawPixel_lm(x, y - 1, mmode, omode);
			osdDrawPixel_lm(x, y + 1, mmode, omode);
		}
		error -= deltay;
		if (error < 0)
		{
			y += ystep;
			error += deltax;
		}
	}
	// Now draw the innards.
	error = deltax / 2;
	y = y0;
	for (x = x0; x < x1; x++)
	{
		if (steep)
		{
			osdDrawPixel_lm(y, x, mmode, imode);
		}
		else
		{
			osdDrawPixel_lm(x, y, mmode, imode);
		}
		error -= deltay;
		if (error < 0)
		{
			y += ystep;
			error += deltax;
		}
	}
}

/**
 * write_word_misaligned: Write a misaligned word across two addresses
 * with an x offset.
 *
 * This allows for many pixels to be set in one write.
 *
 * @param       buff    buffer to write in
 * @param       word    word to write (16 bits)
 * @param       addr    address of first word
 * @param       xoff    x offset (0-15)
 * @param       mode    0 = clear, 1 = set, 2 = toggle
 */
void write_word_misaligned(uint8_t *buff, uint16_t word, unsigned int addr,
		unsigned int xoff, int mode)
{
	uint16_t firstmask = word >> xoff;
	uint16_t lastmask = word << (16 - xoff);
	WRITE_WORD_MODE(buff, addr+1, firstmask && 0x00ff, mode);
	WRITE_WORD_MODE(buff, addr, (firstmask & 0xff00) >> 8, mode);
	if (xoff > 0)
		WRITE_WORD_MODE(buff, addr+2, (lastmask & 0xff00) >> 8, mode);
}

/**
 * write_word_misaligned_NAND: Write a misaligned word across two addresses
 * with an x offset, using a NAND mask.
 *
 * This allows for many pixels to be set in one write.
 *
 * @param       buff    buffer to write in
 * @param       word    word to write (16 bits)
 * @param       addr    address of first word
 * @param       xoff    x offset (0-15)
 *
 * This is identical to calling write_word_misaligned with a mode of 0 but
 * it doesn't go through a lot of switch logic which slows down text writing
 * a lot.
 */
void write_word_misaligned_NAND(uint8_t *buff, uint16_t word, unsigned int addr,
		unsigned int xoff)
{
	uint16_t firstmask = word >> xoff;
	uint16_t lastmask = word << (16 - xoff);
	WRITE_WORD_NAND(buff, addr+1, firstmask & 0x00ff);
	WRITE_WORD_NAND(buff, addr, (firstmask & 0xff00) >> 8);
	if (xoff > 0)
		WRITE_WORD_NAND(buff, addr+2, (lastmask & 0xff00) >> 8);
}

/**
 * write_word_misaligned_OR: Write a misaligned word across two addresses
 * with an x offset, using an OR mask.
 *
 * This allows for many pixels to be set in one write.
 *
 * @param       buff    buffer to write in
 * @param       word    word to write (16 bits)
 * @param       addr    address of first word
 * @param       xoff    x offset (0-15)
 *
 * This is identical to calling write_word_misaligned with a mode of 1 but
 * it doesn't go through a lot of switch logic which slows down text writing
 * a lot.
 */
void write_word_misaligned_OR(uint8_t *buff, uint16_t word, unsigned int addr, unsigned int xoff)
{
	uint16_t firstmask = word >> xoff;
	uint16_t lastmask = word << (16 - xoff);
	WRITE_WORD_OR(buff, addr+1, firstmask & 0x00ff);
	WRITE_WORD_OR(buff, addr, (firstmask & 0xff00) >> 8);
	if (xoff > 0)
		WRITE_WORD_OR(buff, addr + 2, (lastmask & 0xff00) >> 8);

}

/**
 * write_word_misaligned_lm: Write a misaligned word across two
 * words, in both level and mask buffers. This is core to the text
 * writing routines.
 *
 * @param       buff    buffer to write in
 * @param       word    word to write (16 bits)
 * @param       addr    address of first word
 * @param       xoff    x offset (0-15)
 * @param       lmode   0 = clear, 1 = set, 2 = toggle
 * @param       mmode   0 = clear, 1 = set, 2 = toggle
 */
void write_word_misaligned_lm(uint16_t wordl, uint16_t wordm, unsigned int addr,
		unsigned int xoff, int lmode, int mmode)
{
	write_word_misaligned(draw_buffer_level, wordl, addr, xoff, lmode);
	write_word_misaligned(draw_buffer_mask, wordm, addr, xoff, mmode);
}

/**
 * fetch_font_info: Fetch font info structs.
 *
 * @param       ch              character
 * @param       font    font id
 */
int fetch_font_info(uint8_t ch, int font, struct FontEntry *font_info,
		char *lookup)
{
	// First locate the font struct.
	if (font > SIZEOF_ARRAY(fonts))
		return 0; // font does not exist, exit.*/
	// Load the font info; IDs are always sequential.
	*font_info = fonts[font];
	// Locate character in font lookup table. (If required.)
	if (lookup != NULL)
	{
		*lookup = font_info->lookup[ch];
		if (*lookup == 0xff)
			return 0; // character doesn't exist, don't bother writing it.
	}
	return 1;
}

/**
 * write_char16: Draw a character on the current draw buffer.
 * Currently supports outlined characters and characters with
 * a width of up to 8 pixels.
 *
 * @param       ch              character to write
 * @param       x               x coordinate (left)
 * @param       y               y coordinate (top)
 * @param       flags   flags to write with (see gfx.h)
 * @param       font    font to use
 */
void osdWriteChar16(char ch, uint16_t x, uint16_t y, uint8_t font)
{
	APPLY_DEADBAND(x, y);

	uint16_t yy, addr_temp, row, row_temp, xshift;
	uint16_t and_mask, or_mask, level_bits;

	struct FontEntry font_info;
	//char lookup = 0;

	fetch_font_info(0, font, &font_info, NULL);

	// Compute starting address (for x,y) of character.
	uint16_t addr = CALC_BUFF_ADDR(x, y);
	uint16_t wbit = CALC_BIT_IN_WORD(x);

	// If font only supports lowercase or uppercase, make the letter
	// lowercase or uppercase.
	// How big is the character? We handle characters up to 8 pixels
	// wide for now. Support for large characters may be added in future.
	{
		// Ensure we don't overflow.
		if (x + wbit > GRAPHICS_WIDTH_REAL) return;

		// Load data pointer.
		row = ch * font_info.height;
		row_temp = row;
		addr_temp = addr;
		xshift = 16 - font_info.width;
		// We can write mask words easily.
		for (yy = y; yy < y + font_info.height; yy++)
		{
			if (font == 3)
				write_word_misaligned_OR(draw_buffer_mask,
						font_mask12x18[row] << xshift, addr, wbit);
			else
				write_word_misaligned_OR(draw_buffer_mask,
						font_mask8x10[row] << xshift, addr, wbit);
			addr += GRAPHICS_WIDTH_REAL / 8;
			row++;
		}
		// Level bits are more complicated. We need to set or clear
		// level bits, but only where the mask bit is set; otherwise,
		// we need to leave them alone. To do this, for each word, we
		// construct an AND mask and an OR mask, and apply each individually.
		row = row_temp;
		addr = addr_temp;
		for (yy = y; yy < y + font_info.height; yy++)
		{
			if (font == 3)
			{
				level_bits = font_frame12x18[row];
				//if(!(flags & FONT_INVERT)) // data is normally inverted
				level_bits = ~level_bits;
				or_mask = font_mask12x18[row] << xshift;
				and_mask = (font_mask12x18[row] & level_bits) << xshift;
			}
			else
			{
				level_bits = font_frame8x10[row];
				//if(!(flags & FONT_INVERT)) // data is normally inverted
				level_bits = ~level_bits;
				or_mask = font_mask8x10[row] << xshift;
				and_mask = (font_mask8x10[row] & level_bits) << xshift;
			}
			write_word_misaligned_OR(draw_buffer_level, or_mask, addr, wbit);
			// If we're not bold write the AND mask.
			//if(!(flags & FONT_BOLD))
			write_word_misaligned_NAND(draw_buffer_level, and_mask, addr, wbit);
			addr += GRAPHICS_WIDTH_REAL / 8;
			row++;
		}
	}
}

/**
 * write_char: Draw a character on the current draw buffer.
 * Currently supports outlined characters and characters with
 * a width of up to 8 pixels.
 *
 * @param       ch              character to write
 * @param       x               x coordinate (left)
 * @param       y               y coordinate (top)
 * @param       flags   flags to write with (see gfx.h)
 * @param       font    font to use
 */
void osdWriteChar(char ch, uint16_t x, uint16_t y, uint8_t flags, uint8_t font)
{
	APPLY_DEADBAND(x, y);

	uint16_t yy, addr_temp, row, row_temp, xshift;
	uint16_t and_mask, or_mask, level_bits;
	struct FontEntry font_info;
	char lookup = 0;

	fetch_font_info(ch, font, &font_info, &lookup);

	// Compute starting address (for x,y) of character.
	uint16_t addr = CALC_BUFF_ADDR(x, y);
	uint16_t wbit = CALC_BIT_IN_WORD(x);

	// If font only supports lowercase or uppercase, make the letter
	// lowercase or uppercase.
	/*
	 if(font_info.flags & FONT_LOWERCASE_ONLY)
	 ch = tolower(ch);
	 if(font_info.flags & FONT_UPPERCASE_ONLY)
	 ch = toupper(ch);
	 */

	// How big is the character? We handle characters up to 8 pixels
	// wide for now. Support for large characters may be added in future.
	if (font_info.width <= 8)
	{
		// Ensure we don't overflow.
		if (x + wbit > GRAPHICS_WIDTH_REAL) return;

		// Load data pointer.
		row = lookup * font_info.height * 2;
		row_temp = row;
		addr_temp = addr;
		xshift = 16 - font_info.width;

		// We can write mask words easily.
		for (yy = y; yy < y + font_info.height; yy++)
		{
			write_word_misaligned_OR(draw_buffer_mask, font_info.data[row] << xshift, addr, wbit);
			addr += GRAPHICS_WIDTH_REAL / 8;
			row++;
		}

		// Level bits are more complicated. We need to set or clear
		// level bits, but only where the mask bit is set; otherwise,
		// we need to leave them alone. To do this, for each word, we
		// construct an AND mask and an OR mask, and apply each individually.
		row = row_temp;
		addr = addr_temp;
		for (yy = y; yy < y + font_info.height; yy++)
		{
			level_bits = font_info.data[row + font_info.height];
			if (!(flags & FONT_INVERT)) // data is normally inverted
				level_bits = ~level_bits;
			or_mask = font_info.data[row] << xshift;
			and_mask = (font_info.data[row] & level_bits) << xshift;
			write_word_misaligned_OR(draw_buffer_level, or_mask, addr, wbit);

			// If we're not bold write the AND mask.
			//if(!(flags & FONT_BOLD))
			write_word_misaligned_NAND(draw_buffer_level, and_mask, addr, wbit);
			addr += GRAPHICS_WIDTH_REAL / 8;
			row++;
		}
	}
}

/**
 * calc_text_dimensions: Calculate the dimensions of a
 * string in a given font. Supports new lines and
 * carriage returns in text.
 *
 * @param       str                     string to calculate dimensions of
 * @param       font_info       font info structure
 * @param       xs                      horizontal spacing
 * @param       ys                      vertical spacing
 * @param       dim                     return result: struct FontDimensions
 */
void calc_text_dimensions(char *str, struct FontEntry font, int xs, int ys,
		struct FontDimensions *dim)
{
	int max_length = 0, line_length = 0, lines = 1;
	while (*str != 0)
	{
		line_length++;
		if (*str == '\n' || *str == '\r')
		{
			if (line_length > max_length)
				max_length = line_length;
			line_length = 0;
			lines++;
		}
		str++;
	}
	if (line_length > max_length)
		max_length = line_length;
	dim->width = max_length * (font.width + xs);
	dim->height = lines * (font.height + ys);
}

/**
 * write_string: Draw a string on the screen with certain alignment parameters.
 *
 * @param       str             string to write
 * @param       x               x coordinate
 * @param       y               y coordinate
 * @param       xs              horizontal spacing
 * @param       ys              vertical spacing
 * @param       va              vertical align
 * @param       ha              horizontal align
 * @param       flags   		flags (passed to write_char)
 * @param       font    		font
 */
void osdWriteString(char *str, uint16_t x, uint16_t y, uint16_t xs,
		uint16_t ys, uint8_t va, uint8_t ha, uint8_t flags, uint8_t font)
{
	uint16_t xx = 0, yy = 0, xx_original = 0;
	struct FontEntry font_info;
	struct FontDimensions dim;

	// Determine font info and dimensions/position of the string.
	fetch_font_info(0, font, &font_info, NULL);
	calc_text_dimensions(str, font_info, xs, ys, &dim);

	switch (va)
	{
	case TEXT_VA_TOP:
		yy = y;
		break;
	case TEXT_VA_MIDDLE:
		yy = y - (dim.height / 2);
		break;
	case TEXT_VA_BOTTOM:
		yy = y - dim.height;
		break;
	}
	switch (ha)
	{
	case TEXT_HA_LEFT:
		xx = x;
		break;
	case TEXT_HA_CENTER:
		xx = x - (dim.width / 2);
		break;
	case TEXT_HA_RIGHT:
		xx = x - dim.width;
		break;
	}
	// Then write each character.
	xx_original = xx;
	while (*str != 0)
	{
		if (*str == '\n' || *str == '\r')
		{
			yy += ys + font_info.height;
			xx = xx_original;
		}
		else
		{
			if (xx >= 0 && xx < GRAPHICS_RIGHT)
			{
				if (font_info.id < 2)
					osdWriteChar(*str, xx, yy, flags, font);
				else
					osdWriteChar16(*str, xx, yy, font);
			}
			xx += font_info.width + xs;
		}
		str++;
	}
}

/**
 * write_string_formatted: Draw a string with format escape
 * sequences in it. Allows for complex text effects.
 *
 * @param       str             string to write (with format data)
 * @param       x               x coordinate
 * @param       y               y coordinate
 * @param       xs              default horizontal spacing
 * @param       ys              default horizontal spacing
 * @param       va              vertical align
 * @param       ha              horizontal align
 * @param       flags   flags (passed to write_char)
 */
void write_string_formatted(char *str, unsigned int x, unsigned int y,
		unsigned int xs, unsigned int ys, int va, int ha, int flags)
{
	int fcode = 0, fptr = 0, font = 0, fwidth = 0, fheight = 0, xx = x, yy = y,
			max_xx = 0, max_height = 0;
	struct FontEntry font_info;
	// Retrieve sizes of the fonts: bigfont and smallfont.
	fetch_font_info(0, 0, &font_info, NULL);
	int smallfontwidth = font_info.width, smallfontheight = font_info.height;
	fetch_font_info(0, 1, &font_info, NULL);
	int bigfontwidth = font_info.width, bigfontheight = font_info.height;
	// 11 byte stack with last byte as NUL.
	char fstack[11];
	fstack[10] = '\0';
	// First, we need to parse the string for format characters and
	// work out a bounding box. We'll parse again for the final output.
	// This is a simple state machine parser.
	char *ostr = str;
	while (*str)
	{
		if (*str == '<' && fcode == 1) // escape code: skip
			fcode = 0;
		if (*str == '<' && fcode == 0) // begin format code?
		{
			fcode = 1;
			fptr = 0;
		}
		if (*str == '>' && fcode == 1)
		{
			fcode = 0;
			if (strcmp(fstack, "B")) // switch to "big" font (font #1)
			{
				fwidth = bigfontwidth;
				fheight = bigfontheight;
			}
			else if (strcmp(fstack, "S")) // switch to "small" font (font #0)
			{
				fwidth = smallfontwidth;
				fheight = smallfontheight;
			}
			if (fheight > max_height)
				max_height = fheight;
			// Skip over this byte. Go to next byte.
			str++;
			continue;
		}
		if (*str != '<' && *str != '>' && fcode == 1)
		{
			// Add to the format stack (up to 10 bytes.)
			if (fptr > 10) // stop adding bytes
			{
				str++; // go to next byte
				continue;
			}
			fstack[fptr++] = *str;
			fstack[fptr] = '\0'; // clear next byte (ready for next char or to terminate string.)
		}
		if (fcode == 0)
		{
			// Not a format code, raw text.
			xx += fwidth + xs;
			if (*str == '\n')
			{
				if (xx > max_xx)
					max_xx = xx;
				xx = x;
				yy += fheight + ys;
			}
		}
		str++;
	}
	// Reset string pointer.
	str = ostr;
	// Now we've parsed it and got a bbox, we need to work out the dimensions of it
	// and how to align it.
	/*int width = max_xx - x;
	 int height = yy - y;
	 int ay, ax;
	 switch(va)
	 {
	 case TEXT_VA_TOP:               ay = yy; break;
	 case TEXT_VA_MIDDLE:    ay = yy - (height / 2); break;
	 case TEXT_VA_BOTTOM:    ay = yy - height; break;
	 }
	 switch(ha)
	 {
	 case TEXT_HA_LEFT:              ax = x; break;
	 case TEXT_HA_CENTER:    ax = x - (width / 2); break;
	 case TEXT_HA_RIGHT:             ax = x - width; break;
	 }*/
	// So ax,ay is our new text origin. Parse the text format again and paint
	// the text on the display.
	fcode = 0;
	fptr = 0;
	font = 0;
	xx = 0;
	yy = 0;
	while (*str)
	{
		if (*str == '<' && fcode == 1) // escape code: skip
			fcode = 0;
		if (*str == '<' && fcode == 0) // begin format code?
		{
			fcode = 1;
			fptr = 0;
		}
		if (*str == '>' && fcode == 1)
		{
			fcode = 0;
			if (strcmp(fstack, "B")) // switch to "big" font (font #1)
			{
				fwidth = bigfontwidth;
				fheight = bigfontheight;
				font = 1;
			}
			else if (strcmp(fstack, "S")) // switch to "small" font (font #0)
			{
				fwidth = smallfontwidth;
				fheight = smallfontheight;
				font = 0;
			}
			// Skip over this byte. Go to next byte.
			str++;
			continue;
		}
		if (*str != '<' && *str != '>' && fcode == 1)
		{
			// Add to the format stack (up to 10 bytes.)
			if (fptr > 10) // stop adding bytes
			{
				str++; // go to next byte
				continue;
			}
			fstack[fptr++] = *str;
			fstack[fptr] = '\0'; // clear next byte (ready for next char or to terminate string.)
		}
		if (fcode == 0)
		{
			// Not a format code, raw text. So we draw it.
			// TODO - different font sizes.
			osdWriteChar(*str, xx, yy + (max_height - fheight), flags, font);
			xx += fwidth + xs;
			if (*str == '\n')
			{
				if (xx > max_xx)
					max_xx = xx;
				xx = x;
				yy += fheight + ys;
			}
		}
		str++;
	}
}

void osdDrawCharacter(uint8_t c, uint8_t font)
{
	return;
}

void osdDrawDecimal(uint8_t font, int32_t value, uint8_t numberLength, uint8_t zeroPadded, uint8_t decimalPos)
{
    int32_t tmpval;
    int32_t tmpdiv;
    uint8_t i, zero = 0;
    const uint32_t powers[] = { 0, 1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000 };

    if (value >= 0) {
        osdDrawCharacter(' ', font);
    } else {
        value = -value;
        osdDrawCharacter('-', font);
    }

    tmpval = value;

    for (i = numberLength; i > 0; i--) {
        tmpdiv = tmpval / powers[i];
        tmpval = (value % powers[i]);
        // if (tmpdiv || zeroPadded || decimalPos == i - 1)
        if (tmpdiv || zeroPadded || decimalPos == i - 1 || (tmpdiv == 0 && zero)) {
            osdDrawCharacter(tmpdiv + '0', font);
            zero = 1;
        }
        if (decimalPos == i - 1)
            osdDrawCharacter('.', font);

    }
}

// graphics

void drawAttitude(uint16_t x, uint16_t y, int16_t pitch, int16_t roll, uint16_t size)
{
	int16_t a = iSin(roll + 360);
	int16_t b = iCos(roll + 360);
	int16_t c = iSin(roll + 90 + 360) * 5 / 100;
	int16_t d = iCos(roll + 90 + 360) * 5 / 100;

	int16_t k;
	int16_t l;

	int16_t indi30x1 = iCos(30) * (size / 2 + 1) / 100;
	int16_t indi30y1 = iSin(30) * (size / 2 + 1) / 100;

	int16_t indi30x2 = iCos(30) * (size / 2 + 4) / 100;
	int16_t indi30y2 = iSin(30) * (size / 2 + 4) / 100;

	int16_t indi60x1 = iCos(60) * (size / 2 + 1) / 100;
	int16_t indi60y1 = iSin(60) * (size / 2 + 1) / 100;

	int16_t indi60x2 = iCos(60) * (size / 2 + 4) / 100;
	int16_t indi60y2 = iSin(60) * (size / 2 + 4) / 100;

	pitch = pitch % 90;
	if (pitch > 90)
	{
		pitch = pitch - 90;
	}
	if (pitch < -90)
	{
		pitch = pitch + 90;
	}
	a = (a * (size / 2)) / 100;
	b = (b * (size / 2)) / 100;

	if (roll < -90 || roll > 90)
		pitch = pitch * -1;
	k = a * pitch / 90;
	l = b * pitch / 90;

	// scale
	//0
	//drawLine((x)-1-(size/2+4), (y)-1, (x)-1 - (size/2+1), (y)-1);
	//drawLine((x)-1+(size/2+4), (y)-1, (x)-1 + (size/2+1), (y)-1);
	osdDrawLine_outlined((x) - 1 - (size / 2 + 4), (y) - 1,
			(x) - 1 - (size / 2 + 1), (y) - 1, 0, 0, 0, 1);
	osdDrawLine_outlined((x) - 1 + (size / 2 + 4), (y) - 1,
			(x) - 1 + (size / 2 + 1), (y) - 1, 0, 0, 0, 1);

	//30
	//drawLine((x)-1+indi30x1, (y)-1-indi30y1, (x)-1 + indi30x2, (y)-1 - indi30y2);
	//drawLine((x)-1-indi30x1, (y)-1-indi30y1, (x)-1 - indi30x2, (y)-1 - indi30y2);
	osdDrawLine_outlined((x) - 1 + indi30x1, (y) - 1 - indi30y1,
			(x) - 1 + indi30x2, (y) - 1 - indi30y2, 0, 0, 0, 1);
	osdDrawLine_outlined((x) - 1 - indi30x1, (y) - 1 - indi30y1,
			(x) - 1 - indi30x2, (y) - 1 - indi30y2, 0, 0, 0, 1);
	//60
	//drawLine((x)-1+indi60x1, (y)-1-indi60y1, (x)-1 + indi60x2, (y)-1 - indi60y2);
	//drawLine((x)-1-indi60x1, (y)-1-indi60y1, (x)-1 - indi60x2, (y)-1 - indi60y2);
	osdDrawLine_outlined((x) - 1 + indi60x1, (y) - 1 - indi60y1,
			(x) - 1 + indi60x2, (y) - 1 - indi60y2, 0, 0, 0, 1);
	osdDrawLine_outlined((x) - 1 - indi60x1, (y) - 1 - indi60y1,
			(x) - 1 - indi60x2, (y) - 1 - indi60y2, 0, 0, 0, 1);
	//90
	//drawLine((x)-1, (y)-1-(size/2+4), (x)-1, (y)-1 - (size/2+1));
	osdDrawLine_outlined((x) - 1, (y) - 1 - (size / 2 + 4), (x) - 1,
			(y) - 1 - (size / 2 + 1), 0, 0, 0, 1);

	//roll
	//drawLine((x)-1 - b, (y)-1 + a, (x)-1 + b, (y)-1 - a); //Direction line
	osdDrawLine_outlined((x) - 1 - b, (y) - 1 + a, (x) - 1 + b, (y) - 1 - a, 0,
			0, 0, 1); //Direction line
	//"wingtips"
	//drawLine((x)-1 - b, (y)-1 + a, (x)-1 - b + d, (y)-1 + a - c);
	//drawLine((x)-1 + b + d, (y)-1 - a - c, (x)-1 + b, (y)-1 - a);
	osdDrawLine_outlined((x) - 1 - b, (y) - 1 + a, (x) - 1 - b + d,
			(y) - 1 + a - c, 0, 0, 0, 1);
	osdDrawLine_outlined((x) - 1 + b + d, (y) - 1 - a - c, (x) - 1 + b,
			(y) - 1 - a, 0, 0, 0, 1);

	//pitch
	//drawLine((x)-1, (y)-1, (x)-1 - k, (y)-1 - l);
	osdDrawLine_outlined((x) - 1, (y) - 1, (x) - 1 - k, (y) - 1 - l, 0, 0, 0, 1);

	//drawCircle(x-1, y-1, 5);
	//write_circle_outlined(x-1, y-1, 5,0,0,0,1);
	//drawCircle(x-1, y-1, size/2+4);
	//write_circle_outlined(x-1, y-1, size/2+4,0,0,0,1);
}

void drawBattery(uint16_t x, uint16_t y, uint8_t battery, uint16_t size)
{
	int i = 0;
	int batteryLines;
	//top
	/*drawLine((x)-1+(size/2-size/4), (y)-1, (x)-1 + (size/2+size/4), (y)-1);
	 drawLine((x)-1+(size/2-size/4), (y)-1+1, (x)-1 + (size/2+size/4), (y)-1+1);

	 drawLine((x)-1, (y)-1+2, (x)-1 + size, (y)-1+2);
	 //bottom
	 drawLine((x)-1, (y)-1+size*3, (x)-1 + size, (y)-1+size*3);
	 //left
	 drawLine((x)-1, (y)-1+2, (x)-1, (y)-1+size*3);

	 //right
	 drawLine((x)-1+size, (y)-1+2, (x)-1+size, (y)-1+size*3);*/

	write_rectangle_outlined((x) - 1, (y) - 1 + 2, size, size * 3, 0, 1);
	osdDrawVline_lm((x) - 1 + (size / 2 + size / 4) + 1, (y) - 2, (y) - 1 + 1, 0,
			1);
	osdDrawVline_lm((x) - 1 + (size / 2 - size / 4) - 1, (y) - 2, (y) - 1 + 1, 0,
			1);
	osdDrawHline_lm((x) - 1 + (size / 2 - size / 4),
			(x) - 1 + (size / 2 + size / 4), (y) - 2, 0, 1);
	osdDrawHline_lm((x) - 1 + (size / 2 - size / 4),
			(x) - 1 + (size / 2 + size / 4), (y) - 1, 1, 1);
	osdDrawHline_lm((x) - 1 + (size / 2 - size / 4),
			(x) - 1 + (size / 2 + size / 4), (y) - 1 + 1, 1, 1);

	batteryLines = battery * (size * 3 - 2) / 100;
	for (i = 0; i < batteryLines; i++)
	{
		osdDrawHline_lm((x) - 1, (x) - 1 + size, (y) - 1 + size * 3 - i, 1, 1);
	}
}

/*
 void drawAltitude(uint16_t x, uint16_t y, int16_t alt, uint8_t dir) {

 char temp[9]={0};
 char updown=' ';
 uint16_t charx=x/16;
 if(dir==0)
 updown=24;
 if(dir==1)
 updown=25;
 sprintf(temp,"%c%6dm",updown,alt);
 printTextFB(charx,y+2,temp);
 // frame
 drawBox(charx*16-3,y,charx*16+strlen(temp)*8+3,y+11);
 }*/

/**
 * osdHudDrawVscale: Draw a vertical scale.
 *
 * @param       v               value to display as an integer
 * @param       range           range about value to display (+/- range/2 each direction)
 * @param       halign          horizontal alignment: -1 = left, +1 = right.
 * @param       x               x displacement (typ. 0)
 * @param       y               y displacement (typ. half display height)
 * @param       height          height of scale
 * @param       mintick_step    how often a minor tick is shown
 * @param       majtick_step    how often a major tick is shown
 * @param       mintick_len     minor tick length
 * @param       majtick_len     major tick length
 * @param       boundtick_len	boundary tick length
 * @param       max_val         maximum expected value (used to compute size of arrow ticker)
 * @param       flags           special flags (see hud.h.)
 */
void osdHudDrawVscale(int16_t v, int16_t range, int8_t halign, uint16_t x, uint16_t y,
		uint16_t height, uint16_t mintick_step, uint16_t majtick_step, uint16_t mintick_len,
		uint16_t majtick_len, uint16_t boundtick_len, uint16_t max_val, uint16_t flags)
{
	char temp[15];
	struct FontEntry font_info;
	struct FontDimensions dim;
	// Halign should be in a small span.
	//MY_ASSERT(halign >= -1 && halign <= 1);
	// Compute the position of the elements.
	uint16_t majtick_start = 0, majtick_end = 0, mintick_start = 0, mintick_end = 0, boundtick_start = 0, boundtick_end = 0;

	if (halign == HUD_HALIGN_LEFT)
	{
		majtick_start = x;
		majtick_end = x + majtick_len;
		mintick_start = x;
		mintick_end = x + mintick_len;
		boundtick_start = x;
		boundtick_end = x + boundtick_len;
	}
	else if (halign == HUD_HALIGN_RIGHT)
	{
		majtick_start = x;
		majtick_end = x - majtick_len;
		mintick_start = x;
		mintick_end = x - mintick_len;
		boundtick_start = x;
		boundtick_end = x - boundtick_len;
	}

	// Retrieve width of large font (font #0); from this calculate the x spacing.
	fetch_font_info(0, 0, &font_info, NULL);

	int16_t  arrow_len = (font_info.height / 2) + 1; // FIXME, font info being loaded correctly??
	int16_t  text_x_spacing = arrow_len;
	int16_t  max_text_y = 0, text_length = 0;
	int16_t  small_font_char_width = font_info.width + 1; // +1 for horizontal spacing = 1

	// For -(range / 2) to +(range / 2), draw the scale.
	int16_t  range_2 = range / 2; //, height_2 = height / 2;
	int16_t  r = 0, rr = 0, rv = 0, ys = 0, style = 0; //calc_ys = 0,

	// Iterate through each step.
	for (r = -range_2; r <= +range_2; r++)
	{
		style = 0;
		rr = r + range_2 - v;		// normalise range for modulo, subtract value to move ticker tape
		rv = -rr + range_2; 		// for number display

		if (flags & HUD_VSCALE_FLAG_NO_NEGATIVE) rr += majtick_step / 2;

		if (rr % majtick_step == 0) style = 1; 		// major tick
		else if (rr % mintick_step == 0) style = 2; // minor tick
		else style = 0;

		if ((flags & HUD_VSCALE_FLAG_NO_NEGATIVE) && (rv < 0)) continue;

		if (style)
		{
			// Calculate y position.
			ys = ((int32_t) (r * height) / (int32_t) range) + y;

			// Depending on style, draw a minor or a major tick.
			if (style == 1)
			{
				osdDrawHline_outlined(majtick_start, majtick_end, ys, 2, 2, 0, 1);
				memset(temp, ' ', 10);
				sprintf(temp, "%d", rv);
				text_length = (strlen(temp) + 1) * small_font_char_width; // add 1 for margin

				if (text_length > max_text_y) max_text_y = text_length;

				if (halign == HUD_HALIGN_LEFT)
					osdWriteString(temp, majtick_end + text_x_spacing, ys, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_LEFT, 0, 1);
				else
					osdWriteString(temp, majtick_end - text_x_spacing + 1, ys, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_RIGHT, 0, 1);
			}
			else if (style == 2)
				osdDrawHline_outlined(mintick_start, mintick_end, ys, 2, 2, 0, 1);
		}
	}
	// Generate the string for the value, as well as calculating its dimensions.
	memset(temp, ' ', 10);
	sprintf(temp, "%d", v);
	// TODO: add auto-sizing.
	calc_text_dimensions(temp, font_info, 1, 0, &dim);

	int16_t xx = 0, i = 0;

	if (halign == HUD_HALIGN_LEFT) xx = majtick_end + text_x_spacing;
	else xx = majtick_end - text_x_spacing;

	// Draw an arrow from the number to the point.
	for (i = 0; i < arrow_len; i++)
	{
		if (halign == HUD_HALIGN_LEFT)
		{
			osdDrawPixel_lm(xx - arrow_len + i, y - i - 1, 1, 1);
			osdDrawPixel_lm(xx - arrow_len + i, y + i - 1, 1, 1);
			osdDrawHline_lm(xx + dim.width - 1, xx - arrow_len + i + 1, y - i - 1, 0, 1);
			osdDrawHline_lm(xx + dim.width - 1, xx - arrow_len + i + 1, y + i - 1, 0, 1);
		}
		else
		{
			osdDrawPixel_lm(xx + arrow_len - i, y - i - 1, 1, 1);
			osdDrawPixel_lm(xx + arrow_len - i, y + i - 1, 1, 1);
			osdDrawHline_lm(xx - dim.width - 1, xx + arrow_len - i - 1, y - i - 1, 0, 1);
			osdDrawHline_lm(xx - dim.width - 1, xx + arrow_len - i - 1, y + i - 1, 0, 1);
		}
		// FIXME
		// write_hline_lm(xx - dim.width - 1, xx + (arrow_len - i), y - i - 1, 1, 1);
		// write_hline_lm(xx - dim.width - 1, xx + (arrow_len - i), y + i - 1, 1, 1);
	}
	if (halign == HUD_HALIGN_LEFT)
	{
		osdDrawHline_lm(xx, xx + dim.width - 1, y - arrow_len, 1, 1);
		osdDrawHline_lm(xx, xx + dim.width - 1, y + arrow_len - 2, 1, 1);
		osdDrawVline_lm(xx + dim.width - 1, y - arrow_len, y + arrow_len - 2, 1, 1);
	}
	else
	{
		osdDrawHline_lm(xx, xx - dim.width - 1, y - arrow_len, 1, 1);
		osdDrawHline_lm(xx, xx - dim.width - 1, y + arrow_len - 2, 1, 1);
		osdDrawVline_lm(xx - dim.width - 1, y - arrow_len, y + arrow_len - 2, 1, 1);
	}

	// Draw the text.
	if (halign == HUD_HALIGN_LEFT)
		osdWriteString(temp, xx, y, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_LEFT, 0, 0);
	else
		osdWriteString(temp, xx, y, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_RIGHT, 0, 0);

	// Then, add a slow cut off on the edges, so the text doesn't sharply
	// disappear. We simply clear the areas above and below the ticker, and we
	// use little markers on the edges.
	if (halign == HUD_HALIGN_LEFT)
	{
		osdDrawRectangle_filled_lm(
				majtick_end + text_x_spacing,
				y + (height / 2) - (font_info.height / 2),
				max_text_y - boundtick_start, font_info.height, 0, 0);
		osdDrawRectangle_filled_lm(
				majtick_end + text_x_spacing,
				y - (height / 2) - (font_info.height / 2),
				max_text_y - boundtick_start, font_info.height, 0, 0);
	}
	else
	{
		osdDrawRectangle_filled_lm(
				majtick_end - text_x_spacing - max_text_y,
				y + (height / 2) - (font_info.height / 2),
				max_text_y, font_info.height, 0, 0);
		osdDrawRectangle_filled_lm(
				majtick_end - text_x_spacing - max_text_y,
				y - (height / 2) - (font_info.height / 2),
				max_text_y, font_info.height, 0, 0);
	}
	osdDrawHline_outlined(boundtick_start, boundtick_end, y + (height / 2), 2, 2, 0, 1);
	osdDrawHline_outlined(boundtick_start, boundtick_end, y - (height / 2), 2, 2, 0, 1);
}

/**
 * hud_draw_compass: Draw a compass.
 *
 * @param       v                               value for the compass
 * @param       range                   range about value to display (+/- range/2 each direction)
 * @param       width                   length in pixels
 * @param       x                               x displacement (typ. half display width)
 * @param       y                               y displacement (typ. bottom of display)
 * @param       mintick_step    how often a minor tick is shown
 * @param       majtick_step    how often a major tick (heading "xx") is shown
 * @param       mintick_len             minor tick length
 * @param       majtick_len             major tick length
 * @param       flags                   special flags (see hud.h.)
 */
void hud_draw_linear_compass(int v, int range, int width, int x, int y,
		int mintick_step, int majtick_step, int mintick_len, int majtick_len,
		int flags)
{
	v %= 360; // wrap, just in case.
	struct FontEntry font_info;
	int majtick_start = 0, majtick_end = 0, mintick_start = 0, mintick_end = 0,
			textoffset = 0;
	char headingstr[4];
	majtick_start = y;
	majtick_end = y - majtick_len;
	mintick_start = y;
	mintick_end = y - mintick_len;
	textoffset = 8;
	int r, style, rr, xs; // rv,
	int range_2 = range / 2;
	for (r = -range_2; r <= +range_2; r++)
	{
		style = 0;
		rr = (v + r + 360) % 360; // normalise range for modulo, add to move compass track
		//rv = -rr + range_2; // for number display
		if (rr % majtick_step == 0)
			style = 1; // major tick
		else if (rr % mintick_step == 0)
			style = 2; // minor tick
		if (style)
		{
			// Calculate x position.
			xs = ((long int) (r * width) / (long int) range) + x;
			// Draw it.
			if (style == 1)
			{
				osdDrawVline_outlined(xs, majtick_start, majtick_end, 2, 2, 0,
						1);
				// Draw heading above this tick.
				// If it's not one of north, south, east, west, draw the heading.
				// Otherwise, draw one of the identifiers.
				if (rr % 90 != 0)
				{
					// We abbreviate heading to two digits. This has the side effect of being easy to compute.
					headingstr[0] = '0' + (rr / 100);
					headingstr[1] = '0' + ((rr / 10) % 10);
					headingstr[2] = 0;
					headingstr[3] = 0; // nul to terminate
				}
				else
				{
					switch (rr)
					{
					case 0:
						headingstr[0] = 'N';
						break;
					case 90:
						headingstr[0] = 'E';
						break;
					case 180:
						headingstr[0] = 'S';
						break;
					case 270:
						headingstr[0] = 'W';
						break;
					}
					headingstr[1] = 0;
					headingstr[2] = 0;
					headingstr[3] = 0;
				}
				// +1 fudge...!
				osdWriteString(headingstr, xs + 1, majtick_start + textoffset, 1,
						0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, 0, 1);
			}
			else if (style == 2)
				osdDrawVline_outlined(xs, mintick_start, mintick_end, 2, 2, 0,
						1);
		}
	}
	// Then, draw a rectangle with the present heading in it.
	// We want to cover up any other markers on the bottom.
	// First compute font size.
	fetch_font_info(0, 3, &font_info, NULL);
	int text_width = (font_info.width + 1) * 3;
	int rect_width = text_width + 2;
	osdDrawRectangle_filled_lm(x - (rect_width / 2), majtick_start + 2,
			rect_width, font_info.height + 2, 0, 1);
	write_rectangle_outlined(x - (rect_width / 2), majtick_start + 2,
			rect_width, font_info.height + 2, 0, 1);
	headingstr[0] = '0' + (v / 100);
	headingstr[1] = '0' + ((v / 10) % 10);
	headingstr[2] = '0' + (v % 10);
	headingstr[3] = 0;
	osdWriteString(headingstr, x + 1, majtick_start + textoffset + 2, 0, 0,
			TEXT_VA_MIDDLE, TEXT_HA_CENTER, 1, 3);
}
// CORE draw routines end here

void osdDrawArtificialHorizon(float roll, float pitch, int16_t l_x, int16_t l_y, int16_t size)
{
	uint8_t vertical = 0, horizontal = 0;
	int16_t x1, x2;
	int16_t y1, y2;
	int16_t refx = l_x + size / 2;
	int16_t refy = l_y + size / 2;
	float k = 0;
	float alpha = DEG2RAD(roll);
	float dx = sinf(alpha) * (pitch / 90.0f * (size / 2));
	float dy = cosf(alpha) * (pitch / 90.0f * (size / 2));
	int16_t x0 = (size / 2) - dx;
	int16_t y0 = (size / 2) + dy;

	// calculate the line function
	if ((roll != 90) && (roll != -90))
	{
		k = tanf(alpha);
		vertical = 0;
		if (k == 0)
		{
			horizontal = 1;
		}
	}
	else
	{
		vertical = 1;
	}

	// crossing point of line
	if (!vertical && !horizontal)
	{
		// y-y0=k(x-x0)
		int16_t x = 0;
		int16_t y = k * (x - x0) + y0;
		// find right crossing point
		x1 = x;
		y1 = y;
		if (y < 0)
		{
			y1 = 0;
			x1 = ((y1 - y0) + k * x0) / k;
		}
		if (y > size)
		{
			y1 = size;
			x1 = ((y1 - y0) + k * x0) / k;
		}
		// left crossing point
		x = size;
		y = k * (x - x0) + y0;
		x2 = x;
		y2 = y;
		if (y < 0)
		{
			y2 = 0;
			x2 = ((y2 - y0) + k * x0) / k;
		}
		if (y > size)
		{
			y2 = size;
			x2 = ((y2 - y0) + k * x0) / k;
		}
		// move to location
		// horizon line
		osdDrawLine_outlined(x1 + l_x, y1 + l_y, x2 + l_x, y2 + l_y, 0, 0, 0, 1);
		//fill
		if (roll <= 0 && roll > -90)
		{
			//write_string("1", APPLY_HDEADBAND((GRAPHICS_RIGHT/2)),APPLY_VDEADBAND(GRAPHICS_BOTTOM-10), 0, 0, TEXT_VA_BOTTOM, TEXT_HA_CENTER, 0, 3);
			for (int i = y2; i < size; i++)
			{
				x2 = ((i - y0) + k * x0) / k;
				if (x2 > size)
					x2 = size;
				if (x2 < 0)
					x2 = 0;
				osdDrawHline_lm(x2 + l_x, size + l_x, i + l_y, 1, 1);
			}
		}
		else if (roll < -90)
		{
			//write_string("2", APPLY_HDEADBAND((GRAPHICS_RIGHT/2)),APPLY_VDEADBAND(GRAPHICS_BOTTOM-10), 0, 0, TEXT_VA_BOTTOM, TEXT_HA_CENTER, 0, 3);
			for (int i = 0; i < y2; i++)
			{
				x2 = ((i - y0) + k * x0) / k;
				if (x2 > size)
					x2 = size;
				if (x2 < 0)
					x2 = 0;
				osdDrawHline_lm(size + l_x, x2 + l_x, i + l_y, 1, 1);
			}
		}
		else if (roll > 0 && roll < 90)
		{
			//write_string("3", APPLY_HDEADBAND((GRAPHICS_RIGHT/2)),APPLY_VDEADBAND(GRAPHICS_BOTTOM-10), 0, 0, TEXT_VA_BOTTOM, TEXT_HA_CENTER, 0, 3);
			for (int i = y1; i < size; i++)
			{
				x2 = ((i - y0) + k * x0) / k;
				if (x2 > size)
					x2 = size;
				if (x2 < 0)
					x2 = 0;
				osdDrawHline_lm(0 + l_x, x2 + l_x, i + l_y, 1, 1);
			}
		}
		else if (roll > 90)
		{
			//write_string("4", APPLY_HDEADBAND((GRAPHICS_RIGHT/2)),APPLY_VDEADBAND(GRAPHICS_BOTTOM-10), 0, 0, TEXT_VA_BOTTOM, TEXT_HA_CENTER, 0, 3);
			for (int i = 0; i < y1; i++)
			{
				x2 = ((i - y0) + k * x0) / k;
				if (x2 > size)
					x2 = size;
				if (x2 < 0)
					x2 = 0;
				osdDrawHline_lm(x2 + l_x, 0 + l_x, i + l_y, 1, 1);
			}
		}
	}
	else if (vertical)
	{
		// horizon line
		osdDrawLine_outlined(x0 + l_x, 0 + l_y, x0 + l_x, size + l_y, 0, 0, 0,
				1);
		if (roll == 90)
		{
			//write_string("5", APPLY_HDEADBAND((GRAPHICS_RIGHT/2)),APPLY_VDEADBAND(GRAPHICS_BOTTOM-10), 0, 0, TEXT_VA_BOTTOM, TEXT_HA_CENTER, 0, 3);
			for (int i = 0; i < size; i++)
			{
				osdDrawHline_lm(0 + l_x, x0 + l_x, i + l_y, 1, 1);
			}
		}
		else
		{
			//write_string("6", APPLY_HDEADBAND((GRAPHICS_RIGHT/2)),APPLY_VDEADBAND(GRAPHICS_BOTTOM-10), 0, 0, TEXT_VA_BOTTOM, TEXT_HA_CENTER, 0, 3);
			for (int i = 0; i < size; i++)
			{
				osdDrawHline_lm(size + l_x, x0 + l_x, i + l_y, 1, 1);
			}
		}
	}
	else if (horizontal)
	{
		// horizon line
		osdDrawHline_outlined(0 + l_x, size + l_x, y0 + l_y, 0, 0, 0, 1);
		if (roll < 0)
		{
			//write_string("7", APPLY_HDEADBAND((GRAPHICS_RIGHT/2)),APPLY_VDEADBAND(GRAPHICS_BOTTOM-10), 0, 0, TEXT_VA_BOTTOM, TEXT_HA_CENTER, 0, 3);
			for (int i = 0; i < y0; i++)
			{
				osdDrawHline_lm(0 + l_x, size + l_x, i + l_y, 1, 1);
			}
		}
		else
		{
			//write_string("8", APPLY_HDEADBAND((GRAPHICS_RIGHT/2)),APPLY_VDEADBAND(GRAPHICS_BOTTOM-10), 0, 0, TEXT_VA_BOTTOM, TEXT_HA_CENTER, 0, 3);
			for (int i = y0; i < size; i++)
			{
				osdDrawHline_lm(0 + l_x, size + l_x, i + l_y, 1, 1);
			}
		}
	}

	//sides
	osdDrawLine_outlined(l_x, l_y, l_x, l_y + size, 0, 0, 0, 1);
	osdDrawLine_outlined(l_x + size, l_y, l_x + size, l_y + size, 0, 0, 0, 1);
	//plane
	osdDrawLine_outlined(refx - 5, refy, refx + 6, refy, 0, 0, 0, 1);
	osdDrawLine_outlined(refx, refy, refx, refy - 3, 0, 0, 0, 1);
}

void introText(void)
{
	osdWriteString("ARMAZILA", GRAPHICS_RIGHT / 2, 50, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, 0, FONT_12X18);
	osdWriteString("10dM3UOP88 UAV", GRAPHICS_RIGHT / 2, 80, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, 0, FONT_OUTLINED8X14);
	osdWriteString("Flight controller", GRAPHICS_RIGHT / 2, 110, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, 0, FONT_OUTLINED8X14);

	osdWriteString("SW Version: " SW_VERSION, GRAPHICS_RIGHT / 2, 200, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, 0, FONT_OUTLINED8X14);
	osdWriteString("Copyright (C) 2013 Armazila Team", 0, 220, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, FONT_OUTLINED8X14);
}

#if 0
void introGraphics()
{
	/* logo */
	int image=0;
	struct splashEntry splash_info;
	splash_info = splash[image];

	copyimage(APPLY_HDEADBAND(GRAPHICS_RIGHT/2-(splash_info.width)/2), APPLY_VDEADBAND(GRAPHICS_BOTTOM/2-(splash_info.height)/2),image);

	/* frame */
	drawBox(APPLY_HDEADBAND(0),APPLY_VDEADBAND(0),APPLY_HDEADBAND(GRAPHICS_RIGHT-8),APPLY_VDEADBAND(GRAPHICS_BOTTOM));

	// Must mask out last half-word because SPI keeps clocking it out otherwise
	for (uint32_t i = 0; i < 8; i++)
	{
		osdDrawVline( draw_buffer_level,GRAPHICS_WIDTH_REAL-i-1,0,GRAPHICS_HEIGHT_REAL-1,0);
		osdDrawVline( draw_buffer_mask,GRAPHICS_WIDTH_REAL-i-1,0,GRAPHICS_HEIGHT_REAL-1,0);
	}
}
#endif

void calcHomeArrow(int16_t m_yaw)
{
	char temp[50] = { 0 };

	/** http://www.movable-type.co.uk/scripts/latlong.html **/
	float lat1, lat2, lon1, lon2, x, y, brng, u2g;

	// Convert to radians
	lat1 = DEG2RAD(GPS_home[LAT]) / 10000000.0f; // Home lat
	lon1 = DEG2RAD(GPS_home[LON]) / 10000000.0f; // Home lon
	lat2 = DEG2RAD(GPS_coord[LAT]) / 10000000.0f; // UAV lat
	lon2 = DEG2RAD(GPS_coord[LON]) / 10000000.0f; // UAV lon

	// Bearing
	/**
	 var y = Math.sin(dLon) * Math.cos(lat2);
	 var x = Math.cos(lat1)*Math.sin(lat2) -
	 Math.sin(lat1)*Math.cos(lat2)*Math.cos(dLon);
	 var brng = Math.atan2(y, x).toDeg();
	 **/
	y = sinf(lon2 - lon1) * cosf(lat2);
	x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(lon2 - lon1);
	brng = RAD2DEG(atan2f(y,x));
	if (brng < 0) brng += 360;

	// yaw corrected bearing, needs compass
	u2g = brng - 180 - m_yaw;
	if (u2g < 0) u2g += 360;

	// Haversine formula for distance
	/**
	 var R = 6371; // km
	 var dLat = (lat2-lat1).toRad();
	 var dLon = (lon2-lon1).toRad();
	 var a = Math.sin(dLat/2) * Math.sin(dLat/2) +
	 Math.cos(lat1.toRad()) * Math.cos(lat2.toRad()) *
	 Math.sin(dLon/2) * Math.sin(dLon/2);
	 var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
	 var d = R * c;
	 **/
#if 0
	float a, c, d;
	float elevation;
	float gcsAlt = GPS_altitude; // Home MSL altitude
	float uavAlt = GPS_altitude; // UAV MSL altitude
	float dAlt = uavAlt - gcsAlt; // Altitude difference

	a = sinf((lat2 - lat1) / 2) * sinf((lat2 - lat1) / 2)
			+ cosf(lat1) * cosf(lat2) * sinf((lon2 - lon1) / 2)
					* sinf((lon2 - lon1) / 2);
	c = 2 * atan2f(sqrtf(a), sqrtf(1 - a));
	d = 6371 * 1000 * c;

	// Elevation  v depends servo direction
	if (d != 0)
		elevation = 90 - RAD2DEG(atanf(dAlt/d));
	else
		elevation = 0;
	//! TODO: sanity check

	sprintf(temp, "hea:%d", (int) brng);
	osdWriteString(temp, GRAPHICS_RIGHT / 2, 30 +  0, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, 0, 2);
	sprintf(temp, "ele:%d", (int) elevation);
	osdWriteString(temp, GRAPHICS_RIGHT / 2, 30 + 10, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, 0, 2);
	sprintf(temp, "dis:%d", (int) d);
	osdWriteString(temp, GRAPHICS_RIGHT / 2, 30 + 20, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, 0, 2);
	sprintf(temp, "u2g:%d", (int) u2g);
	osdWriteString(temp, GRAPHICS_RIGHT / 2, 30 + 30, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, 0, 2);
#endif
	sprintf(temp, "%c%c", (int) (u2g / 22.5f) * 2 + 0x90, (int) (u2g / 22.5f) * 2 + 0x91);
	osdWriteString(temp, 20, 20, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, 3);
}

void osdScreen0(void)
{
	static uint16_t blinkTimer = 0;
	char temp[50] = { 0 };

	blinkTimer++;
	if (blinkTimer == 50) blinkTimer = 0;

	clearGraphics();

	if (!flag(FLAG_GPS_FIX_HOME) && (blinkTimer < 25))
	{
		osdWriteString("HOME NOT SET", GRAPHICS_RIGHT/2, GRAPHICS_BOTTOM / 2, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, 0, 3);
	}

	sprintf(temp, "LAT:%d", GPS_coord[LAT]);
	osdWriteString(temp, 20, GRAPHICS_BOTTOM - 30, 0, 0, TEXT_VA_BOTTOM, TEXT_HA_LEFT, 0, 3);


	sprintf(temp, "LAT:%d", GPS_coord[LAT]);
	osdWriteString(temp, 20, GRAPHICS_BOTTOM - 10, 0, 0, TEXT_VA_BOTTOM, TEXT_HA_LEFT, 0, 3);

	sprintf(temp, "SAT:%d", (int) GPS_numSat);
	osdWriteString(temp, GRAPHICS_RIGHT - 5, GRAPHICS_BOTTOM - 20, 0, 0, TEXT_VA_TOP, TEXT_HA_RIGHT, 0, 3);

	/* Print ADC voltage FLIGHT*/
	uint16_t batt = batteryAdcToVoltage(adcGetChannel(ADC_VOLTAGE_SENSOR));
	sprintf(temp, "BAT:%d.%dV", batt / 10, batt - ((batt / 10) * 10));
	osdWriteString(temp, GRAPHICS_RIGHT - 5, GRAPHICS_BOTTOM - 40, 0, 0, TEXT_VA_TOP, TEXT_HA_RIGHT, 0, 3);

	if (heading > 180)
		calcHomeArrow((int16_t) (heading - 360));
	else
		calcHomeArrow((int16_t) (heading));
}

void osdScreen1(void)
{
	clearGraphics();
/*
	drawBox(2,2,GRAPHICS_WIDTH_REAL-4,GRAPHICS_HEIGHT_REAL-4);
	osdDrawRectangle_filled(draw_buffer_mask,0,0,GRAPHICS_WIDTH_REAL-2,GRAPHICS_HEIGHT_REAL-2,0);
	osdDrawRectangle_filled(draw_buffer_mask,2,2,GRAPHICS_WIDTH_REAL-4-2,GRAPHICS_HEIGHT_REAL-4-2,2);
	osdDrawRectangle_filled(draw_buffer_mask,3,3,GRAPHICS_WIDTH_REAL-4-1,GRAPHICS_HEIGHT_REAL-4-1,0);

	osdDrawRectangle_filled(draw_buffer_mask,5,5,GRAPHICS_WIDTH_REAL-4-5,GRAPHICS_HEIGHT_REAL-4-5,0);
	write_rectangle_outlined(10,10,GRAPHICS_WIDTH_REAL-20,GRAPHICS_HEIGHT_REAL-20,0,0);
	drawLine(GRAPHICS_WIDTH_REAL-1, GRAPHICS_HEIGHT_REAL-1,(GRAPHICS_WIDTH_REAL/2)-1, GRAPHICS_HEIGHT_REAL-1 );
	drawCircle((GRAPHICS_WIDTH_REAL/2)-1, (GRAPHICS_HEIGHT_REAL/2)-1, (GRAPHICS_HEIGHT_REAL/2)-1);
	drawCircle((GRAPHICS_SIZE/2)-1, (GRAPHICS_SIZE/2)-1, (GRAPHICS_SIZE/2)-2);
	drawLine(0, (GRAPHICS_SIZE/2)-1, GRAPHICS_SIZE-1, (GRAPHICS_SIZE/2)-1);
	drawLine((GRAPHICS_SIZE/2)-1, 0, (GRAPHICS_SIZE/2)-1, GRAPHICS_SIZE-1);
*/
/*
	angleA++;
	if(angleB<=-90)
	{
	sum=2;
	}
	if(angleB>=90)
	{
	sum=-2;
	}
	angleB+=sum;
	angleC+=2;
*/

// GPS HACK
/*
	if (heading > 180)
		calcHomeArrow(heading - 360);
	else
		calcHomeArrow(heading);
*/
	/* Draw Attitude Indicator */
//	drawAttitude(100, 120, angle[PITCH] / 10, angle[ROLL] / 10, 96);
//	write_string("Hello OP-OSD", 60, 12, 1, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, 0);
//	printText16( 60, 12,"Hello OP-OSD");

	char temp[50] = { 0 };

	sprintf(temp, "Lat:%d", GPS_coord[LAT]);
	osdWriteString(temp, 5, 5, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, 2);

	sprintf(temp, "Lon:%d", GPS_coord[LON]);
	osdWriteString(temp, 5, 15, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, 2);

	sprintf(temp, "Fix:%d", (int) flag(FLAG_GPS_FIX));
	osdWriteString(temp, 5, 25, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, 2);

	sprintf(temp, "Sat:%d", (int) GPS_numSat);
	osdWriteString(temp, 5, 35, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, 2);

	/* Print Number of detected video Lines */
	sprintf(temp, "Lines:%4d", osdData.maxScanLine);
	osdWriteString(temp, 5, 45, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, 2);

	/* Print RTC time */
	sprintf(temp, "%02d:%02d:%02d", datetime.hour, datetime.minute, datetime.second);
	osdWriteString(temp, 5, 80, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, 3);

	/* Print CPU temperature */
	sprintf(temp, "Temp:%dC", (uint16_t) (adcGetChannel(ADC_TEMP_SENSOR) * 0.29296875f - 264));
	osdWriteString(temp, 5, GRAPHICS_BOTTOM - 5, 0, 0, TEXT_VA_BOTTOM, TEXT_HA_LEFT, 0, 2);

	/* Print ADC voltage FLIGHT*/
	sprintf(temp, "FltV:%dV", 10);
	osdWriteString(temp,  5, GRAPHICS_BOTTOM - 15, 0, 0, TEXT_VA_BOTTOM, TEXT_HA_LEFT, 0, 2);

	/* Print ADC voltage RSSI */
//sprintf(temp,"Curr:%4dA",(int)(PIOS_ADC_PinGet(0)*300*61/4096));
//write_string(temp, (GRAPHICS_WIDTH_REAL - 2),60, 0, 0, TEXT_VA_TOP, TEXT_HA_RIGHT, 0, 2);
	/* Draw Battery Gauge */
	/*
	 m_batt++;
	 uint8_t dir=3;
	 if(m_batt==101)
	 m_batt=0;
	 if(m_pitch>0)
	 {
	 dir=0;
	 m_alt+=m_pitch/2;
	 }
	 else if(m_pitch<0)
	 {
	 dir=1;
	 m_alt+=m_pitch/2;
	 }
	 */

	/*
	 if(OsdSettings.Battery == OSDSETTINGS_BATTERY_ENABLED)
	 {
	 drawBattery(APPLY_HDEADBAND(OsdSettings.BatterySetup[OSDSETTINGS_BATTERYSETUP_X]),APPLY_VDEADBAND(OsdSettings.BatterySetup[OSDSETTINGS_BATTERYSETUP_Y]),m_batt,16);
	 }
	 */

//drawAltitude(200, 50, m_alt, dir);
//drawArrow(96, GRAPHICS_HEIGHT_REAL / 2, angleB, 32);
// Draw airspeed (left side.)
//		hud_draw_vertical_scale((int) GPS_speed, 100, -1, APPLY_HDEADBAND(10), APPLY_VDEADBAND(10), 100, 10, 20, 7, 12, 15, 1000, HUD_VSCALE_FLAG_NO_NEGATIVE);
// Draw altimeter (right side.)
//		hud_draw_vertical_scale((int) GPS_altitude, 200, +1, APPLY_HDEADBAND(10), APPLY_VDEADBAND(10), 100, 20, 100, 7, 12, 15, 500, 0);
// Draw compass.

	if (heading < 0)
	{
//		hud_draw_linear_compass(360 + heading, 150, 120, GRAPHICS_RIGHT / 2, GRAPHICS_BOTTOM - 5, 15, 30, 7, 12, 0);
	}
	else
	{
//		hud_draw_linear_compass(heading, 150, 120, GRAPHICS_RIGHT / 2, GRAPHICS_BOTTOM - 5, 15, 30, 7, 12, 0);
	}
}

void osdHorizon(void)
{
	clearGraphics();

    uint16_t x_dim = GRAPHICS_RIGHT;	// Number of screen pixels along x axis
    uint16_t y_dim = GRAPHICS_BOTTOM;  	// Number of screen pixels along y axis
    uint16_t L = 79;	              	// Length of main horizon line indicator
    int16_t  L2 = 49;					// Length of sub horizon line indicator
    uint16_t l = 10;                    // Length of small angle indicator lines
    // float theta_max = 42.5 * (M_PI / 180);        // Max pitch angle displayed on screen
    float    theta_max = 23 * (M_PI / 180); // OSD FOV/2
    int16_t  x_c, y_c;
    int16_t  x_a[36] = { 0, };
    int16_t  y_a[36] = { 0, };
    uint8_t  t_a[36] = { 0, };
    uint8_t  a_a[36] = { 0, };
    float    pitch;
    float    roll;
    float    d1, alpha1;
    int16_t  i, idx = 0;
    int16_t  flag = 5;               // was 7
    int16_t  x_c2, y_c2;
    float    cosroll, sinroll;

    pitch = DEG2RAD(angle[PITCH]) / 10;
    roll  = DEG2RAD(angle[ROLL ]) / 10;

    cosroll = cosf(roll);
    sinroll = sinf(roll);

    x_c = x_dim / 2;
    y_c = y_dim / 2 * (1 - pitch / theta_max);
    x_c2 = x_dim / 2 + sinroll * flag;
    y_c2 = y_dim / 2 * (1 - pitch / theta_max) + cosroll * flag;

    for (i = 10; i <= 90; i += 10) {
        d1 = sqrtf(17 * 17 + powf(i * M_PI / 180.0f / theta_max * y_dim / 2, 2));
        alpha1 = atan2f((i * M_PI / 180.0f / theta_max * y_dim / 2), 17);
        // d2 = sqrtf(50 * 50 + powf(i * M_PI / 180 / theta_max * y_dim / 2, 2));
        // alpha2 = atanf((i * M_PI / 180 / theta_max * y_dim / 2) / 50);

        // + i.e. down
        t_a[idx] = 1;
        a_a[idx] = i;
        x_a[idx] = x_dim / 2 + d1 * cosf(alpha1 - roll);
        y_a[idx++] = (y_dim / 2) * (1 - pitch / theta_max) + d1 * sinf(alpha1 - roll);
        t_a[idx] = 1;
        a_a[idx] = i;
        x_a[idx] = x_dim / 2 - d1 * cosf(alpha1 + roll);
        y_a[idx++] = (y_dim / 2) * (1 - pitch / theta_max) + d1 * sinf(alpha1 + roll);
        // - i.e. up
        t_a[idx] = 0;
        a_a[idx] = i;
        x_a[idx] = x_dim / 2 + d1 * cosf(alpha1 + roll);
        y_a[idx++] = (y_dim / 2) * (1 - pitch / theta_max) - d1 * sinf(alpha1 + roll);
        t_a[idx] = 0;
        a_a[idx] = i;
        x_a[idx] = x_dim / 2 - d1 * cosf(alpha1 - roll);
        y_a[idx++] = (y_dim / 2) * (1 - pitch / theta_max) - d1 * sinf(alpha1 - roll);
    }

    for (i = 0; i < 36; i++) {
    	osdDrawLine_lm(x_a[i] - l * cosroll, y_a[i] - l * -sinroll, x_a[i] + l * cosroll, y_a[i] + l * -sinroll, t_a[i], 1);
        if (i % 2) {
//        	char temp[12];
//        	sprintf(temp, "%d", a_a[i]);
//        	write_string(temp, x_a[i], y_a[i] - 4, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, 2);
//            osdSetCursor(x_a[i], y_a[i] - 4);
//            osdDrawDecimal(FONT_8PX_FIXED, a_a[i], 3, 0, -1);
        }
    }

    // cross
    //osdDrawHorizontalLine(x_dim / 2 - 5, y_dim / 2, 11, 1);
    //osdDrawVerticalLine(x_dim / 2, y_dim / 2 - 5, 11, 1);
    //osdDrawCircle(x_dim / 2, y_dim / 2, 39, 1, 0xFF);

    // center
//    osdDrawCircle(x_dim / 2, y_dim / 2, 3, 1, 0xFF);
    osdDrawHline_lm(x_dim / 2 - 7, x_dim / 2 - 3, y_dim / 2, 1, 1);
    osdDrawHline_lm(x_dim / 2 + 3, x_dim / 2 + 7, y_dim / 2, 1, 1);
    osdDrawVline_lm(x_dim / 2, y_dim / 2 - 6, y_dim / 2 - 2, 1, 1);

    // horizont
    osdDrawLine_lm(x_c - L * cosroll, y_c - L * -sinroll, x_c + L * cosroll, y_c + L * -sinroll, 1, 1);

    // sub horizont
    osdDrawLine_lm(x_c2 - L2 * cosroll, y_c2 - L2 * -sinroll, x_c2 + L2 * cosroll, y_c2 + L2 * -sinroll, 1, 1);
}

void osdScreen4(void)
{
	char temp[50] = { 0 };

	clearGraphics();
	// Pixel ratio = y/x 1.56

	// GPS_speed in cm/s -----------------------------------------------------------------------------
	//speed = (uint16_t)(GPS_speed * 0.36f)
	osdHudDrawVscale(34, 50, HUD_HALIGN_RIGHT, 45, GRAPHICS_BOTTOM / 2, 180,
			5, 10, 4, 7, 10, 100, HUD_VSCALE_FLAG_NO_NEGATIVE);

	// EstAlt in cm ----------------------------------------------------------------------------------
	osdHudDrawVscale(EstAlt / 100, 100, HUD_HALIGN_LEFT, 180, GRAPHICS_BOTTOM / 2, 180,
			10, 20, 4, 7, 10, 500, 0);

	// Flag status line ---------------------------------------------------------------------------------
#if 1
	if (flag(FLAG_ARMED))
		osdWriteString("ARM", 0, GRAPHICS_BOTTOM, 0, 0, TEXT_VA_BOTTOM, TEXT_HA_LEFT, 0, FONT_OUTLINED8X14);

	if (flag(FLAG_HORIZON_MODE) && flag(FLAG_ANGLE_MODE))
		osdWriteString("HORIZ", 32, GRAPHICS_BOTTOM, 0, 0, TEXT_VA_BOTTOM, TEXT_HA_LEFT, 0, FONT_OUTLINED8X14);

	if (flag(FLAG_ALTHOLD_MODE))
		osdWriteString("ALTH", 80, GRAPHICS_BOTTOM, 0, 0, TEXT_VA_BOTTOM, TEXT_HA_LEFT, 0, FONT_OUTLINED8X14);

	if (flag(FLAG_MAG_MODE))
		osdWriteString("MAG", 120, GRAPHICS_BOTTOM, 0, 0, TEXT_VA_BOTTOM, TEXT_HA_LEFT, 0, FONT_OUTLINED8X14);

	if (flag(FLAG_GPSHOLD_MODE))
		osdWriteString("GPSH", 152, GRAPHICS_BOTTOM, 0, 0, TEXT_VA_BOTTOM, TEXT_HA_LEFT, 0, FONT_OUTLINED8X14);

	if (flag(FLAG_GPSHOME_MODE))
		osdWriteString("RTH", 192, GRAPHICS_BOTTOM, 0, 0, TEXT_VA_BOTTOM, TEXT_HA_LEFT, 0, FONT_OUTLINED8X14);
#else
	osdWriteString("ARM LEVEL BARO MAG GPSH RTH", 0, GRAPHICS_BOTTOM, 0, 0, TEXT_VA_BOTTOM, TEXT_HA_LEFT, 0, FONT_OUTLINED8X14);
	if (f.ARMED)
		osdDrawRectangle_filled(draw_buffer_level, 0, GRAPHICS_BOTTOM - 14, 24, 14, 2);

	if (flag(FLAG_HORIZON_MODE) && flag(FLAG_ANGLE_MODE))
		osdDrawRectangle_filled(draw_buffer_level, 32, GRAPHICS_BOTTOM - 14, 40, 14, 2);

	if (flag(FLAG_ALTHOLD_MODE))
		osdDrawRectangle_filled(draw_buffer_level, 80, GRAPHICS_BOTTOM - 14, 32, 14, 2);

	if (f.MAG_MODE)
		osdDrawRectangle_filled(draw_buffer_level, 120, GRAPHICS_BOTTOM - 14, 24, 14, 2);

	if (f.GPS_HOLD_MODE)
		osdDrawRectangle_filled(draw_buffer_level, 152, GRAPHICS_BOTTOM - 14, 32, 14, 2);

	if (f.GPS_HOME_MODE)
		osdDrawRectangle_filled(draw_buffer_level, 192, GRAPHICS_BOTTOM - 14, 24, 14, 2);
#endif

	// Status bar 20-37 lines from bottom ---------------------------------------------------------------
#define STATUS_LINE_HEIGHT		18
#define STATUS_LINE_Y0			GRAPHICS_BOTTOM - 20
#define STATUS_LINE_Y1			STATUS_LINE_Y0 - STATUS_LINE_HEIGHT

	// TODO: requires refactoring !!!
	uint8_t events_count = 0;
	uint8_t event_idx[2];
	static char * events[] = {
		"HOME NOT SET",
		"RESET BY WDG",
	};

	if (!flag(FLAG_GPS_FIX_HOME))
	{
		event_idx[events_count] = 0;
		events_count++;
	}

	static uint16_t wdg_event_flag = 250;

	if (flag(FLAG_WDG_OCCURRED) && wdg_event_flag > 0)
	{
		wdg_event_flag--;
		event_idx[events_count] = 1;
		events_count++;
	}

	static uint8_t event_num = 0;
	static uint8_t blinkTimer = 0;

	blinkTimer++;
	if (blinkTimer == 50) {
		blinkTimer = 0;
		event_num++;
		if (event_num >= events_count)
			event_num = 0;
	}

	if (blinkTimer < 25)
	{
		if (!flag(FLAG_GPS_FIX_HOME))
		{
			osdWriteString(events[event_num], GRAPHICS_RIGHT/2, STATUS_LINE_Y0, 0, 0, TEXT_VA_BOTTOM, TEXT_HA_CENTER, 0, FONT_OUTLINED8X14);
		}
	}
	else
	{
		// Clear status line (only mask layer)
		//osdDrawRectangle_filled(draw_buffer_mask, GRAPHICS_LEFT, STATUS_LINE_Y0, GRAPHICS_RIGHT, STATUS_LINE_HEIGHT, 0);
	}

	// Center cross -----------------------------------------------------------------------------------------
    osdDrawHline_lm(GRAPHICS_RIGHT / 2 - 7, GRAPHICS_RIGHT / 2 - 3, GRAPHICS_BOTTOM / 2, 1, 1);
    osdDrawHline_lm(GRAPHICS_RIGHT / 2 + 3, GRAPHICS_RIGHT / 2 + 7, GRAPHICS_BOTTOM / 2, 1, 1);
    osdDrawVline_lm(GRAPHICS_RIGHT / 2, GRAPHICS_BOTTOM / 2 - 11, GRAPHICS_BOTTOM / 2 - 5, 1, 1);
    osdDrawVline_lm(GRAPHICS_RIGHT / 2, GRAPHICS_BOTTOM / 2 + 11, GRAPHICS_BOTTOM / 2 + 5, 1, 1);

    // Horizont ---------------------------------------------------------------------------------------------
#define  GRAPHICS_LINES_NUM   4
	int16_t x = GRAPHICS_RIGHT / 2;
	int16_t y = GRAPHICS_BOTTOM / 2;

    // size = 50;
	//const static s8 gLine[GRAPHICS_LINES_NUM] = { -65, -30, -1, 1, 30, 65 };
	//const static u8 gColor[GRAPHICS_LINES_NUM] = { 0, 0, 0, 1, 1, 1 };
	const static int8_t  gLine[GRAPHICS_LINES_NUM] = { -45, -1, 1, 45 };
	const static uint8_t gColor[GRAPHICS_LINES_NUM] = { 0, 0, 1, 1 };

	uint16_t x1, x2, y1, y2;
	uint16_t aa = angle[ROLL] / 10 + 360;
	uint16_t pp = angle[PITCH] / 10;
	uint8_t n;

	for (n = 0; n < GRAPHICS_LINES_NUM; n++)
	{
		x1 = x - iCos(aa + gLine[n] - pp) / 2;		// optimazed (/ 2) = (* size / 100)
		y1 = y + iSin(aa + gLine[n] - pp) / 1.5;
		x2 = x + iCos(aa - gLine[n] + pp) / 2;
		y2 = y - iSin(aa - gLine[n] + pp) / 1.5;

		osdDrawLine_outlined(x1, y1, x2, y2, 0, 0, 0, gColor[n]);
	}

	// Heading arrow ---------------------------------------------------------------------------------------
	// x (-65...+62), y (-90...+90)
	//osdDrawRectangle_lm(GRAPHICS_RIGHT / 2 - 65, GRAPHICS_BOTTOM / 2 - 90, 127, 180, 1,1);

	int16_t dir = 180 + 360 - _GPS_directionToHome;
	int16_t dis = _GPS_distanceToHome;

	if (dis > 100) dis = 100 + (dis - 100) / 10;
	if (dis > 1000) dis = 1000 + (dis - 1000) / 100;

	x = GRAPHICS_RIGHT / 2 - ((int32_t)iSin(dir) * dis) / 600;
	y = GRAPHICS_BOTTOM / 2 - ((int32_t)iCos(dir) * dis) / 300;

	if (x < GRAPHICS_RIGHT / 2 - 65) x = GRAPHICS_RIGHT / 2 - 65;
	if (x > GRAPHICS_RIGHT / 2 + 62) x = GRAPHICS_RIGHT / 2 + 62;
	if (y < GRAPHICS_BOTTOM / 2 - 90) y = GRAPHICS_BOTTOM / 2 - 90;
	if (y > GRAPHICS_BOTTOM / 2 + 90) y = GRAPHICS_BOTTOM / 2 + 90;

	int16_t h = heading + 360;
	//int16_t h = GPS_ground_course;

	x1 = x + iSin(h) * 10 / 100 / 2;
	y1 = y - iCos(h) * 10 / 100 / 1.5;
	x2 = x + iSin(h + 120) * 10 / 100 / 2;
	y2 = y - iCos(h + 120) * 10 / 100 / 1.5;

	osdDrawLine_outlined(x1, y1, x2, y2, 0, 0, 0, 1);

	//x1 = x + iSin(h) * 10 / 100 / 2;
	//y1 = y - iCos(h) * 10 / 100 / 1.5;
	x2 = x + iSin(h + 240) * 10 / 100 / 2;
	y2 = y - iCos(h + 240) * 10 / 100 / 1.5;

	osdDrawLine_outlined(x1, y1, x2, y2, 0, 0, 0, 1);

    // Print distance to home
	if (_GPS_distanceToHome < 1000)
		sprintf(temp, "%d", _GPS_distanceToHome);
	else
		sprintf(temp, "%d.%d", _GPS_distanceToHome / 1000, (_GPS_distanceToHome / 100) % 10);
	osdWriteString(temp,  x, y + 8, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, 0, FONT_OUTLINED8X8);

	// Telemetry -------------------------------------------------------------------------------------
	/* Print ADC voltage FLIGHT*/
	sprintf(temp, "%d.%d V", vbat / 10, vbat % 10);
	osdWriteString(temp,  5, 0, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, FONT_OUTLINED8X14);
	sprintf(temp, "%d mah", ebat);
	osdWriteString(temp,  5, 16, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, FONT_OUTLINED8X14);
	sprintf(temp, "%02d:%02d:%02d", datetime.hour, datetime.minute, datetime.second);
	osdWriteString(temp,  GRAPHICS_RIGHT - 5, 0, 0, 0, TEXT_VA_TOP, TEXT_HA_RIGHT, 0, FONT_OUTLINED8X14);
	sprintf(temp, "GPS: %d", GPS_numSat);
	osdWriteString(temp,  GRAPHICS_RIGHT - 5, 16, 0, 0, TEXT_VA_TOP, TEXT_HA_RIGHT, 0, FONT_OUTLINED8X14);
}

/***************************************************************************************/
void updateGraphics()
{
	static uint16_t introTimer = 25 * 10;	// Time to shown intro

	if (introTimer > 0 && !flag(FLAG_WDG_OCCURRED))
	{
		introTimer --;
		introText();
		return;
	}

	switch (cfg.osd_screen)
	{
	case 0:
		osdScreen0();
		break;
	case 1:
		osdScreen1();
		break;
	case 2:
		clearGraphics();

		osdDrawArtificialHorizon(-angle[ROLL] / 10, angle[PITCH] / 10, 55, 72, 115);

		// GPS_speed in cm/s
		osdHudDrawVscale((uint16_t)((float)GPS_speed * 0.36f), 20, HUD_HALIGN_RIGHT, 50, GRAPHICS_BOTTOM / 2, 180,
				5, 10, 4, 7, 10, 100, HUD_VSCALE_FLAG_NO_NEGATIVE);

		// EstAlt in cm
		osdHudDrawVscale(EstAlt / 100, 100, HUD_HALIGN_LEFT, 175, GRAPHICS_BOTTOM / 2, 180,
				10, 20, 4, 7, 10, 500, 0);
		break;
	case 3:
		osdHorizon();
		break;
	case 4:
		osdScreen4();
		break;
	case 5:
		// Test 1 - Four rectangle
		osdDrawRectangle_filled_lm(5, 					5, 					  50, 25, 1, 1);
		osdDrawRectangle_filled_lm(5, 					GRAPHICS_BOTTOM - 30, 50, 25, 0, 1);
		osdDrawRectangle_filled_lm(GRAPHICS_RIGHT - 55, 5, 					  50, 25, 0, 1);
		osdDrawRectangle_filled_lm(GRAPHICS_RIGHT - 55, GRAPHICS_BOTTOM - 30, 50, 25, 1, 1);
		break;
	case 6:
		// Test 2 - Scanning vertical line
	{
		static uint16_t pix = GRAPHICS_LEFT;
		char temps[20] = { 0 };

		//osdDrawVline_lm(pix, GRAPHICS_TOP, GRAPHICS_BOTTOM, 1, 1);
		osdDrawRectangle_filled_lm(pix, GRAPHICS_TOP, 3, GRAPHICS_BOTTOM, 1, 1);
		sprintf(temps, "x=%3d", pix);
		osdWriteString(temps, GRAPHICS_RIGHT / 2, GRAPHICS_BOTTOM / 2, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, 0, 3);
		pix ++;
		if (pix == GRAPHICS_RIGHT) pix = GRAPHICS_LEFT;
	}
	break;
	case 7:
		// Test 3 - Fixed vertical line. Use osd_x
		{
			char temps[20] = { 0 };

			//osdDrawVline_lm(pix, GRAPHICS_TOP, GRAPHICS_BOTTOM, 1, 1);
			osdDrawRectangle_filled_lm(osd_x, GRAPHICS_TOP, 3, GRAPHICS_BOTTOM, 1, 1);
			sprintf(temps, "x=%3d", osd_x);
			osdWriteString(temps, GRAPHICS_RIGHT / 2, GRAPHICS_BOTTOM / 2, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, 0, 3);
		}
		break;
	default:
		osdDrawVline_lm(GRAPHICS_RIGHT / 2, 0, GRAPHICS_BOTTOM, 1, 1);
		osdDrawHline_lm(0, GRAPHICS_RIGHT, GRAPHICS_BOTTOM / 2, 1, 1);
		break;
	}
#if 0
	// Must mask out last half-word because SPI keeps clocking it out otherwise
	for (uint32_t i = 0; i < 8; i++)
	{
		osdDrawVline(draw_buffer_level, GRAPHICS_WIDTH_REAL - i - 1, 0, GRAPHICS_HEIGHT_REAL - 1, 0);
		osdDrawVline(draw_buffer_mask, GRAPHICS_WIDTH_REAL - i - 1, 0, GRAPHICS_HEIGHT_REAL - 1, 0);
	}
#endif
}

portTASK_FUNCTION_PROTO(osdTask, pvParameters)
{
	osdInit();

	while (1)
	{
		/*
		 * 	Для OSD используется два буфера. Пока один отображается в другой
		 * 	можно рисовать. Каждые два полукадра буферы меняются местами, при
		 * 	этом взводится семафор osdSemaphore. Начиная с этого момента можно
		 * 	рисовать во второй буфер. Надо следить что бы Время рисования
		 * 	(osdCicleTime) не привышало 40 мс иначе на изображении появятся
		 * 	артифакты.
		 */
		xSemaphoreTake(osdSemaphore, portMAX_DELAY);

		uint32_t sTime, eTime;

		sTime = micros();
		updateGraphics();

		eTime = micros();

		// OSD Cicle Time
		osdCicleTime = (eTime - sTime) * 0.05 + osdCicleTime * 0.95;
		//debug[1] = osdCicleTime;
	}
}
