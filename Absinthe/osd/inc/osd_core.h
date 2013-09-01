/*
 * osd_core.h
 *
 *  Created on: 15.03.2013
 *      Author: avgorbi
 */

#ifndef OSD_CORE_H_
#define OSD_CORE_H_

// First non blank OSD line
#define GRAPHICS_LINE 32

//top/left deadband
#define GRAPHICS_HDEADBAND 18
#define GRAPHICS_VDEADBAND 0

// Real OSD size
#define GRAPHICS_WIDTH_REAL		(230 + GRAPHICS_HDEADBAND)
#define GRAPHICS_HEIGHT_REAL	(270 + GRAPHICS_VDEADBAND)

#define GRAPHICS_WIDTH_REAL_NTSC	(320 + GRAPHICS_HDEADBAND)
#define GRAPHICS_HEIGHT_REAL_NTSC	(225 + GRAPHICS_VDEADBAND)

//draw area
#define GRAPHICS_TOP 			0
#define GRAPHICS_LEFT 			0
#define GRAPHICS_BOTTOM 		(GRAPHICS_HEIGHT_REAL - GRAPHICS_VDEADBAND - 1)
#define GRAPHICS_RIGHT 			(GRAPHICS_WIDTH_REAL - GRAPHICS_HDEADBAND - 1)

#define GRAPHICS_WIDTH 			(GRAPHICS_WIDTH_REAL / 8)	// Use maximum size
#define GRAPHICS_HEIGHT 		GRAPHICS_HEIGHT_REAL		// Use maximum size

// dma lenght
#define BUFFER_LINE_LENGTH      (GRAPHICS_WIDTH)  				//Yes, in bytes.

// line types
enum {
	LINE_TYPE_BLANK		= 0,
	LINE_TYPE_GRAPHICS	= 2
};

// Time vars
typedef struct {
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
} TTime;

// OSD data
typedef struct {
	uint8_t buffer0_level[GRAPHICS_HEIGHT * GRAPHICS_WIDTH];
	uint8_t buffer0_mask[GRAPHICS_HEIGHT * GRAPHICS_WIDTH];
	uint8_t buffer1_level[GRAPHICS_HEIGHT * GRAPHICS_WIDTH];
	uint8_t buffer1_mask[GRAPHICS_HEIGHT * GRAPHICS_WIDTH];
	uint16_t buffer_padding;		// Dummy bytes

    uint16_t currentScanLine;
    uint16_t maxScanLine;
    uint16_t activeScanLine;
    uint16_t Height;                // depend from PAL. OSD_HEIGHT_PAL or OSD_HEIGHT_NTSC
    uint8_t  PAL;                    // PAL - true or NTSC - false
	uint8_t  LineType;				// LINE_TYPE_BLANK or LINE_TYPE_GRAPHICS
} osdData_t;

//! Bitmask for drawing circle octant 0.
#define GFX_OCTANT0     (1 << 0)
//! Bitmask for drawing circle octant 1.
#define GFX_OCTANT1     (1 << 1)
//! Bitmask for drawing circle octant 2.
#define GFX_OCTANT2     (1 << 2)
//! Bitmask for drawing circle octant 3.
#define GFX_OCTANT3     (1 << 3)
//! Bitmask for drawing circle octant 4.
#define GFX_OCTANT4     (1 << 4)
//! Bitmask for drawing circle octant 5.
#define GFX_OCTANT5     (1 << 5)
//! Bitmask for drawing circle octant 6.
#define GFX_OCTANT6     (1 << 6)
//! Bitmask for drawing circle octant 7.
#define GFX_OCTANT7     (1 << 7)
//! Bitmask for drawing circle quadrant 0.
#define GFX_QUADRANT0   (GFX_OCTANT0 | GFX_OCTANT1)
//! Bitmask for drawing circle quadrant 1.
#define GFX_QUADRANT1   (GFX_OCTANT2 | GFX_OCTANT3)
//! Bitmask for drawing circle quadrant 2.
#define GFX_QUADRANT2   (GFX_OCTANT4 | GFX_OCTANT5)
//! Bitmask for drawing circle quadrant 3.
#define GFX_QUADRANT3   (GFX_OCTANT6 | GFX_OCTANT7)

// Macro to swap buffers given a temporary pointer.
#define SWAP_BUFFS(tmp, a, b) { tmp = a; a = b; b = tmp; }

void osdInit(void);

#endif /* OSD_CORE_H_ */
