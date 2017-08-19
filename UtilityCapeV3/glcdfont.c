// This is the 'classic' fixed-space bitmap font for Adafruit_GFX since 1.0.
// See gfxfont.h for newer custom bitmap font info.

#ifndef FONT5X7_H
#define FONT5X7_H

#ifdef __AVR__
 #include <avr/io.h>
 #include <avr/pgmspace.h>
#elif defined(ESP8266)
 #include <pgmspace.h>
#else
 #define PROGMEM
#endif

// Standard ASCII 5x7 font

static const unsigned char font[] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x3E, 0x5B, 0x4F, 0x5B, 0x3E,
	0x3E, 0x6B, 0x4F, 0x6B, 0x3E,
	0x1C, 0x3E, 0x7C, 0x3E, 0x1C,
	0x18, 0x3C, 0x7E, 0x3C, 0x18,
	0x1C, 0x57, 0x7D, 0x57, 0x1C,
	0x1C, 0x5E, 0x7F, 0x5E, 0x1C,
	0x00, 0x18, 0x3C, 0x18, 0x00,
	0xFF, 0xE7, 0xC3, 0xE7, 0xFF,
	0x00, 0x18, 0x24, 0x18, 0x00,
	0xFF, 0xE7, 0xDB, 0xE7, 0xFF,
	0x30, 0x48, 0x3A, 0x06, 0x0E,
	0x26, 0x29, 0x79, 0x29, 0x26,
	0x40, 0x7F, 0x05, 0x05, 0x07,
	0x40, 0x7F, 0x05, 0x25, 0x3F,
	0x5A, 0x3C, 0xE7, 0x3C, 0x5A,
	0x7F, 0x3E, 0x1C, 0x1C, 0x08,
	0x08, 0x1C, 0x1C, 0x3E, 0x7F,
	0x14, 0x22, 0x7F, 0x22, 0x14,
	0x5F, 0x5F, 0x00, 0x5F, 0x5F,
	0x06, 0x09, 0x7F, 0x01, 0x7F,
	0x00, 0x66, 0x89, 0x95, 0x6A,
	0x60, 0x60, 0x60, 0x60, 0x60,
	0x94, 0xA2, 0xFF, 0xA2, 0x94,
	0x08, 0x04, 0x7E, 0x04, 0x08,
	0x10, 0x20, 0x7E, 0x20, 0x10,
	0x08, 0x08, 0x2A, 0x1C, 0x08,
	0x08, 0x1C, 0x2A, 0x08, 0x08,
	0x1E, 0x10, 0x10, 0x10, 0x10,
	0x0C, 0x1E, 0x0C, 0x1E, 0x0C,
	0x30, 0x38, 0x3E, 0x38, 0x30,
	0x06, 0x0E, 0x3E, 0x0E, 0x06,
	0x00, 0x00, 0x00, 0x00, 0x00, /* chr: ' '  (2 wide) */
0x00, 0x00, 0x5e, 0x00, 0x00, /* chr: '!'  (1 wide) */
0x06, 0x00, 0x06, 0x00, 0x00, /* chr: '"'  (3 wide) */
0x14, 0x3e, 0x14, 0x3e, 0x14, /* chr: '#'  (5 wide) */
0x24, 0x2a, 0x7e, 0x2a, 0x12, /* chr: '$'  (5 wide) */
0x44, 0x20, 0x10, 0x08, 0x44, /* chr: '%'  (5 wide) */
0x34, 0x4a, 0x54, 0x20, 0x50, /* chr: '&'  (5 wide) */
0x04, 0x02, 0x00, 0x00, 0x00, /* chr: '''  (2 wide) */
0x3c, 0x42, 0x00, 0x00, 0x00, /* chr: '('  (2 wide) */
0x42, 0x3c, 0x00, 0x00, 0x00, /* chr: ')'  (2 wide) */
0x54, 0x38, 0x38, 0x54, 0x00, /* chr: '*'  (4 wide) */
0x10, 0x38, 0x10, 0x00, 0x00, /* chr: '+'  (3 wide) */
0x40, 0x20, 0x00, 0x00, 0x00, /* chr: ','  (2 wide) */
0x10, 0x10, 0x10, 0x00, 0x00, /* chr: '-'  (3 wide) */
0x40, 0x00, 0x00, 0x00, 0x00, /* chr: '.'  (1 wide) */
0x40, 0x20, 0x10, 0x08, 0x04, /* chr: '/'  (5 wide) */
0x3c, 0x52, 0x4a, 0x3c, 0x00, /* chr: '0'  (4 wide) */
0x44, 0x7e, 0x40, 0x00, 0x00, /* chr: '1'  (3 wide) */
0x64, 0x52, 0x4a, 0x44, 0x00, /* chr: '2'  (4 wide) */
0x42, 0x4a, 0x4a, 0x34, 0x00, /* chr: '3'  (4 wide) */
0x1e, 0x10, 0x7c, 0x10, 0x00, /* chr: '4'  (4 wide) */
0x4e, 0x4a, 0x4a, 0x32, 0x00, /* chr: '5'  (4 wide) */
0x3c, 0x4a, 0x4a, 0x32, 0x00, /* chr: '6'  (4 wide) */
0x02, 0x72, 0x0a, 0x06, 0x00, /* chr: '7'  (4 wide) */
0x34, 0x4a, 0x4a, 0x34, 0x00, /* chr: '8'  (4 wide) */
0x0c, 0x52, 0x52, 0x3c, 0x00, /* chr: '9'  (4 wide) */
0x24, 0x00, 0x00, 0x00, 0x00, /* chr: ':'  (1 wide) */
0x40, 0x24, 0x00, 0x00, 0x00, /* chr: ';'  (2 wide) */
0x10, 0x28, 0x44, 0x00, 0x00, /* chr: '<'  (3 wide) */
0x28, 0x28, 0x28, 0x00, 0x00, /* chr: '='  (3 wide) */
0x44, 0x28, 0x10, 0x00, 0x00, /* chr: '>'  (3 wide) */
0x04, 0x52, 0x0a, 0x04, 0x00, /* chr: '?'  (4 wide) */
0x38, 0x04, 0x34, 0x44, 0x38, /* chr: '@'  (5 wide) */
0x7c, 0x12, 0x12, 0x7c, 0x00, /* chr: 'A'  (4 wide) */
0x7e, 0x4a, 0x4a, 0x34, 0x00, /* chr: 'B'  (4 wide) */
0x3c, 0x42, 0x42, 0x24, 0x00, /* chr: 'C'  (4 wide) */
0x7e, 0x42, 0x42, 0x3c, 0x00, /* chr: 'D'  (4 wide) */
0x7e, 0x4a, 0x4a, 0x42, 0x00, /* chr: 'E'  (4 wide) */
0x7e, 0x0a, 0x0a, 0x02, 0x00, /* chr: 'F'  (4 wide) */
0x3c, 0x42, 0x52, 0x34, 0x00, /* chr: 'G'  (4 wide) */
0x7e, 0x08, 0x08, 0x7e, 0x00, /* chr: 'H'  (4 wide) */
0x00, 0x00, 0x7e, 0x00, 0x00, /* chr: 'I'  (1 wide) */
0x20, 0x40, 0x3e, 0x00, 0x00, /* chr: 'J'  (3 wide) */
0x7e, 0x08, 0x14, 0x22, 0x40, /* chr: 'K'  (5 wide) */
0x7e, 0x40, 0x40, 0x40, 0x00, /* chr: 'L'  (4 wide) */
0x7e, 0x04, 0x08, 0x04, 0x7e, /* chr: 'M'  (5 wide) */
0x7e, 0x04, 0x08, 0x10, 0x7e, /* chr: 'N'  (5 wide) */
0x3c, 0x42, 0x42, 0x3c, 0x00, /* chr: 'O'  (4 wide) */
0x7e, 0x12, 0x12, 0x0c, 0x00, /* chr: 'P'  (4 wide) */
0x3c, 0x42, 0x22, 0x5c, 0x00, /* chr: 'Q'  (4 wide) */
0x7e, 0x12, 0x32, 0x4c, 0x00, /* chr: 'R'  (4 wide) */
0x24, 0x4a, 0x52, 0x24, 0x00, /* chr: 'S'  (4 wide) */
0x02, 0x02, 0x7e, 0x02, 0x02, /* chr: 'T'  (5 wide) */
0x3e, 0x40, 0x40, 0x3e, 0x00, /* chr: 'U'  (4 wide) */
0x0e, 0x30, 0x40, 0x30, 0x0e, /* chr: 'V'  (5 wide) */
0x3e, 0x40, 0x20, 0x40, 0x3e, /* chr: 'W'  (5 wide) */
0x42, 0x24, 0x18, 0x24, 0x42, /* chr: 'X'  (5 wide) */
0x06, 0x08, 0x70, 0x08, 0x06, /* chr: 'Y'  (5 wide) */
0x62, 0x52, 0x4a, 0x46, 0x00, /* chr: 'Z'  (4 wide) */
0x7e, 0x42, 0x00, 0x00, 0x00, /* chr: '['  (2 wide) */
0x04, 0x08, 0x10, 0x20, 0x40, /* chr: '\'  (5 wide) */
0x42, 0x7e, 0x00, 0x00, 0x00, /* chr: ']'  (2 wide) */
0x04, 0x02, 0x04, 0x00, 0x00, /* chr: '^'  (3 wide) */
0x40, 0x40, 0x40, 0x40, 0x00,  /* chr: '_'  (4 wide) */
	0x00, 0x03, 0x07, 0x08, 0x00,
	0x20, 0x54, 0x54, 0x78, 0x40,
	0x7F, 0x28, 0x44, 0x44, 0x38,
	0x38, 0x44, 0x44, 0x44, 0x28,
	0x38, 0x44, 0x44, 0x28, 0x7F,
	0x38, 0x54, 0x54, 0x54, 0x18,
	0x00, 0x08, 0x7E, 0x09, 0x02,
	0x18, 0xA4, 0xA4, 0x9C, 0x78,
	0x7F, 0x08, 0x04, 0x04, 0x78,
	0x00, 0x44, 0x7D, 0x40, 0x00,
	0x20, 0x40, 0x40, 0x3D, 0x00,
	0x7F, 0x10, 0x28, 0x44, 0x00,
	0x00, 0x41, 0x7F, 0x40, 0x00,
	0x7C, 0x04, 0x78, 0x04, 0x78,
	0x7C, 0x08, 0x04, 0x04, 0x78,
	0x38, 0x44, 0x44, 0x44, 0x38,
	0xFC, 0x18, 0x24, 0x24, 0x18,
	0x18, 0x24, 0x24, 0x18, 0xFC,
	0x7C, 0x08, 0x04, 0x04, 0x08,
	0x48, 0x54, 0x54, 0x54, 0x24,
	0x04, 0x04, 0x3F, 0x44, 0x24,
	0x3C, 0x40, 0x40, 0x20, 0x7C,
	0x1C, 0x20, 0x40, 0x20, 0x1C,
	0x3C, 0x40, 0x30, 0x40, 0x3C,
	0x44, 0x28, 0x10, 0x28, 0x44,
	0x4C, 0x90, 0x90, 0x90, 0x7C,
	0x44, 0x64, 0x54, 0x4C, 0x44,
	0x00, 0x08, 0x36, 0x41, 0x00,
	0x00, 0x00, 0x77, 0x00, 0x00,
	0x00, 0x41, 0x36, 0x08, 0x00,
	0x02, 0x01, 0x02, 0x04, 0x02,
	0x3C, 0x26, 0x23, 0x26, 0x3C,
	0x1E, 0xA1, 0xA1, 0x61, 0x12,
	0x3A, 0x40, 0x40, 0x20, 0x7A,
	0x38, 0x54, 0x54, 0x55, 0x59,
	0x21, 0x55, 0x55, 0x79, 0x41,
	0x22, 0x54, 0x54, 0x78, 0x42, // a-umlaut
	0x21, 0x55, 0x54, 0x78, 0x40,
	0x20, 0x54, 0x55, 0x79, 0x40,
	0x0C, 0x1E, 0x52, 0x72, 0x12,
	0x39, 0x55, 0x55, 0x55, 0x59,
	0x39, 0x54, 0x54, 0x54, 0x59,
	0x39, 0x55, 0x54, 0x54, 0x58,
	0x00, 0x00, 0x45, 0x7C, 0x41,
	0x00, 0x02, 0x45, 0x7D, 0x42,
	0x00, 0x01, 0x45, 0x7C, 0x40,
	0x7D, 0x12, 0x11, 0x12, 0x7D, // A-umlaut
	0xF0, 0x28, 0x25, 0x28, 0xF0,
	0x7C, 0x54, 0x55, 0x45, 0x00,
	0x20, 0x54, 0x54, 0x7C, 0x54,
	0x7C, 0x0A, 0x09, 0x7F, 0x49,
	0x32, 0x49, 0x49, 0x49, 0x32,
	0x3A, 0x44, 0x44, 0x44, 0x3A, // o-umlaut
	0x32, 0x4A, 0x48, 0x48, 0x30,
	0x3A, 0x41, 0x41, 0x21, 0x7A,
	0x3A, 0x42, 0x40, 0x20, 0x78,
	0x00, 0x9D, 0xA0, 0xA0, 0x7D,
	0x3D, 0x42, 0x42, 0x42, 0x3D, // O-umlaut
	0x3D, 0x40, 0x40, 0x40, 0x3D,
	0x3C, 0x24, 0xFF, 0x24, 0x24,
	0x48, 0x7E, 0x49, 0x43, 0x66,
	0x2B, 0x2F, 0xFC, 0x2F, 0x2B,
	0xFF, 0x09, 0x29, 0xF6, 0x20,
	0xC0, 0x88, 0x7E, 0x09, 0x03,
	0x20, 0x54, 0x54, 0x79, 0x41,
	0x00, 0x00, 0x44, 0x7D, 0x41,
	0x30, 0x48, 0x48, 0x4A, 0x32,
	0x38, 0x40, 0x40, 0x22, 0x7A,
	0x00, 0x7A, 0x0A, 0x0A, 0x72,
	0x7D, 0x0D, 0x19, 0x31, 0x7D,
	0x26, 0x29, 0x29, 0x2F, 0x28,
	0x26, 0x29, 0x29, 0x29, 0x26,
	0x30, 0x48, 0x4D, 0x40, 0x20,
	0x38, 0x08, 0x08, 0x08, 0x08,
	0x08, 0x08, 0x08, 0x08, 0x38,
	0x2F, 0x10, 0xC8, 0xAC, 0xBA,
	0x2F, 0x10, 0x28, 0x34, 0xFA,
	0x00, 0x00, 0x7B, 0x00, 0x00,
	0x08, 0x14, 0x2A, 0x14, 0x22,
	0x22, 0x14, 0x2A, 0x14, 0x08,
	0x55, 0x00, 0x55, 0x00, 0x55, // #176 (25% block) missing in old code
	0xAA, 0x55, 0xAA, 0x55, 0xAA, // 50% block
	0xFF, 0x55, 0xFF, 0x55, 0xFF, // 75% block
	0x00, 0x00, 0x00, 0xFF, 0x00,
	0x10, 0x10, 0x10, 0xFF, 0x00,
	0x14, 0x14, 0x14, 0xFF, 0x00,
	0x10, 0x10, 0xFF, 0x00, 0xFF,
	0x10, 0x10, 0xF0, 0x10, 0xF0,
	0x14, 0x14, 0x14, 0xFC, 0x00,
	0x14, 0x14, 0xF7, 0x00, 0xFF,
	0x00, 0x00, 0xFF, 0x00, 0xFF,
	0x14, 0x14, 0xF4, 0x04, 0xFC,
	0x14, 0x14, 0x17, 0x10, 0x1F,
	0x10, 0x10, 0x1F, 0x10, 0x1F,
	0x14, 0x14, 0x14, 0x1F, 0x00,
	0x10, 0x10, 0x10, 0xF0, 0x00,
	0x00, 0x00, 0x00, 0x1F, 0x10,
	0x10, 0x10, 0x10, 0x1F, 0x10,
	0x10, 0x10, 0x10, 0xF0, 0x10,
	0x00, 0x00, 0x00, 0xFF, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0xFF, 0x10,
	0x00, 0x00, 0x00, 0xFF, 0x14,
	0x00, 0x00, 0xFF, 0x00, 0xFF,
	0x00, 0x00, 0x1F, 0x10, 0x17,
	0x00, 0x00, 0xFC, 0x04, 0xF4,
	0x14, 0x14, 0x17, 0x10, 0x17,
	0x14, 0x14, 0xF4, 0x04, 0xF4,
	0x00, 0x00, 0xFF, 0x00, 0xF7,
	0x14, 0x14, 0x14, 0x14, 0x14,
	0x14, 0x14, 0xF7, 0x00, 0xF7,
	0x14, 0x14, 0x14, 0x17, 0x14,
	0x10, 0x10, 0x1F, 0x10, 0x1F,
	0x14, 0x14, 0x14, 0xF4, 0x14,
	0x10, 0x10, 0xF0, 0x10, 0xF0,
	0x00, 0x00, 0x1F, 0x10, 0x1F,
	0x00, 0x00, 0x00, 0x1F, 0x14,
	0x00, 0x00, 0x00, 0xFC, 0x14,
	0x00, 0x00, 0xF0, 0x10, 0xF0,
	0x10, 0x10, 0xFF, 0x10, 0xFF,
	0x14, 0x14, 0x14, 0xFF, 0x14,
	0x10, 0x10, 0x10, 0x1F, 0x00,
	0x00, 0x00, 0x00, 0xF0, 0x10,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
	0xFF, 0xFF, 0xFF, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xFF, 0xFF,
	0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
	0x38, 0x44, 0x44, 0x38, 0x44,
	0xFC, 0x4A, 0x4A, 0x4A, 0x34, // sharp-s or beta
	0x7E, 0x02, 0x02, 0x06, 0x06,
	0x02, 0x7E, 0x02, 0x7E, 0x02,
	0x63, 0x55, 0x49, 0x41, 0x63,
	0x38, 0x44, 0x44, 0x3C, 0x04,
	0x40, 0x7E, 0x20, 0x1E, 0x20,
	0x06, 0x02, 0x7E, 0x02, 0x02,
	0x99, 0xA5, 0xE7, 0xA5, 0x99,
	0x1C, 0x2A, 0x49, 0x2A, 0x1C,
	0x4C, 0x72, 0x01, 0x72, 0x4C,
	0x30, 0x4A, 0x4D, 0x4D, 0x30,
	0x30, 0x48, 0x78, 0x48, 0x30,
	0xBC, 0x62, 0x5A, 0x46, 0x3D,
	0x3E, 0x49, 0x49, 0x49, 0x00,
	0x7E, 0x01, 0x01, 0x01, 0x7E,
	0x2A, 0x2A, 0x2A, 0x2A, 0x2A,
	0x44, 0x44, 0x5F, 0x44, 0x44,
	0x40, 0x51, 0x4A, 0x44, 0x40,
	0x40, 0x44, 0x4A, 0x51, 0x40,
	0x00, 0x00, 0xFF, 0x01, 0x03,
	0xE0, 0x80, 0xFF, 0x00, 0x00,
	0x08, 0x08, 0x6B, 0x6B, 0x08,
	0x36, 0x12, 0x36, 0x24, 0x36,
	0x06, 0x0F, 0x09, 0x0F, 0x06,
	0x00, 0x00, 0x18, 0x18, 0x00,
	0x00, 0x00, 0x10, 0x10, 0x00,
	0x30, 0x40, 0xFF, 0x01, 0x01,
	0x00, 0x1F, 0x01, 0x01, 0x1E,
	0x00, 0x19, 0x1D, 0x17, 0x12,
	0x00, 0x3C, 0x3C, 0x3C, 0x3C,
	0x00, 0x00, 0x00, 0x00, 0x00  // #255 NBSP
};
#endif // FONT5X7_H