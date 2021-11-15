/*  Arduino sketch that emulates a Commodore 64
    Copyright (C) 2021  Doctor Volt

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#include "keyboard.h"
#include "cpu.h"
#include "char_rom.h"

// Change pins here, if necessary
#if defined ARDUINO_AVR_MEGA2560
#include "bitmaps.h"
#define SYN_DD_REG DDRE // Bit in PORT that is connected for sync
#define SYNCPIN PE4     // Sync pulse (Pin 2)
#define BLNKPIN PE5     // Blanking / Black level (Pin 3)
#else
#define SYN_DD_REG DDRD // Data Direction register
#define SYNCPIN PD2     // Sync pulse (Pin 2)
#define BLNKPIN PD3     // Blanking / Black level (Pin3)
#endif

//#define NTSC //uncomment this, if your (CRT) TV shows never the same color (NTSC). However with PAL the thing has better performance
#ifdef NTSC
#define HSYNC 126  // Hsync frequency (divided from Fcpu)
#define LINES 262  // Lines per field
#define START_L 41 // First computer line (Margin from top)
#else
#define HSYNC 127 // Hsync frequency (divided from Fcpu)
#define LINES 311 // Lines per field
#define START_L 69
#endif

#define CPL 40 // Characters per line
#define END_L START_L + 199
#define VSYNCLEN 8 // Length of VSYNC at start of frame

volatile byte VBE = 0; // Set this to 1 to let the CPU sleep while shifting out characters.
                       // This smoothens the image

#define DELAY500 asm("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"); // Every NOP delays for 62.5ns

uint16_t scanline = 0;
uint32_t frame_nr = 0;

void cursor() // Emulate Cursor here for better performance
{
  uint8_t &phase = sysram[0xCF]; // 0 if reversed, 1 if not reversed
  uint8_t &cuc = sysram[0xCE];   // Character under cursor
  uint16_t pos = ((sysram[0xD2] << 8) + sysram[0xD1] - VIDEOADDR) + sysram[0xD3];
  phase ^= 1;
  if (phase)
    cuc = videomem[pos];
  videomem[pos] = cuc | (phase << 7); // Invert it
}

ISR(TIMER0_COMPA_vect) // Raster interrupt. This is called for every scanline (64ms)
{
  bitSet(SYN_DD_REG, BLNKPIN); // Start blanking interval (Front porch)
  DELAY500;
  DELAY500;
  DELAY500;                    // Front porch
  bitSet(SYN_DD_REG, SYNCPIN); // Start sync
  scanline >= LINES ? scanline = 0 : scanline++;
  if (scanline >= VSYNCLEN) // Hor. Sync interval
  {
    // Draw visible line
    delayMicroseconds(4);          // Length of Sync Pulse
    bitClear(SYN_DD_REG, SYNCPIN); // End of sync / Start back Porch
    delayMicroseconds(7);          // Length of back Porch
    bitClear(SYN_DD_REG, BLNKPIN); // End of back porch / Start visible part of line

    if ((scanline >= START_L) && (scanline <= END_L)) // Screen area with characters
    {
      uint16_t line = scanline - START_L;

      delayMicroseconds(2); // H-align
      DELAY500;
#if defined ARDUINO_AVR_MEGA2560
      uint16_t start = (line >> 3) * 320 + (line & 0x07);
      uint8_t *p_bitmap = (uint8_t *)bitmaps[bnr] + start;
      if (vic2.scr1 & 32)
      { // Hires bitmap mode
        for (uint16_t p = 0; p < 320; p += 8)
        {
          UDR0 = pgm_read_byte(p_bitmap + p);
          asm("nop\n");
        }
        sei();
        asm("sleep\n");
      }
      else
#endif
      {                                                   // Text mode
        uint16_t firstchr = (line >> 3) * CPL;            // Index of first character
        uint8_t register *p_vidmem = &videomem[firstchr]; // Pointer to first character in current line
        for (uint16_t p = 0; p < CPL; p++)
        {
          UDR0 = pgm_read_byte_near(&charROM[line & 0x07][*(p_vidmem++)]);
          // while (!(UCSR0A & _BV(UDRE0)));
        }
      }
      //return;
    }
    else if (scanline == START_L - 1) // Last frame line before output
    {
      VBE = 1; // Sleep CPU
      KBDISABLE();
    }
    else if (scanline == END_L + 1) // First frame line after output
    {
      VBE = 0;
      KBENABLE();
    }
  }
  else if (scanline == 1 && !sysram[0xCC]) // Cursor enabled
  {
    if (!sysram[0xCD]--)
    {
      cursor();
      sysram[0xCD] = 20; // Cursor blink phase
    }
  }
}

void on_keypressed(uint8_t code) // Called when key pressed or released
{
  if (code)
  {
    if (code == 3) // Stop (Esc) key
    {
      stop();
      return;
    }
    sysram[0x277] = code;
    sysram[0xC6] = 1;
    scanline += 16; // Reduces flickering*/
  }
  else              // Key released
    scanline += 58; // Reduces flickering*/
}

void setup()
{
  pinMode(13, OUTPUT); // Debug pin
  TCCR0A = 0;
  TCCR0B = 0;
  OCR0A = HSYNC;         // = (16MHz) / (15625*8) - 1
  bitSet(TCCR0A, WGM01); // CTC mode
  bitSet(TCCR0B, CS01);  // clk/8
  set_sleep_mode(SLEEP_MODE_IDLE);
  UCSR0C = _BV(UMSEL00) | _BV(UMSEL01); // Set USART to SPI Mode
  UBRR0 = 0;                            // 8 MHz pixel clock
  bitSet(UCSR0B, TXEN0);

  resetCPU();
  while (videomem[0] != 32) // Boot the C64
    execCPU();

  //vic2.scr1 = 32; // Start with slide show (Mega only)
  kb_init();
  bitSet(TIMSK0, OCIE0A); // Start screen output
}

void loop()
{
  while (VBE == 1) // Let CPU sleep between the scanlines
    asm("sleep\n");
  execCPU();
}
