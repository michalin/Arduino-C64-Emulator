/*  PS/2 Keyboard driver for Commodore 64 emulator
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
//Modifiers
#define IDLE 0
#define ALT_L 0x11
#define SHIFT 0x12
#define CTRL 0x14
#define COMMODORE ALT_L
#define CRSR 0xE0
#define ERR_TIMEOUT 127

#define DE //German Keyboard layout
#ifdef DE
const uint8_t scancodes[128] PROGMEM = { //Mapping between PS/2 scancodes and petscii codes
    0, 0, 0, 135, 134, 133, 137, 0, 0, 0, 140, 139, 138, 0, 39, 0, 0, 0, 0, 0,
    0, 81, 49, 0, 0, 0, 89, 83, 65, 87, 50, 0, 0, 67, 88, 68, 69, 52, 51, 0,
    0, 32, 86, 70, 84, 82, 53, 0, 0, 78, 66, 72, 71, 90, 54, 0, 0, 0, 77, 74,
    85, 55, 56, 0, 0, 44, 75, 73, 79, 48, 57, 0, 0, 46, 47, 76, 59, 80, 45, 0,
    0, 0, 64, 0, 91, 61, 0, 0, 0, 0, 13, 93, 0, 35, 0, 0, 0, 0, 0, 0,
    0, 0, 20, 0, 0, 0, 0, 95, 0, 0, 0, 0, 0, 0, 0, 0, 0, 94, 3, 0,
    0, 0, 0, 0, 0, 0, 0, ERR_TIMEOUT};
const uint8_t scancodes_l2[128] PROGMEM = { //Mapping if Shift pressed
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 113, 33, 0, 0, 0, 111, 115, 97, 119, 34, 0, 0, 99, 110, 100, 101, 36, 0, 0,
    0, 0, 118, 102, 116, 114, 37, 0, 0, 110, 98, 104, 103, 112, 38, 0, 0, 0, 109, 106,
    117, 47, 40, 0, 0, 59, 107, 105, 111, 61, 41, 0, 0, 58, 63, 108, 58, 112, 0, 0,
    0, 0, 0, 0, 0, 43, 0, 0, 0, 0, 0, 0, 0, 39, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 126, 0, 0,
    0, 0, 0, 0, 0, 0, 0, ERR_TIMEOUT};

#else //US Layout
const uint8_t scancodes[128] PROGMEM = { //Mapping between PS/2 scancodes and petscii codes
    0, 0, 0, 135, 134, 133, 137, 0, 0, 0, 140, 139, 138, 0, 39, 0, 0, 0, 0, 0,
    0, 81, 49, 0, 0, 0, 90, 83, 65, 87, 50, 0, 0, 67, 88, 68, 69, 52, 51, 0,
    0, 32, 86, 70, 84, 82, 53, 0, 0, 78, 66, 72, 71, 89, 54, 0, 0, 0, 77, 74,
    85, 55, 56, 0, 0, 44, 75, 73, 79, 48, 57, 0, 0, 46, 47, 76, 59, 80, 45, 0,
    0, 0, 39, 0, 91, 61, 0, 0, 0, 0, 13, 93, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 20, 0, 0, 0, 0, 95, 0, 0, 0, 0, 0, 0, 0, 0, 0, 94, 3, 0,
    0, 0, 0, 0, 0, 0, 0, ERR_TIMEOUT};
const uint8_t scancodes_l2[128] PROGMEM = { //Shift pressed
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 113, 33, 0, 0, 0, 112, 115, 97, 119, 64, 0, 0, 99, 110, 100, 101, 36, 35, 0,
    0, 0, 118, 102, 116, 114, 37, 0, 0, 110, 98, 104, 103, 111, 0, 0, 0, 0, 109, 106,
    117, 38, 42, 0, 0, 60, 107, 105, 111, 41, 40, 0, 0, 62, 63, 108, 58, 112, 0, 0,
    0, 0, 34, 0, 0, 43, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 126, 0, 0,
    0, 0, 0, 0, 0, 0, 0, ERR_TIMEOUT};
#endif

uint8_t kb_readByte() //Gets single frame from keyboard
{
  uint16_t scnval = 0;
  uint16_t timeout = 1000;
  for (uint8_t i = 0; i < 11; i++)
  {
    while(digitalRead(CLK_PIN))
    {
      if (timeout-- == 0)
        return ERR_TIMEOUT;
      delayMicroseconds(10);
    }
    scnval |= digitalRead(DTA_PIN) << i;

    timeout = 10;
    while(!digitalRead(CLK_PIN))
    {
      if (timeout-- == 0)
        return ERR_TIMEOUT;
      delayMicroseconds(10);
    }
  }
  scnval >>= 1;   //Shift out start bit
  scnval &= 0xFF; //Cut off parity and stop bit
  return scnval;
}

uint8_t kb_read()
{
  uint8_t code = 0, scnval = 0;
  static uint8_t modifier;

  uint8_t kbbyte = kb_readByte();
  switch (kbbyte)
  {
  case CRSR: //Cursor control
    scnval = kb_readByte();
    modifier = CRSR;
    if (scnval == BREAK)
    {
      kb_readByte();
      modifier = IDLE;
      scnval = 0;
    }
    break;
  case BREAK:
    kbbyte = kb_readByte();
    if (modifier && modifier == kbbyte)
      modifier = IDLE;
    break;
  case SHIFT: //Shift key
  case ALT_L: //-> Commodore
  case CTRL:
    modifier = kbbyte;
    break;
  case ERR_TIMEOUT:
    //scnval = ERR_TIMEOUT;
  break;
  default:
    scnval = kbbyte;
    break;
  }
  if (!scnval || scnval > 127)
    return 0;
#ifdef KBTEST
  Debug.print(DBG_INFO, "modifier: %x, value: %d", modifier, scnval);
#endif

  switch (modifier)
  {
  case CRSR:
    switch (scnval)
    {
    case 0x74: //CRSR RIGHT
      code = 29;
      break;
    case 0x72: //CRSR DN
      code = 17;
      break;
    case 0x6B: //CRSR LEFT
      code = 157;
      break;
    case 0x75: //CRSR UP
      code = 145;
      break;
    case 0x70: //INST
      code = 148;
      break;
    case 0x6C:   //POS1
      code = 19; //HOME
      break;
    case 0x71: //DEL
      code = 20;
      break;
    }
    break;
  case IDLE:
    code = pgm_read_byte(scancodes + scnval);
    break;
  case SHIFT:
    code = pgm_read_byte(scancodes_l2 + scnval);
    break;
  default:
    code = 0;
  }
#ifdef KBTEST
  Debug.print(DBG_INFO, "modifier: %x, petscii: %d", modifier, code);
#endif
  return code;
}

void irq()
{
  uint8_t val = kb_read();
  on_keypressed(val);
  uint8_t pcif = digitalPinToPCICRbit(CLK_PIN);
  bitSet(PCIFR, pcif); //Get rid of pending port interrupts
}
ISR(PCINT0_vect)
{
  irq();
}
ISR(PCINT1_vect)
{
  irq();
}
ISR(PCINT2_vect)
{
  irq();
}

void kb_init()
{
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DTA_PIN, INPUT_PULLUP);
  volatile uint8_t *pcicr = digitalPinToPCICR(DTA_PIN);
  uint8_t pcie = digitalPinToPCICRbit(DTA_PIN);
  bitSet(*pcicr, pcie);
  volatile uint8_t *pcmsk = digitalPinToPCMSK(DTA_PIN);
  uint8_t pcint = digitalPinToPCMSKbit(DTA_PIN);
  bitSet(*pcmsk, pcint); // Pin change interrupt on Clock pin
}
