#ifndef CPU_H
#define CPU_H

//#include <Arduino.h>
#include <EEPROM.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define VIDEOADDR 0x400  //Start of video buffer
#define VIDEOLEN 1024   //Size of video buffer (Byte)

#if defined ARDUINO_AVR_MEGA2560
struct VIC2{
  uint8_t scr1 = 0x1B; //Screen control register 1
  uint8_t msr = 0x15;  //Memory setup register
};
extern VIC2 vic2;
#endif

void execCPU();
void resetCPU();
void intrq();
uint16_t getpc();
uint8_t read(uint16_t address);
void write(uint16_t address, uint8_t value);
extern uint8_t sysram[];
extern uint8_t videomem[];
//extern uint8_t DDR; //Port data direction register
//extern uint8_t PORT; //Port data register

void stop();

#endif
