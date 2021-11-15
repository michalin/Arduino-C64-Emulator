#include <Arduino.h>

#ifndef KEYMAP_H
#define KEYMAP_H

#define BREAK 0xF0 //Key released
#define DTA_PIN 10 //Data
#define CLK_PIN 11 //Clock
//#define PCINT_vect PCINT0_vect //Depends on data pin

#define KBDISABLE()\
    pinMode(CLK_PIN, OUTPUT);\
    digitalWrite(CLK_PIN, LOW);

#define KBENABLE()\
    pinMode(CLK_PIN, INPUT_PULLUP);

void kb_init();
void on_keypressed(uint8_t code);
#endif