#include "tiger.h"
#include "like.h"
#include "subscribe.h"
#include "bell.h"
#include "lambo.h"

uint8_t bnr = 0; //Index of bitmap to display in Hi-res mode

const uint8_t *bitmaps[5] = {
    &hires0[0], &hires1[0], &hires2[0], &hires3[0], &hires4[0]
};
