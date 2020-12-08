#pragma once

#include <inttypes.h>
#include "color.h"

namespace Leds {

void Init();
void SetBrightness(uint8_t Brt);

// Buf of 7 pixels
void ShowPic(Color_t *PClr);
}
